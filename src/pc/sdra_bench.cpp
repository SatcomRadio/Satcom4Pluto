// test client: connect like SDRangel's Remote TCP Input and measure the stream.
//   sdra_bench [port] [seconds] [out.cs16]
// Raw IQ (protocol revision 0 / IQ-only) and framed messages (flags bit 7:
// dataIQzlib + setCenterFrequency, used by the channel slots) are both handled;
// zlib payloads are checked (stored blocks + Adler-32) and unpacked.
#include "net.hpp"
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
static uint32_t g32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
int main(int argc, char **argv) {
  int port = argc > 1 ? atoi(argv[1]) : 1234;
  double secs = argc > 2 ? atof(argv[2]) : 10;
  FILE *fo = argc > 3 ? fopen(argv[3], "wb") : nullptr;
  sock_t s = net::connect_tcp("127.0.0.1", port, 5000);
  if (s == SGCH_BADSOCK) { printf("cannot connect\n"); return 1; }
  net::set_bufsize(s, 8 << 20);
  net::Reader rd(s);
  uint8_t h[128];
  if (!rd.read(h, 128)) { printf("no header\n"); return 1; }
  uint32_t flags = g32(h + 20), rate = g32(h + 24), bits = g32(h + 56);
  uint64_t f0 = (uint64_t)g32(h + 8) << 32 | g32(h + 12);
  bool framed = (flags >> 7) & 1;
  printf("%.4s device %u, %.6f MHz, %u S/s, %u bits, %s\n", (char *)h, g32(h + 4), f0 / 1e6, rate, bits,
         framed ? "framed (zlib)" : "raw IQ");
  long long t = 0, bad = 0, msgs = 0, freqs = 0;
  auto t0 = std::chrono::steady_clock::now();
  double dt = 0;
  std::vector<uint8_t> b(4 << 20);
  while (dt < secs) {
    if (!framed) {
      long k = net::recv_some(s, (char *)b.data(), b.size());
      if (k <= 0) break;
      t += k;
      if (fo) fwrite(b.data(), 1, k, fo);
    } else {
      uint8_t c[5];
      if (!rd.read(c, 5)) break;
      if (c[0] == 0x01) { freqs++; printf("  setCenterFrequency %.6f MHz\n", g32(c + 1) / 1e6); }
      else if (c[0] == 0xf2) {
        uint32_t n = g32(c + 1);
        if (n > b.size() || !rd.read(b.data(), n)) break;
        msgs++;
        // 78 01 | 01 LEN NLEN | data | adler
        uint32_t len = b[3] | b[4] << 8, nlen = b[5] | b[6] << 8;
        bool ok = b[0] == 0x78 && b[2] == 0x01 && (len ^ nlen) == 0xFFFF && len + 11 == n;
        uint32_t a = 1, bb = 0;
        for (uint32_t i = 0; i < len; i++) { a = (a + b[7 + i]) % 65521; bb = (bb + a) % 65521; }
        ok = ok && g32(b.data() + 7 + len) == ((bb << 16) | a);
        if (!ok) bad++;
        t += len;
        if (fo) fwrite(b.data() + 7, 1, len, fo);
      } else { printf("  unexpected message 0x%02x\n", c[0]); break; }
    }
    dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }
  int bps = bits == 8 ? 2 : bits == 16 ? 4 : bits == 24 ? 6 : 8;
  printf("%lld bytes in %.2f s = %.4f MS/s (%u-bit IQ, nominal %.4f)%s", t, dt, t / (double)bps / 1e6 / dt, bits,
         rate / 1e6, framed ? "" : "\n");
  if (framed) printf("; %lld zlib messages, %lld bad, %lld frequency changes\n", msgs, bad, freqs);
  if (fo) fclose(fo);
  return 0;
}
