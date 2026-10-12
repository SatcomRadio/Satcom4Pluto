// feed words (hex per line) -> rebuilt samples (cf32) ; noise fill off
#include "sgch_rx.hpp"
#include <cstdio>
#include <vector>
int main(int argc, char **argv) {
  sgch::Config c; c.uri = "offline"; c.noise_fill = false;
  sgch::Receiver rx(c);
  FILE *f = fopen(argv[1], "r"); unsigned v; std::vector<uint32_t> w;
  while (fscanf(f, "%x", &v) == 1) w.push_back(v);
  rx.feed_words(w.data(), w.size());
  FILE *o = fopen(argv[2], "wb");
  std::vector<std::complex<float>> b(1 << 16);
  size_t k;
  while ((k = rx.read(b.data(), b.size(), 1000)) > 0) fwrite(b.data(), 8, k, o);
  fclose(o);
  auto st = rx.stats();
  printf("frames %llu lost %llu resyncs %llu\n", (unsigned long long)st.frames, (unsigned long long)st.lost_frames, (unsigned long long)st.resyncs);
}
