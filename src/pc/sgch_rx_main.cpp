// SPDX-License-Identifier: MIT
// sgch_rx: rebuild the wideband stream from the Pluto channelizer and
//   * write it to a file or stdout (cf32 / cs16), or
//   * serve it to SDRangel's "Remote TCP Input" (SDRA protocol, --server), and/or
//   * serve every detected downlink as its own narrowband IQ stream (--channels,
//     one SDRangel Remote TCP Input per slot, ports 7301, 7302, ...).
#include "channels.hpp"
#include "net.hpp"
#include "sgch_rx.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

static volatile bool g_stop = false;
static bool g_remote_ctl = false;   // --server/--rtltcp: let the client change LO / rate / gain / bandwidth
static bool g_rtl = false;          // --rtltcp: plain rtl_tcp protocol (OpenWebRX, SDRangel "RTL0", GQRX...)
static double g_rtl_gain_db = 1e9;  // --rtl-gain-db: digital gain before 8 bits (1e9 = automatic)
static sgch::ChannelMux *volatile g_mux = nullptr;
static void on_sig(int) { g_stop = true; }

// ---------------------------------------------------------------------------
// SDRangel Remote TCP (SDRA) server
namespace {

void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = (uint8_t)v; }
void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)(v >> 32)); put32(p + 4, (uint32_t)v); }
void put16(uint8_t *p, int16_t v) { p[0] = (uint16_t)v >> 8; p[1] = (uint8_t)v; }
uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

struct Session {
  sgch::Receiver &rx;
  sock_t s;
  std::atomic<int> bits{16};
  std::atomic<bool> alive{true};
  bool verbose;
  bool rtl = false;
};

void command_loop(Session *ss) {
  net::Reader rd(ss->s);
  uint8_t c[5];
  volatile bool keep = true;
  while (ss->alive && !g_stop) {
    if (!rd.read(c, 5, &keep)) break;
    uint32_t v = get32(c + 1);
    if (ss->rtl) {
      // rtl_tcp: 0x01 frequency, 0x02 sample rate, 0x03 gain mode (1 = manual), 0x04 gain (tenths dB),
      // 0x08 AGC; everything else is RTL2832-specific
      if (!g_remote_ctl) {
        if (ss->verbose) fprintf(stderr, "sgch_rx: ignored rtl_tcp command 0x%02x %u (radio locked)\n", c[0], v);
        continue;
      }
      switch (c[0]) {
        case 0x01: ss->rx.set_frequency((double)v); break;
        case 0x02: ss->rx.set_sample_rate((double)v); break;
        case 0x03: ss->rx.set_agc(v == 0); break;
        case 0x04: ss->rx.set_agc(false); ss->rx.set_gain((int32_t)v / 10.0); break;
        case 0x08: ss->rx.set_agc(v != 0); break;
        default: break;
      }
      continue;
    }
    bool radio = c[0] == 0x01 || c[0] == 0x02 || c[0] == 0x03 || c[0] == 0x04 || c[0] == 0x08 || c[0] == 0x40;
    if (radio && !g_remote_ctl) {   // radio settings are fixed by the command line (see --remote-control)
      if (ss->verbose) fprintf(stderr, "sgch_rx: ignored SDRangel command 0x%02x %u (radio locked)\n", c[0], v);
      continue;
    }
    switch (c[0]) {
      case 0x01: ss->rx.set_frequency((double)v); break;                     // centre frequency
      case 0x02: ss->rx.set_sample_rate((double)v); break;                   // device sample rate
      case 0x03: ss->rx.set_agc(v == 0); break;                              // 0 = auto, 1 = manual
      case 0x04: ss->rx.set_agc(false); ss->rx.set_gain((int32_t)v / 10.0); break;   // tenths of a dB
      case 0x08: ss->rx.set_agc(v != 0); break;                              // AGC on/off
      case 0x40: if (v) ss->rx.set_bandwidth((double)v); break;              // RF bandwidth
      case 0xc6:                                                             // bits per sample
        if (v == 8 || v == 16 || v == 24 || v == 32) ss->bits = (int)v;
        break;
      case 0xc2:
        if (v) fprintf(stderr, "sgch_rx: SDRangel asked for decimation 2^%u -- set Decimation to 1 in SDRangel\n", v);
        break;
      default: break;   // ppm, bias tee, DC/IQ correction, squelch...: not used
    }
    if (ss->verbose) fprintf(stderr, "sgch_rx: command 0x%02x %u\n", c[0], v);
  }
  ss->alive = false;
}

void send_header(sgch::Receiver &rx, sock_t s) {
  uint8_t h[128];
  memset(h, 0, sizeof(h));
  memcpy(h, "SDRA", 4);
  put32(h + 4, 0x99);   // device: USRP (wide sample-rate range in the SDRangel GUI)
  double f = rx.get_frequency(), fs = rx.get_sample_rate(), g = rx.get_gain(), bw = rx.get_bandwidth();
  bool agc = rx.get_agc();
  put64(h + 8, (uint64_t)llround(f));
  put32(h + 16, 0);                                // ppm
  // flags: agc; bit 6 = remote control allowed (SDRangel greys its radio controls without it)
  put32(h + 20, (agc ? 1u << 2 : 0) | (g_remote_ctl ? 1u << 6 : 0));
  put32(h + 24, (uint32_t)llround(fs));
  put32(h + 28, 0);                                // log2 decimation
  put16(h + 32, (int16_t)lround(g * 10));
  put32(h + 40, (uint32_t)llround(bw));
  put32(h + 44, 0);                                // channel frequency offset
  put32(h + 48, 0);                                // channel gain
  put32(h + 52, (uint32_t)llround(fs));            // channel sample rate = device rate
  put32(h + 56, 16);                               // bits
  put32(h + 60, 1);                                // protocol revision
  float sq = -100.0f, gate = 0.001f;
  uint32_t u;
  memcpy(&u, &sq, 4); put32(h + 64, u);
  memcpy(&u, &gate, 4); put32(h + 68, u);
  net::send_all(s, h, sizeof(h));
  fprintf(stderr, "sgch_rx: SDRangel connected: %.6f MHz, %.3f MS/s, gain %.0f dB%s\n", f / 1e6, fs / 1e6, g,
          agc ? " (AGC)" : "");
}

// poll the sgchd track list and drive the channel slots
void track_poller(sgch::Receiver *rx, sgch::ChannelMux *mux, double period_s) {
  int fails = 0;
  while (!g_stop) {
    std::string r = rx->ctl_command("TRACKS");
    std::vector<sgch::TrackInfo> t;
    double lo = 0, fs = 0, binw = 0;
    if (sgch::parse_tracks(r, t, &lo, &fs, &binw) && binw > 0) {
      mux->set_radio(lo, fs, (int)lround(fs / binw));
      mux->update_tracks(t);
      fails = 0;
    } else if (++fails == 10) {
      fprintf(stderr, "sgch_rx: sgchd gives no track list (%s) -- detector v2 needs sgchd >= 2.0 "
                      "(not in --all / --default mode)\n", r.substr(0, 80).c_str());
    }
    for (int i = 0; i < (int)(period_s * 20) && !g_stop; i++) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

bool g_keep_rx = false;   // channel slots need the receiver running all the time

int run_server(sgch::Receiver &rx, const std::string &bind, int port, bool verbose) {
  sock_t ls = net::listen_tcp(bind, port);
  if (ls == SGCH_BADSOCK) {
    fprintf(stderr, "sgch_rx: cannot listen on %s:%d\n", bind.c_str(), port);
    return 1;
  }
  net::set_timeout(ls, 500);
  if (g_rtl)
    fprintf(stderr, "sgch_rx: rtl_tcp server on %s:%d (OpenWebRX: rtl_tcp device; SDRangel: protocol RTL0)\n",
            bind.c_str(), port);
  else
    fprintf(stderr, "sgch_rx: waiting for SDRangel (Remote TCP Input -> %s:%d, protocol SDRangel)\n",
            bind == "0.0.0.0" ? "this PC" : bind.c_str(), port);
  std::vector<std::complex<float>> b(16384);
  std::vector<uint8_t> o(b.size() * 8);
  while (!g_stop) {
#ifdef _WIN32
    // SO_RCVTIMEO does not apply to accept() on Windows: poll with select()
    fd_set fs;
    FD_ZERO(&fs);
    FD_SET(ls, &fs);
    timeval tv{0, 500000};
    if (select(0, &fs, nullptr, nullptr, &tv) <= 0) continue;
#endif
    sock_t s = ::accept(ls, nullptr, nullptr);
    if (s == SGCH_BADSOCK) continue;
    net::set_bufsize(s, 4 << 20);
    net::set_timeout(s, 1000);
    Session ss{rx, s};
    ss.verbose = verbose;
    bool rtl_fixed = g_rtl_gain_db < 1e8;
    double rtl_g = rtl_fixed ? std::pow(10.0, g_rtl_gain_db / 20) : 1.0, rtl_acc = 0;
    uint64_t rtl_n = 0;
    if (g_rtl) {
      // rtl_tcp greeting: "RTL0", tuner type (5 = R820T), number of gain steps; then u8 IQ
      uint8_t h[12];
      memcpy(h, "RTL0", 4);
      put32(h + 4, 5);
      put32(h + 8, 29);
      net::send_all(s, h, sizeof(h));
      ss.rtl = true;
      ss.bits = 8;
      fprintf(stderr, "sgch_rx: rtl_tcp client connected (%.6f MHz, %.3f MS/s, 8-bit IQ)\n",
              rx.get_frequency() / 1e6, rx.get_sample_rate() / 1e6);
    } else {
      send_header(rx, s);
    }
    if (!rx.running()) rx.start();
    std::thread cmd(command_loop, &ss);
    auto t0 = std::chrono::steady_clock::now();
    uint64_t sent = 0, words0 = rx.stats().words;
    while (ss.alive && !g_stop) {
      size_t k = rx.read(b.data(), b.size(), 200000);
      if (!k) continue;
      int bits = ss.bits;
      size_t nb = 0;
      const float *x = reinterpret_cast<const float *>(b.data());
      if (ss.rtl && !rtl_fixed) {
        // automatic level for 8 bits: measure the rms of the first ~0.5 s, then keep the gain
        // fixed (a moving gain would make the waterfall pump). Target rms 16 LSB (-18 dBFS).
        for (size_t i = 0; i < 2 * k; i++) rtl_acc += (double)x[i] * x[i];
        rtl_n += 2 * k;
        if (rtl_n >= (uint64_t)(rx.get_sample_rate() > 0 ? rx.get_sample_rate() : 4e6)) {
          double rms = std::sqrt(rtl_acc / (double)rtl_n) * 127.5;
          rtl_g = rms > 1e-6 ? std::min(1000.0, std::max(1.0, 16.0 / rms)) : 1.0;
          rtl_fixed = true;
          fprintf(stderr, "sgch_rx: rtl_tcp level: rms %.2f LSB -> digital gain %+.1f dB (fixed; --rtl-gain-db to set)\n",
                  rms, 20 * std::log10(rtl_g));
        }
      }
      if (bits == 8) {
        const float off = ss.rtl ? 128.0f : 128.5f;   // rtl_tcp: u8, 127.5 = zero (+0.5 rounding)
        const float sc = 127.5f * (ss.rtl ? (float)rtl_g : 1.0f);
        for (size_t i = 0; i < 2 * k; i++) {
          float v = x[i] * sc + off;
          v = v < 0.0f ? 0.0f : (v > 255.0f ? 255.0f : v);
          o[nb++] = (uint8_t)(int)v;
        }
      } else if (bits == 16) {
        int16_t *q = reinterpret_cast<int16_t *>(o.data());
        for (size_t i = 0; i < 2 * k; i++) {
          float v = x[i] * 32767.0f;
          v = v < -32768.0f ? -32768.0f : (v > 32767.0f ? 32767.0f : v);
          q[i] = (int16_t)lrintf(v);
        }
        nb = 4 * k;
      } else if (bits == 24) {
        for (size_t i = 0; i < 2 * k; i++) {
          float v = x[i] * 8388607.0f;
          v = v < -8388608.0f ? -8388608.0f : (v > 8388607.0f ? 8388607.0f : v);
          int32_t w = (int32_t)lrintf(v);
          o[nb++] = (uint8_t)w; o[nb++] = (uint8_t)(w >> 8); o[nb++] = (uint8_t)(w >> 16);
        }
      } else {
        int32_t *q = reinterpret_cast<int32_t *>(o.data());
        for (size_t i = 0; i < 2 * k; i++) {
          double v = (double)x[i] * 2147483647.0;
          v = v < -2147483648.0 ? -2147483648.0 : (v > 2147483647.0 ? 2147483647.0 : v);
          q[i] = (int32_t)v;
        }
        nb = 8 * k;
      }
      if (!net::send_all(s, o.data(), nb)) break;
      sent += k;
      double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (dt >= 10.0) {
        auto st = rx.stats();
        fprintf(stderr,
                "sgch_rx: %.2f MS/s out | frames %llu lost %llu | %d windows, %d bins, osh %d | "
                "from Pluto %.2f MS/s (%s) | dropped %llu\n",
                sent / dt / 1e6, (unsigned long long)st.frames, (unsigned long long)st.lost_frames, st.runs,
                st.bins, st.osh, (st.words - words0) / dt / 1e6, st.transport, (unsigned long long)st.dropped_out);
        words0 = st.words;
        t0 = std::chrono::steady_clock::now();
        sent = 0;
      }
    }
    ss.alive = false;
    net::shutdown_sock(s);
    cmd.join();
    net::close_sock(s);
    if (!g_keep_rx) rx.stop();
    fprintf(stderr, "sgch_rx: %s disconnected\n", g_rtl ? "rtl_tcp client" : "SDRangel");
  }
  net::close_sock(ls);
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  sgch::Config c;
  std::string out = "-", fmt = "cf32", opts, bind = "127.0.0.1";
  double freq = 0, rate = 0, gain = -1000, rfbw = 0;
  long long nsamp = -1;
  int server_port = 0;
  sgch::ChannelConfig cc;
  bool channels = false;
  double watchdog_s = 10;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto v = [&]() { if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", a.c_str()); exit(2); } return std::string(argv[++i]); };
    if (a == "-u") c.uri = v();
    else if (a == "-c") c.ctl_host = v();
    else if (a == "-f") freq = atof(v().c_str());
    else if (a == "-s") rate = atof(v().c_str());
    else if (a == "-g") gain = atof(v().c_str());
    else if (a == "-B") rfbw = atof(v().c_str());
    else if (a == "-n") nsamp = atoll(v().c_str());
    else if (a == "-o") out = v();
    else if (a == "-F") fmt = v();
    else if (a == "--no-fill") c.noise_fill = false;
    else if (a == "--noise-db") c.noise_db = atof(v().c_str());
    else if (a == "-x") opts = v();
    else if (a == "-v") c.verbose = true;
    else if (a == "-b") c.buffer_words = (size_t)atoll(v().c_str());
    else if (a == "--iiod") c.transport = 2;
    else if (a == "--stream") c.transport = 1;
    else if (a == "--rtltcp") {
      g_rtl = true;
      server_port = 1234;
      if (i + 1 < argc && argv[i + 1][0] != '-') server_port = atoi(argv[++i]);
    }
    else if (a == "--server") {
      server_port = 1234;
      if (i + 1 < argc && argv[i + 1][0] != '-') server_port = atoi(argv[++i]);
    } else if (a == "--bind") { bind = v(); cc.bind = bind; }
    else if (a == "--channels") {
      channels = true;
      if (i + 1 < argc && argv[i + 1][0] != '-') cc.nslots = atoi(argv[++i]);
    }
    else if (a == "--base-port") cc.base_port = atoi(v().c_str());
    else if (a == "--slot-bins") cc.slot_bins = atoi(v().c_str());
    else if (a == "--linger") cc.linger_s = atof(v().c_str());
    else if (a == "--list") cc.list_file = v();
    else if (a == "--no-children") cc.children = false;
    else if (a == "--remote-control") g_remote_ctl = true;
    else if (a == "--rtl-gain-db") g_rtl_gain_db = atof(v().c_str());
    else if (a == "--watchdog") watchdog_s = atof(v().c_str());
    else {
      fprintf(stderr,
              "usage: sgch_rx [-u ip:192.168.2.1] [-c sgchd_host] [-f LO_Hz] [-s rate] [-g gain_dB] [-B rf_bw_Hz]\n"
              "               [--no-fill] [--noise-db dB] [-x \"sgchd options\"] [-b buffer_words] [--iiod|--stream] [-v]\n"
              "               ( --rtltcp [port] [--bind 0.0.0.0]  plain rtl_tcp (u8 IQ) for OpenWebRX / SDRangel RTL0 )\n"
              "               [--rtl-gain-db dB (digital gain before 8 bits; default: automatic, set once)]\n"
              "               [--watchdog S (exit code 3 when the Pluto stream stops for S s; 0 = off)]\n"
              "               ( --server [port] [--bind 127.0.0.1|0.0.0.0] [--remote-control]\n"
              "                                                     SDRangel Remote TCP Input, wideband; the radio\n"
              "                                                     settings are locked unless --remote-control\n"
              "               | --channels [n] [--base-port 7301] [--slot-bins 16] [--linger 3] [--list file]\n"
              "                                                     one narrowband stream per detected downlink\n"
              "               | [-n samples] [-o file|-] [-F cf32|cs16] )      raw IQ to a file / stdout\n");
      return 2;
    }
  }
  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);
  setvbuf(stderr, nullptr, _IONBF, 0);
  if (channels) {
    c.wideband = server_port != 0;          // the wideband rebuild only when it is served
    c.on_frame = [](uint32_t seq, const std::complex<float> *X, const uint8_t *have, int N, float sc) {
      sgch::ChannelMux *m = g_mux;
      if (m) m->frame(seq, X, have, N, sc);
    };
  }
  try {
    sgch::Receiver rx(c);
    if (watchdog_s > 0) {
      // the Pluto was switched off / rebooted / unplugged: exit, so that a wrapper script
      // (satcom4pluto.bat) starts again and sets the radio up from scratch
      std::thread([&rx, watchdog_s] {
        uint64_t last = 0;
        auto t_ok = std::chrono::steady_clock::now();
        while (!g_stop) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          auto now = std::chrono::steady_clock::now();
          uint64_t w = rx.stats().words;
          if (!rx.running() || w != last) { last = w; t_ok = now; continue; }
          if (std::chrono::duration<double>(now - t_ok).count() > watchdog_s) {
            fprintf(stderr, "sgch_rx: no data from the Pluto for %.0f s -- exiting (code 3)\n", watchdog_s);
            fflush(stderr);
            std::_Exit(3);
          }
        }
      }).detach();
    }
    if (rate > 0) rx.set_sample_rate(rate);
    if (freq > 0) rx.set_frequency(freq);
    if (gain > -999) { rx.set_agc(false); rx.set_gain(gain); }
    if (rfbw > 0) rx.set_bandwidth(rfbw);
    if (!opts.empty()) fprintf(stderr, "sgchd: %s", rx.ctl_command("SET " + opts).c_str());
    if (channels) {
      int M = cc.slot_bins;
      if (M < 4 || M > 512 || (M & (M - 1))) { fprintf(stderr, "sgch_rx: --slot-bins must be a power of two (4..512)\n"); return 2; }
      cc.verbose = c.verbose;
      sgch::ChannelMux mux(cc);
      // the receiver object exists already: hook the frame callback through a pointer
      g_mux = &mux;
      mux.start();
      g_keep_rx = true;
      rx.start();
      std::thread poll(track_poller, &rx, &mux, 0.25);
      fprintf(stderr, "sgch_rx: %d channel slots on ports %d..%d (SDRangel: Remote TCP Input, protocol SDRangel)\n",
              cc.nslots, cc.base_port, cc.base_port + cc.nslots - 1);
      int rc = 0;
      if (server_port) rc = run_server(rx, bind, server_port, c.verbose);
      else while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(200));
      g_stop = true;
      poll.join();
      rx.stop();
      mux.stop();
      g_mux = nullptr;
      return rc;
    }
    if (server_port) return run_server(rx, bind, server_port, c.verbose);

    FILE *fo = out == "-" ? stdout : fopen(out.c_str(), "wb");
    if (!fo) { perror(out.c_str()); return 1; }
#ifdef _WIN32
    if (fo == stdout) _setmode(_fileno(stdout), _O_BINARY);
#endif
    rx.start();
    std::vector<std::complex<float>> b(65536);
    std::vector<int16_t> s16(2 * b.size());
    long long done = 0;
    double next_rep = 0;
    while (!g_stop && (nsamp < 0 || done < nsamp)) {
      size_t want = b.size();
      if (nsamp >= 0 && (long long)want > nsamp - done) want = nsamp - done;
      size_t k = rx.read(b.data(), want, 500000);
      if (!k) continue;
      if (fmt == "cs16") {
        for (size_t i = 0; i < k; i++) {
          float re = b[i].real() * 32767.f, im = b[i].imag() * 32767.f;
          s16[2 * i] = (int16_t)std::max(-32768.f, std::min(32767.f, re));
          s16[2 * i + 1] = (int16_t)std::max(-32768.f, std::min(32767.f, im));
        }
        fwrite(s16.data(), 4, k, fo);
      } else fwrite(b.data(), 8, k, fo);
      done += k;
      if (c.verbose && done >= next_rep) {
        auto st = rx.stats();
        fprintf(stderr, "samples %lld frames %llu lost %llu resync %llu runs %d bins %d osh %d words %llu spec %.1fs %s\n",
                done, (unsigned long long)st.frames, (unsigned long long)st.lost_frames,
                (unsigned long long)st.resyncs, st.runs, st.bins, st.osh, (unsigned long long)st.words,
                st.spec_age_s, st.transport);
        next_rep = done + 30e6;
      }
    }
    rx.stop();
    if (fo != stdout) fclose(fo);
  } catch (std::exception &e) {
    fprintf(stderr, "sgch_rx: %s\n", e.what());
    return 1;
  }
  return 0;
}
