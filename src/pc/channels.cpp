// SPDX-License-Identifier: MIT
#include "channels.hpp"

#define POCKETFFT_NO_MULTITHREADING
#include "pocketfft_hdronly.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sgch {

typedef pocketfft::detail::pocketfft_c<float> fft_plan;

static double now_s() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}
static void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = (uint8_t)v; }
static void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)(v >> 32)); put32(p + 4, (uint32_t)v); }

// zlib stream with one stored (uncompressed) deflate block: SDRangel's
// RemoteTCPInput inflates "dataIQzlib" messages; stored blocks cost no CPU.
static size_t zlib_stored(const uint8_t *d, size_t n, uint8_t *o) {
  size_t k = 0;
  o[k++] = 0x78; o[k++] = 0x01;
  o[k++] = 0x01;                                  // BFINAL = 1, BTYPE = 00 (stored)
  o[k++] = (uint8_t)n; o[k++] = (uint8_t)(n >> 8);
  o[k++] = (uint8_t)~n; o[k++] = (uint8_t)(~n >> 8);
  memcpy(o + k, d, n); k += n;
  uint32_t a = 1, b = 0;
  for (size_t i = 0; i < n; i++) { a = (a + d[i]) % 65521; b = (b + a) % 65521; }
  uint32_t ad = (b << 16) | a;
  o[k++] = ad >> 24; o[k++] = ad >> 16; o[k++] = ad >> 8; o[k++] = (uint8_t)ad;
  return k;
}

// ---------------------------------------------------------------------------
class Slot {
 public:
  Slot(int k, const ChannelConfig &c) : k_(k), c_(c) {}
  ~Slot() { stop(); delete (fft_plan *)plan_; if (dump_) fclose(dump_); }

  int k_;
  const ChannelConfig &c_;
  // assignment (protected by the mux mutex)
  int track = 0;                  // 0: idle
  double released = 0;            // time the track expired / vanished (0: alive)
  int c_nat = 0, ext_s = 0, ext_e = -1;   // centred extraction bins
  double fc_hz = 0, bw_hz = 0, snr_db = 0;
  int N = 0, M = 16;
  double fs_slot = 0;

  // synthesis state (only the rx thread)
  std::vector<std::complex<float>> ola_, tb_;
  std::vector<float> ws_;
  void *plan_ = nullptr;
  double rot_ph = 0, rot_inc = 0;
  bool have_last = false;
  uint32_t last_seq = 0;
  FILE *dump_ = nullptr;

  // output ring + server
  std::mutex m_;
  std::condition_variable cv_;
  std::vector<std::complex<float>> ring_ = std::vector<std::complex<float>>(1 << 18);
  size_t head_ = 0, fill_ = 0;
  uint64_t produced_ = 0, dropped_ = 0;
  std::atomic<bool> run_{false};
  std::atomic<uint64_t> freq_req_{0};    // centre frequency to announce (0: none pending)
  uint64_t freq_now_ = 0;
  std::thread thr_;

  void setup(int n, int m, double fs) {
    if (n == N && m == M && plan_) return;
    N = n; M = m;
    fs_slot = fs * M / N;
    ola_.assign(M, 0); tb_.assign(M, 0); ws_.resize(M);
    for (int i = 0; i < M; i++) ws_[i] = (float)std::sin(M_PI * i / M);
    delete (fft_plan *)plan_;
    plan_ = new fft_plan((size_t)M);
    have_last = false;
  }

  void push(const std::complex<float> *s, size_t n) {
    {
      std::lock_guard<std::mutex> g(m_);
      size_t cap = ring_.size();
      for (size_t i = 0; i < n; i++) {
        ring_[head_] = s[i];
        head_ = (head_ + 1) % cap;
      }
      fill_ += n;
      if (fill_ > cap) { dropped_ += fill_ - cap; fill_ = cap; }
      produced_ += n;
    }
    cv_.notify_all();
    if (dump_) fwrite(s, sizeof(std::complex<float>), n, dump_);
  }

  // one frame; X natural order; ext/c given in centred bins (mux mutex held)
  void synth(uint32_t seq, const std::complex<float> *X, const uint8_t *have, float scale) {
    if (have_last) {
      uint32_t gap = seq - last_seq;
      if (gap == 0 || gap > 20000) { std::fill(ola_.begin(), ola_.end(), std::complex<float>(0, 0)); }
    }
    have_last = true; last_seq = seq;
    std::fill(tb_.begin(), tb_.end(), std::complex<float>(0, 0));
    if (track && X) {
      for (int d = -M / 2; d < M / 2; d++) {
        int cj = c_nat + N / 2 + d;                 // centred index
        if (cj < ext_s || cj > ext_e) continue;
        int b = ((c_nat + d) % N + N) % N;
        if (have[b]) tb_[(d + M) % M] = X[b];
      }
    }
    ((fft_plan *)plan_)->exec((pocketfft::detail::cmplx<float> *)tb_.data(), 1.0f, false);
    float sg = ((c_nat & 1) && (seq & 1)) ? -scale : scale;   // e^{-j pi c k}
    for (int m = 0; m < M; m++) ola_[m] += tb_[m] * (ws_[m] * sg);
    std::complex<float> out[512];
    int H = M / 2;
    for (int m = 0; m < H; m++) {
      std::complex<float> r((float)std::cos(rot_ph), (float)-std::sin(rot_ph));
      out[m] = ola_[m] * r;
      rot_ph += rot_inc;
      if (rot_ph > M_PI) rot_ph -= 2 * M_PI;
      if (rot_ph < -M_PI) rot_ph += 2 * M_PI;
    }
    std::copy(ola_.begin() + H, ola_.end(), ola_.begin());
    std::fill(ola_.begin() + H, ola_.end(), std::complex<float>(0, 0));
    push(out, H);
  }

  void start() {
    if (run_) return;
    run_ = true;
    thr_ = std::thread(&Slot::server, this);
  }
  void stop() {
    if (!run_) return;
    run_ = false;
    cv_.notify_all();
    if (thr_.joinable()) thr_.join();
  }

  void header(sock_t s, uint64_t f) {
    uint8_t h[128];
    memset(h, 0, sizeof(h));
    memcpy(h, "SDRA", 4);
    put32(h + 4, 0);                               // device UNKNOWN: any sample rate in the GUI
    put64(h + 8, f);
    put32(h + 20, 1u << 7);                        // framed (compressed) IQ, no remote control
    uint32_t r = (uint32_t)llround(fs_slot);
    put32(h + 24, r);
    put32(h + 52, r);                              // channel rate = device rate
    put32(h + 56, 16);
    put32(h + 60, 1);
    float sq = -100.0f, gate = 0.001f;
    uint32_t u;
    memcpy(&u, &sq, 4); put32(h + 64, u);
    memcpy(&u, &gate, 4); put32(h + 68, u);
    net::send_all(s, h, sizeof(h));
  }

  void server() {
    sock_t ls = net::listen_tcp(c_.bind, c_.base_port + k_);
    if (ls == SGCH_BADSOCK) {
      fprintf(stderr, "sgch_rx: slot %d: cannot listen on port %d (in use, or reserved by Windows: "
                      "netsh interface ipv4 show excludedportrange protocol=tcp) -- try --base-port\n",
              k_, c_.base_port + k_);
      return;
    }
    std::vector<std::complex<float>> b(2048);
    std::vector<int16_t> q(2 * b.size());
    std::vector<uint8_t> z(q.size() * 2 + 64);
    while (run_) {
      fd_set fs; FD_ZERO(&fs); FD_SET(ls, &fs);
      timeval tv{0, 300000};
      if (select((int)ls + 1, &fs, nullptr, nullptr, &tv) <= 0) continue;
      sock_t s = ::accept(ls, nullptr, nullptr);
      if (s == SGCH_BADSOCK) continue;
      net::set_bufsize(s, 1 << 20);
      net::set_timeout(s, 1000);
      uint64_t f = freq_req_.load();
      if (!f) f = freq_now_;
      freq_now_ = f;
      freq_req_ = 0;
      header(s, f);
      {
        std::lock_guard<std::mutex> g(m_);
        fill_ = std::min(fill_, (size_t)(fs_slot * 0.05));   // start with little latency
      }
      fprintf(stderr, "sgch_rx: slot %d: SDRangel connected (port %d, %.0f S/s, %.6f MHz)\n", k_, c_.base_port + k_,
              fs_slot, f / 1e6);
      bool ok = true;
      while (run_ && ok) {
        // drain anything SDRangel sends (settings commands are ignored for slots)
        for (;;) {
          fd_set r; FD_ZERO(&r); FD_SET(s, &r);
          timeval t0{0, 0};
          if (select((int)s + 1, &r, nullptr, nullptr, &t0) <= 0) break;
          char junk[256];
          long k = net::recv_some(s, junk, sizeof(junk));
          if (k <= 0) { ok = false; break; }
        }
        if (!ok) break;
        uint64_t fr = freq_req_.exchange(0);
        if (fr && fr != freq_now_) {
          uint8_t cmd[5];
          cmd[0] = 0x01;                            // setCenterFrequency
          put32(cmd + 1, (uint32_t)fr);
          if (!net::send_all(s, cmd, 5)) break;
          freq_now_ = fr;
        }
        size_t n = 0;
        {
          std::unique_lock<std::mutex> g(m_);
          cv_.wait_for(g, std::chrono::milliseconds(100), [&] { return fill_ >= b.size() || !run_; });
          n = std::min(fill_, b.size());
          size_t cap = ring_.size(), tail = (head_ + cap - fill_) % cap;
          for (size_t i = 0; i < n; i++) b[i] = ring_[(tail + i) % cap];
          fill_ -= n;
        }
        if (!n) continue;
        for (size_t i = 0; i < n; i++) {
          float re = b[i].real() * 32767.f, im = b[i].imag() * 32767.f;
          q[2 * i] = (int16_t)std::max(-32768.f, std::min(32767.f, re));
          q[2 * i + 1] = (int16_t)std::max(-32768.f, std::min(32767.f, im));
        }
        size_t zl = zlib_stored((const uint8_t *)q.data(), n * 4, z.data() + 5);
        z[0] = 0xf2;                                // dataIQzlib
        put32(z.data() + 1, (uint32_t)zl);
        if (!net::send_all(s, z.data(), zl + 5)) break;
      }
      net::close_sock(s);
      fprintf(stderr, "sgch_rx: slot %d: SDRangel disconnected\n", k_);
    }
    net::close_sock(ls);
  }
};

// ---------------------------------------------------------------------------
ChannelMux::ChannelMux(const ChannelConfig &c) : c_(c) {
  for (int k = 0; k < c_.nslots; k++) slots_.push_back(new Slot(k, c_));
}
ChannelMux::~ChannelMux() {
  stop();
  for (auto *s : slots_) delete s;
}
void ChannelMux::start() { for (auto *s : slots_) s->start(); }
void ChannelMux::stop() { for (auto *s : slots_) s->stop(); }

void ChannelMux::set_radio(double lo, double fs, int N) {
  std::lock_guard<std::mutex> g(mtx_);
  if (lo == lo_ && fs == fs_ && N == N_) return;
  lo_ = lo; fs_ = fs; N_ = N;
  for (auto *s : slots_) {
    s->track = 0;
    s->setup(N, c_.slot_bins, fs);
    s->freq_req_ = (uint64_t)llround(lo);
    if (!dump_prefix_.empty() && !s->dump_) {
      std::string fn = dump_prefix_ + std::to_string(s->k_) + ".cf32";
      s->dump_ = fopen(fn.c_str(), "wb");
    }
  }
  write_list_locked();
}

void ChannelMux::update_tracks(const std::vector<TrackInfo> &t) {
  std::lock_guard<std::mutex> g(mtx_);
  tracks_ = t;
  assign_locked();
}

void ChannelMux::assign_locked() {
  if (!N_ || fs_ <= 0) return;
  double binw = fs_ / N_, now = now_s();
  int M = c_.slot_bins;
  // existing assignments
  for (auto *s : slots_) {
    if (!s->track) continue;
    const TrackInfo *ti = nullptr;
    for (auto &x : tracks_) if (x.id == s->track) ti = &x;
    bool alive = ti && ti->streaming;
    if (alive) {
      s->released = 0;
      s->snr_db = ti->snr_db; s->bw_hz = ti->bw_hz;
      int es = (int)lrint((ti->ext_lo_hz - lo_) / binw + N_ / 2 + 0.5);
      int ee = (int)lrint((ti->ext_hi_hz - lo_) / binw + N_ / 2 - 0.5);
      int c = s->c_nat + N_ / 2;
      s->ext_s = std::max(es, c - M / 2 + 1);
      s->ext_e = std::min(ee, c + M / 2 - 2);
      double off = (ti->fc_hz - lo_) / binw + N_ / 2 - c;
      if (std::fabs(off) > M / 2 - 3) s->track = 0;      // drifted out of the slot: reassign
    } else {
      if (s->released == 0) s->released = now;
      if (now - s->released > c_.linger_s) { s->track = 0; s->released = 0; }
    }
  }
  // new downlinks, strongest first
  std::vector<const TrackInfo *> cand;
  for (auto &x : tracks_) {
    if (!x.streaming) continue;
    if (x.parent >= 0 && (!c_.children || !x.independent)) continue;
    bool has = false;
    for (auto *s : slots_) if (s->track == x.id) has = true;
    if (!has) cand.push_back(&x);
  }
  // primary downlinks first (strongest first), then independent narrow-in-wide ones
  std::sort(cand.begin(), cand.end(), [](const TrackInfo *a, const TrackInfo *b) {
    if ((a->parent >= 0) != (b->parent >= 0)) return a->parent < 0;
    return a->snr_db > b->snr_db;
  });
  for (auto *x : cand) {
    // already inside the band of an assigned slot: SDRangel can tune it there
    int cx = (int)lrint((x->fc_hz - lo_) / binw + N_ / 2);
    bool covered = false;
    for (auto *s : slots_) if (s->track && cx >= s->ext_s && cx <= s->ext_e) covered = true;
    if (covered) continue;
    Slot *fr = nullptr;
    for (auto *s : slots_) if (!s->track) { fr = s; break; }
    if (!fr) break;
    int c = (int)lrint((x->fc_hz - lo_) / binw + N_ / 2);
    fr->track = x->id;
    fr->released = 0;
    fr->c_nat = c - N_ / 2;
    int es = (int)lrint((x->ext_lo_hz - lo_) / binw + N_ / 2 + 0.5);
    int ee = (int)lrint((x->ext_hi_hz - lo_) / binw + N_ / 2 - 0.5);
    fr->ext_s = std::max(es, c - M / 2 + 1);
    fr->ext_e = std::min(ee, c + M / 2 - 2);
    fr->fc_hz = x->fc_hz; fr->bw_hz = x->bw_hz; fr->snr_db = x->snr_db;
    // the downlink sits (fc - bin centre) from the slot centre: rotate it to 0 Hz
    double shift = x->fc_hz - (lo_ + fr->c_nat * binw);
    fr->rot_inc = 2 * M_PI * shift / fr->fs_slot;
    fr->freq_req_ = (uint64_t)llround(x->fc_hz);
  }
  write_list_locked();
}

void ChannelMux::frame(uint32_t seq, const std::complex<float> *X, const uint8_t *have, int N, float scale) {
  std::lock_guard<std::mutex> g(mtx_);
  if (N != N_) return;
  for (auto *s : slots_) {
    if (s->N != N) s->setup(N, c_.slot_bins, fs_);
    s->synth(seq, X, have, scale);
  }
}

std::string ChannelMux::table() {
  std::lock_guard<std::mutex> g(mtx_);
  return last_table_;
}

void ChannelMux::write_list_locked() {
  std::ostringstream o;
  char b[256];
  snprintf(b, sizeof(b), "# slot port rate_Sps centre_MHz track bw_kHz snr_dB state (LO %.6f MHz)\n", lo_ / 1e6);
  o << b;
  for (auto *s : slots_) {
    if (s->track)
      snprintf(b, sizeof(b), "%d %d %.0f %.6f %d %.1f %.1f %s\n", s->k_, c_.base_port + s->k_, s->fs_slot,
               s->fc_hz / 1e6, s->track, s->bw_hz / 1e3, s->snr_db, s->released ? "fading" : "on");
    else
      snprintf(b, sizeof(b), "%d %d %.0f - - - - idle\n", s->k_, c_.base_port + s->k_, s->fs_slot);
    o << b;
  }
  std::string t = o.str();
  if (t != last_table_) {
    last_table_ = t;
    if (!c_.list_file.empty()) {
      std::string tmp = c_.list_file + ".tmp";
      FILE *f = fopen(tmp.c_str(), "w");
      if (f) { fputs(t.c_str(), f); fclose(f); remove(c_.list_file.c_str()); rename(tmp.c_str(), c_.list_file.c_str()); }
    }
    if (c_.verbose || true) {
      fprintf(stderr, "sgch_rx: channels:\n%s", t.c_str());
    }
  }
}

bool parse_tracks(const std::string &r, std::vector<TrackInfo> &out, double *lo, double *fs, double *binw) {
  std::istringstream in(r);
  std::string line;
  out.clear();
  if (!std::getline(in, line) || line.compare(0, 3, "OK ") != 0) return false;
  int n = 0;
  double l = 0, f = 0, bw = 0, t = 0;
  if (sscanf(line.c_str(), "OK %d fs=%lf lo=%lf bin_hz=%lf t=%lf", &n, &f, &l, &bw, &t) < 4) return false;
  if (lo) *lo = l;
  if (fs) *fs = f;
  if (binw) *binw = bw;
  while (std::getline(in, line)) {
    TrackInfo x;
    char st[32];
    double fc, bwk, elo, ehi;
    int str = 1;
    int k = sscanf(line.c_str(), "%d %31s %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %d", &x.id, st, &fc, &bwk, &x.snr_db,
                   &x.z, &x.conf, &x.age_s, &x.last_s, &x.parent, &x.independent, &elo, &ehi, &str);
    if (k < 13) continue;
    x.state = st;
    x.fc_hz = fc * 1e6; x.bw_hz = bwk * 1e3; x.ext_lo_hz = elo * 1e6; x.ext_hi_hz = ehi * 1e6;
    x.streaming = k >= 14 ? str : (x.state == "CONFIRMED" || x.state == "ACTIVE" || x.state == "HOLD");
    out.push_back(x);
  }
  return true;
}

}  // namespace sgch
