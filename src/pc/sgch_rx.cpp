// SPDX-License-Identifier: MIT
#include "sgch_rx.hpp"

#include "iiod.hpp"
#include "net.hpp"

#define POCKETFFT_NO_MULTITHREADING
#include "pocketfft_hdronly.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sgch {

typedef pocketfft::detail::pocketfft_c<float> fft_plan;
static const uint32_t MAGIC = 0x53474348u;   // "SGCH"
static const char *RXDEV = "cf-ad9361-lpc";
static const char *PHY = "ad9361-phy";

static double now_s() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

static void uri_split(const std::string &uri, std::string &host, int &port) {
  // ip:HOST[:port]  (usb: is not supported -- use the Pluto's RNDIS address)
  port = 30431;
  host = "192.168.2.1";
  if (uri.rfind("ip:", 0) != 0) return;
  std::string h = uri.substr(3);
  if (!h.empty() && h[0] == '[') {
    size_t e = h.find(']');
    host = h.substr(1, e - 1);
    if (e + 1 < h.size() && h[e + 1] == ':') port = atoi(h.c_str() + e + 2);
    return;
  }
  size_t c = h.find(':');
  if (c == std::string::npos) host = h;
  else { host = h.substr(0, c); port = atoi(h.c_str() + c + 1); }
}

Receiver::Receiver(const Config &cfg) : cfg_(cfg) {
  uri_split(cfg_.uri, host_, iiod_port_);
  if (cfg_.ctl_host.empty()) cfg_.ctl_host = host_;
  ring_.resize(cfg_.ring_samples);
  // gaussian table (Box-Muller once); the noise fill only indexes it
  gtab_.resize(1 << 16);
  for (size_t i = 0; i < gtab_.size(); i += 2) {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    double a = ((rng_ >> 8) + 0.5) / 16777216.0;
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    double b = ((rng_ >> 8) + 0.5) / 16777216.0;
    double r = std::sqrt(-2.0 * std::log(a));
    gtab_[i] = (float)(r * std::cos(2 * M_PI * b));
    gtab_[i + 1] = (float)(r * std::sin(2 * M_PI * b));
  }
  if (cfg_.uri == "offline") return;
  attr_ = new iiod::Client;
  strm_ = new iiod::Client;
  if (!attr_->open(host_, iiod_port_))
    throw std::runtime_error("cannot connect to iiod at " + host_ + ":" + std::to_string(iiod_port_) +
                             " (is the Pluto reachable?)");
  if (phy_attr("INPUT", "voltage0", "sampling_frequency").empty())
    throw std::runtime_error("ad9361-phy not found on " + host_ + " (is this a Pluto?)");
}

Receiver::~Receiver() {
  stop();
  delete (fft_plan *)plan_;
  delete attr_;
  delete strm_;
}

// ---------------------------------------------------------------------------
// radio settings (ad9361-phy attributes over iiod)
std::string Receiver::phy_attr(const char *dir, const char *chan, const char *attr) {
  if (!attr_) return "";
  return attr_->read_attr(PHY, dir, chan, attr);
}
void Receiver::phy_write(const char *dir, const char *chan, const char *attr, const std::string &v) {
  if (!attr_) return;
  long r = attr_->write_attr(PHY, dir, chan, attr, v);
  if (r < 0 && cfg_.verbose) fprintf(stderr, "sgch_rx: write %s %s = %s failed (%ld)\n", chan, attr, v.c_str(), r);
}
static std::string i64s(double v) { return std::to_string((long long)llround(v)); }
void Receiver::set_frequency(double hz) { phy_write("OUTPUT", "altvoltage0", "frequency", i64s(hz)); }
double Receiver::get_frequency() { return atof(phy_attr("OUTPUT", "altvoltage0", "frequency").c_str()); }
void Receiver::set_sample_rate(double sps) { phy_write("INPUT", "voltage0", "sampling_frequency", i64s(sps)); }
double Receiver::get_sample_rate() { return atof(phy_attr("INPUT", "voltage0", "sampling_frequency").c_str()); }
void Receiver::set_bandwidth(double hz) { phy_write("INPUT", "voltage0", "rf_bandwidth", i64s(hz)); }
double Receiver::get_bandwidth() { return atof(phy_attr("INPUT", "voltage0", "rf_bandwidth").c_str()); }
void Receiver::set_gain(double db) {
  char b[32];
  snprintf(b, sizeof(b), "%.0f", db);
  phy_write("INPUT", "voltage0", "hardwaregain", b);
}
double Receiver::get_gain() { return atof(phy_attr("INPUT", "voltage0", "hardwaregain").c_str()); }
void Receiver::set_agc(bool on) { phy_write("INPUT", "voltage0", "gain_control_mode", on ? "slow_attack" : "manual"); }
bool Receiver::get_agc() { return phy_attr("INPUT", "voltage0", "gain_control_mode").compare(0, 6, "manual") != 0; }

// ---------------------------------------------------------------------------
// control connection to sgchd
std::string Receiver::ctl_command(const std::string &cmd) {
  sock_t fd = net::connect_tcp(cfg_.ctl_host, cfg_.ctl_port, 2000);
  if (fd == SGCH_BADSOCK) return "ERR cannot connect to sgchd at " + cfg_.ctl_host + ":" + std::to_string(cfg_.ctl_port);
  std::string c = cmd + "\nQUIT\n";
  net::send_all(fd, c.data(), c.size());
  std::string r;
  char b[4096];
  long k;
  while ((k = net::recv_some(fd, b, sizeof(b))) > 0) r.append(b, (size_t)k);
  net::close_sock(fd);
  size_t p = r.rfind("OK bye");
  if (p != std::string::npos) r.erase(p);
  return r;
}

void Receiver::ctl_thread() {
  while (run_) {
    sock_t fd = net::connect_tcp(cfg_.ctl_host, cfg_.ctl_port, 3000);
    if (fd == SGCH_BADSOCK) {
      for (int i = 0; i < 20 && run_; i++) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    const char *sub = "MODE 1\nSUBSCRIBE\n";
    net::send_all(fd, sub, strlen(sub));
    std::string acc;
    std::vector<float> sp;
    std::vector<char> b(65536);
    while (run_) {
      long k = net::recv_some(fd, b.data(), b.size());
      if (k == 0) break;
      if (k < 0) { if (net::timed_out()) continue; break; }
      acc.append(b.data(), (size_t)k);
      for (;;) {
        size_t nl = acc.find('\n');
        if (nl == std::string::npos) break;
        if (acc.compare(0, 5, "SPEC ") != 0) { acc.erase(0, nl + 1); continue; }
        int n = 0, osh = 0; double fs = 0, lo = 0; unsigned upd = 0;
        if (sscanf(acc.c_str(), "SPEC %d %lf %lf %d %u", &n, &fs, &lo, &osh, &upd) < 4 || n <= 0 || n > 65536) {
          acc.erase(0, nl + 1); continue;
        }
        size_t need = nl + 1 + (size_t)n * 4;
        if (acc.size() < need) break;
        sp.resize(n);
        memcpy(sp.data(), acc.data() + nl + 1, (size_t)n * 4);
        acc.erase(0, need);
        // per-bin |X|^2 in output units (after osh) -> amplitude per real/imag part, absolute units
        std::lock_guard<std::mutex> g(spec_mtx_);
        fill_amp_.resize(n);
        double k2 = std::pow(4.0, osh) * std::pow(10.0, cfg_.noise_db / 10.0);
        for (int i = 0; i < n; i++) fill_amp_[i] = (float)std::sqrt(std::max(0.0, sp[i] * k2 / 2.0));
        spec_time_ = now_s();
      }
    }
    net::close_sock(fd);
  }
}

// ---------------------------------------------------------------------------
float Receiver::gauss() {
  rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
  return gtab_[rng_ >> 16];
}

void Receiver::push_out(const std::complex<float> *s, size_t n) {
  std::unique_lock<std::mutex> g(out_mtx_);
  size_t cap = ring_.size();
  if (r_fill_ + n > cap) {   // consumer too slow: drop the oldest
    size_t d = r_fill_ + n - cap;
    r_tail_ = (r_tail_ + d) % cap; r_fill_ -= d;
    st_.dropped_out += d;
  }
  size_t i = 0;
  while (i < n) {
    size_t k = std::min(n - i, cap - r_head_);
    std::copy(s + i, s + i + k, ring_.begin() + r_head_);
    r_head_ = (r_head_ + k) % cap; i += k;
  }
  r_fill_ += n;
  st_.samples_out += n;
  g.unlock();
  out_cv_.notify_all();
}

size_t Receiver::read(std::complex<float> *out, size_t n, long timeout_us) {
  std::unique_lock<std::mutex> g(out_mtx_);
  if (!out_cv_.wait_for(g, std::chrono::microseconds(timeout_us), [&] { return r_fill_ >= n || !run_; }))
    if (r_fill_ == 0) return 0;
  size_t k = std::min(n, r_fill_);
  size_t cap = ring_.size();
  size_t i = 0;
  while (i < k) {
    size_t m = std::min(k - i, cap - r_tail_);
    std::copy(ring_.begin() + r_tail_, ring_.begin() + r_tail_ + m, out + i);
    r_tail_ = (r_tail_ + m) % cap; i += m;
  }
  r_fill_ -= k;
  return k;
}

Stats Receiver::stats() {
  std::lock_guard<std::mutex> g(st_mtx_);
  Stats s = st_;
  {
    std::lock_guard<std::mutex> g2(spec_mtx_);
    s.spec_age_s = spec_time_ > 0 ? now_s() - spec_time_ : -1;
  }
  return s;
}

// ---------------------------------------------------------------------------
// synthesis: X_ (absolute FFT units) -> IFFT -> sqrt-Hann -> overlap-add
void Receiver::synth_frame(uint32_t) {
  std::vector<float> amp;
  bool fill = cfg_.noise_fill;
  if (fill) {
    std::lock_guard<std::mutex> g(spec_mtx_);
    if ((int)fill_amp_.size() == N_ && now_s() - spec_time_ < 5.0) amp = fill_amp_;
    else fill = false;
  }
  for (int b = 0; b < N_; b++) {
    if (have_[b]) tbuf_[b] = X_[b];
    else if (fill) tbuf_[b] = std::complex<float>(amp[b] * gauss(), amp[b] * gauss());
    else tbuf_[b] = 0;
  }
  ((fft_plan *)plan_)->exec((pocketfft::detail::cmplx<float> *)tbuf_.data(), 1.0f, false);   // backward, unnormalised
  for (int n = 0; n < N_; n++) ola_[n] += tbuf_[n] * (win_[n] * out_scale_);
  push_out(ola_.data(), H_);
  std::copy(ola_.begin() + H_, ola_.end(), ola_.begin());
  std::fill(ola_.begin() + H_, ola_.end(), std::complex<float>(0, 0));
}

void Receiver::emit_noise_frames(uint32_t from, uint32_t to) {
  // frames from..to-1 never arrived: synthesise them from the noise fill only
  for (uint32_t s = from; s != to; s++) {
    std::fill(have_.begin(), have_.end(), 0);
    synth_frame(s);
  }
}

void Receiver::frame_done() {
  {
    std::lock_guard<std::mutex> g(st_mtx_);
    st_.frames++;
    st_.runs = f_nruns_;
    st_.osh = f_osh_;
    int nb = 0;
    for (int b = 0; b < N_; b++) nb += have_[b];
    st_.bins = nb;
  }
  if (have_last_) {
    uint32_t gap = f_seq_ - last_seq_;
    if (gap == 0 || gap > 20000) {          // restart (core reset or long outage)
      std::fill(ola_.begin(), ola_.end(), std::complex<float>(0, 0));
    } else if (gap > 1) {
      std::lock_guard<std::mutex> g(st_mtx_);
      st_.lost_frames += gap - 1;
    }
    if (gap > 1 && gap <= 20000) {
      if (cfg_.on_frame)
        for (uint32_t q = last_seq_ + 1; q != f_seq_; q++) cfg_.on_frame(q, nullptr, have_.data(), N_, out_scale_);
      if (cfg_.wideband) {
        std::vector<std::complex<float>> keepX = X_;
        std::vector<uint8_t> keepH = have_;
        emit_noise_frames(last_seq_ + 1, f_seq_);
        X_ = keepX; have_ = keepH;
      }
    }
  }
  if (cfg_.on_frame) cfg_.on_frame(f_seq_, X_.data(), have_.data(), N_, out_scale_);
  if (cfg_.wideband) synth_frame(f_seq_);
  last_seq_ = f_seq_;
  have_last_ = true;
}

void Receiver::parse_words(const uint32_t *w, size_t n) {
  for (size_t i = 0; i < n; i++) {
    uint32_t v = w[i];
    switch (pst_) {
      case P_SYNC:
        if (v == MAGIC) pst_ = P_H1;
        else { std::lock_guard<std::mutex> g(st_mtx_); st_.resyncs++; }
        break;
      case P_H1:
        f_seq_ = v; pst_ = P_H2;
        break;
      case P_H2: {
        f_h2_ = v;
        f_osh_ = v >> 28;
        int l = (v >> 24) & 15;
        f_nruns_ = v & 0xFFFF;
        if (l < 8 || l > 15 || f_nruns_ > 4096) { pst_ = P_SYNC; break; }
        if (l != L_ || win_.empty()) {
          L_ = l; N_ = 1 << l; H_ = N_ / 2;
          X_.assign(N_, 0); have_.assign(N_, 0); ola_.assign(N_, 0); tbuf_.assign(N_, 0);
          win_.resize(N_);
          for (int k = 0; k < N_; k++) win_[k] = (float)std::sin(M_PI * k / N_);
          delete (fft_plan *)plan_;
          plan_ = new fft_plan((size_t)N_);
          // x_adc = ifft(X) * 2^(L-10); full scale (12 bit) = 2048; the backward FFT is unnormalised
          out_scale_ = (float)(std::pow(2.0, L_ - 10) / 2048.0 / N_);
          have_last_ = false;
        }
        std::fill(have_.begin(), have_.end(), 0);
        f_run_ = 0; r_start_ = 0; r_len_ = 0;
        if (f_nruns_ == 0) { frame_done(); pst_ = P_SYNC; }
        else pst_ = P_RUNHDR;
        break;
      }
      case P_RUNHDR: {
        int s = v & 0xFFFF, len = v >> 16;
        if (len == 0 || s + len > N_ || s < r_start_ + r_len_) { pst_ = (v == MAGIC) ? P_H1 : P_SYNC; break; }
        r_start_ = s; r_len_ = len; r_pos_ = 0;
        pst_ = P_DATA;
        break;
      }
      case P_DATA: {
        int b = r_start_ + r_pos_;
        int16_t re = (int16_t)(v & 0xFFFF), im = (int16_t)(v >> 16);
        float sc = (float)(1 << f_osh_);
        X_[b] = std::complex<float>(re * sc, im * sc);
        have_[b] = (v != 0);
        if (++r_pos_ == r_len_) {
          if (++f_run_ == f_nruns_) { frame_done(); pst_ = P_SYNC; }
          else pst_ = P_RUNHDR;
        }
        break;
      }
    }
  }
}

bool Receiver::rx_stream_session() {
  sock_t s = net::connect_tcp(cfg_.ctl_host, cfg_.ctl_port + 1, 3000);
  if (s == SGCH_BADSOCK) return false;
  net::set_bufsize(s, 8 << 20);
  net::set_timeout(s, 1000);
  stream_sock_ = s;
  std::vector<uint32_t> buf(1 << 16);
  bool got_any = false;
  size_t carry = 0;   // bytes of a partial word
  {
    std::lock_guard<std::mutex> g(st_mtx_);
    st_.transport = "sgchd stream";
  }
  pst_ = P_SYNC;
  while (run_) {
    long k = net::recv_some(s, (char *)buf.data() + carry, buf.size() * 4 - carry);
    if (k < 0 && net::timed_out()) continue;
    if (k <= 0) break;
    if (!got_any) {
      got_any = true;
      if (!memcmp(buf.data(), "ERR ", 4)) {   // RX buffer busy (iiod / other program)
        if (cfg_.verbose) fprintf(stderr, "sgch_rx: sgchd stream: %.*s", (int)k, (char *)buf.data());
        got_any = false;
        break;
      }
    }
    size_t tot = carry + (size_t)k, n = tot / 4;
    {
      std::lock_guard<std::mutex> g(st_mtx_);
      st_.words += n;
    }
    parse_words(buf.data(), n);
    carry = tot - n * 4;
    if (carry) memmove(buf.data(), (char *)buf.data() + n * 4, carry);
  }
  stream_sock_ = SGCH_BADSOCK;
  net::close_sock(s);
  return got_any;
}

void Receiver::rx_thread() {
  if (cfg_.transport != 2) {
    while (run_) {
      bool ok = rx_stream_session();
      if (!run_) return;
      if (!ok && cfg_.transport == 0) break;   // no stream port (older sgchd): iiod
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!run_) return;
    if (cfg_.verbose) fprintf(stderr, "sgch_rx: sgchd stream not available, using iiod\n");
  }
  {
    std::lock_guard<std::mutex> g(st_mtx_);
    st_.transport = "iiod";
  }
  std::vector<uint32_t> buf(cfg_.buffer_words);
  int fails = 0;
  FILE *raw = getenv("SGCH_RAWDUMP") ? fopen(getenv("SGCH_RAWDUMP"), "wb") : nullptr;   // debug
  while (run_) {
    if (!strm_->ok()) {
      if (!strm_->open(host_, iiod_port_) ) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      strm_->set_bufsize(8 << 20);
      strm_->set_timeout(1000);
      // RX1 I/Q (voltage0/voltage1): one 32-bit word per sample
      long r = strm_->open_buffer(RXDEV, cfg_.buffer_words, "00000003");
      if (r < 0) {
        if (cfg_.verbose) fprintf(stderr, "sgch_rx: OPEN %s failed (%ld)\n", RXDEV, r);
        strm_->close();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      pst_ = P_SYNC;
    }
    long k = strm_->read_buffer(RXDEV, buf.data(), buf.size() * 4, &keep_);
    if (!run_) break;
    if (k <= 0) {
      std::lock_guard<std::mutex> g(st_mtx_);
      st_.overflows++;
      if (++fails > 3) { strm_->close(); fails = 0; }
      continue;
    }
    fails = 0;
    size_t n = (size_t)k / 4;
    {
      std::lock_guard<std::mutex> g(st_mtx_);
      st_.words += n;
    }
    if (raw) { uint32_t m[2] = {0xDEADBEEFu, (uint32_t)n}; fwrite(m, 4, 2, raw); fwrite(buf.data(), 4, n, raw); }
    parse_words(buf.data(), n);
  }
  if (raw) fclose(raw);
  if (strm_->ok()) {
    strm_->close();   // closing the connection releases the buffer in iiod
  }
}

void Receiver::start() {
  if (run_) return;
  {
    std::lock_guard<std::mutex> g(out_mtx_);
    r_head_ = r_tail_ = r_fill_ = 0;
  }
  run_ = true;
  keep_ = true;
  pst_ = P_SYNC;
  have_last_ = false;
  ctl_thr_ = std::thread(&Receiver::ctl_thread, this);
  if (strm_) rx_thr_ = std::thread(&Receiver::rx_thread, this);
}

void Receiver::stop() {
  if (!run_) return;
  run_ = false;
  keep_ = false;
  if (strm_) strm_->abort();
  net::shutdown_sock(stream_sock_.load());
  out_cv_.notify_all();
  if (rx_thr_.joinable()) rx_thr_.join();
  if (ctl_thr_.joinable()) ctl_thr_.join();
}

}  // namespace sgch
