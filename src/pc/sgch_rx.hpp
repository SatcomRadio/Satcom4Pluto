// SPDX-License-Identifier: MIT
//
// sgch_rx -- receive the Pluto channelizer frames (iiod network protocol) and
// rebuild a continuous wideband IQ stream (overlap-add synthesis, optional
// noise fill). No libiio / FFTW needed (own iiod client, pocketfft).
#pragma once
#include <atomic>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "net.hpp"

namespace iiod { class Client; }

namespace sgch {

struct Config {
  std::string uri = "ip:192.168.2.1";   // ip:HOST[:port] of the Pluto (iiod)
  std::string ctl_host;                  // sgchd host (default: host of uri)
  int ctl_port = 30470;
  bool noise_fill = true;                // synthetic noise where no bins are sent
  double noise_db = 0.0;                 // noise-fill level offset (dB)
  size_t buffer_words = 65536;           // DMA buffer (32-bit words)
  size_t ring_samples = 1 << 23;         // output FIFO
  bool verbose = false;
  int transport = 0;                     // 0 auto (sgchd stream, else iiod), 1 sgchd stream, 2 iiod
  bool wideband = true;                  // rebuild the wideband stream (read())
  // called for every analysis frame (natural bin order, absolute FFT units; have[b]
  // = bin present). Lost frames are reported with X = nullptr. scale: FFT units ->
  // full scale for an unnormalised inverse FFT
  std::function<void(uint32_t seq, const std::complex<float> *X, const uint8_t *have, int N, float scale)> on_frame;
};

struct Stats {
  uint64_t frames = 0, lost_frames = 0, resyncs = 0, words = 0, overflows = 0;
  uint64_t samples_out = 0, dropped_out = 0;
  const char *transport = "-";
  int runs = 0, bins = 0, osh = 0;
  double spec_age_s = -1;
};

class Receiver {
 public:
  explicit Receiver(const Config &cfg);
  ~Receiver();

  // radio settings (ad9361-phy)
  void set_frequency(double hz);
  double get_frequency();
  void set_sample_rate(double sps);
  double get_sample_rate();
  void set_bandwidth(double hz);
  double get_bandwidth();
  void set_gain(double db);
  double get_gain();
  void set_agc(bool on);
  bool get_agc();

  void start();
  void stop();
  // blocking read of n complex samples (scaled to +-1.0 full scale); returns
  // the number read (< n only on timeout)
  size_t read(std::complex<float> *out, size_t n, long timeout_us);
  Stats stats();
  std::string ctl_command(const std::string &cmd);   // one command to sgchd, reply line(s)
  int bins() const { return N_; }
  bool running() const { return run_; }
  // offline use (uri "offline"): feed DMA words directly
  void feed_words(const uint32_t *w, size_t n) { run_ = true; parse_words(w, n); }

 private:
  void rx_thread();
  bool rx_stream_session();              // raw words from sgchd (port ctl_port+1)
  void ctl_thread();
  void parse_words(const uint32_t *w, size_t n);
  void frame_done();
  void synth_frame(uint32_t seq);
  void emit_noise_frames(uint32_t from, uint32_t to);
  void push_out(const std::complex<float> *s, size_t n);

  Config cfg_;
  std::string host_;
  int iiod_port_ = 30431;
  iiod::Client *attr_ = nullptr, *strm_ = nullptr;
  std::string phy_attr(const char *dir, const char *chan, const char *attr);
  void phy_write(const char *dir, const char *chan, const char *attr, const std::string &v);
  std::thread rx_thr_, ctl_thr_;
  std::atomic<bool> run_{false};
  volatile bool keep_ = false;
  std::atomic<sock_t> stream_sock_{SGCH_BADSOCK};

  // frame parser
  int L_ = 13, N_ = 8192, H_ = 4096;
  enum { P_SYNC, P_H1, P_H2, P_RUNHDR, P_DATA } pst_ = P_SYNC;
  uint32_t f_seq_ = 0, f_h2_ = 0;
  int f_nruns_ = 0, f_run_ = 0, f_osh_ = 0;
  int r_start_ = 0, r_len_ = 0, r_pos_ = 0;
  std::vector<std::complex<float>> X_;     // spectrum of the frame being parsed
  std::vector<uint8_t> have_;              // bin carries data in this frame
  bool have_last_ = false;
  uint32_t last_seq_ = 0;

  // synthesis
  std::vector<float> win_;
  std::vector<std::complex<float>> ola_;   // N: overlap buffer
  std::vector<std::complex<float>> tbuf_;
  void *plan_ = nullptr;
  float out_scale_ = 1.0f;
  uint32_t rng_ = 12345;
  std::vector<float> gtab_;                // gaussian table for the noise fill
  float gauss();

  // noise-fill spectrum (|X|^2 per bin, absolute units)
  std::mutex spec_mtx_;
  std::vector<float> fill_amp_;
  double spec_time_ = 0;

  // output ring
  std::mutex out_mtx_;
  std::condition_variable out_cv_;
  std::vector<std::complex<float>> ring_;
  size_t r_head_ = 0, r_tail_ = 0, r_fill_ = 0;

  std::mutex st_mtx_;
  Stats st_;
};

}  // namespace sgch
