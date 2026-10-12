// SPDX-License-Identifier: MIT
//
// channels -- per-downlink narrowband IQ for SDRangel.
//
// sgchd (detector v2) publishes its tracks (TRACKS command). For every streaming
// track a "slot" rebuilds only that downlink's bins with a small IFFT
// (decimated sub-band overlap-add synthesis, M bins -> fs*M/N samples/s), shifts
// the downlink to 0 Hz and serves it on its own SDRangel Remote TCP port
// (SDRA protocol, zlib-framed IQ so that the server can push centre-frequency
// changes when a slot is given to another downlink).
//
// Slot k listens on base_port + k (default 7301: ports around 1235 are often reserved by Windows). A slot keeps its downlink until the track
// expires (plus a linger time); its sample stream never stops (zeros while idle),
// so SDRangel never sees gaps.
#pragma once
#include <atomic>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "net.hpp"

namespace sgch {

struct TrackInfo {
  int id = 0;
  std::string state;
  double fc_hz = 0, bw_hz = 0, snr_db = 0, z = 0, conf = 0, age_s = 0, last_s = 0;
  int parent = -1, independent = 1, streaming = 0;
  double ext_lo_hz = 0, ext_hi_hz = 0;
};

struct ChannelConfig {
  int nslots = 8;
  int base_port = 7301;
  std::string bind = "127.0.0.1";
  int slot_bins = 16;          // M (power of two): slot rate = fs * M / N
  double linger_s = 3.0;       // keep a slot after its track expired
  bool children = true;        // independent narrow-in-wide tracks get slots too
  std::string list_file;       // channel list (text), rewritten on every change
  bool verbose = false;
};

class Slot;

class ChannelMux {
 public:
  explicit ChannelMux(const ChannelConfig &c);
  ~ChannelMux();
  void start();
  void stop();
  // radio context (from sgchd): LO, sample rate, number of bins
  void set_radio(double lo_hz, double fs, int N);
  // new track list (polled from sgchd)
  void update_tracks(const std::vector<TrackInfo> &t);
  // one analysis frame from the channelizer (natural bin order, absolute FFT units);
  // have[b] = bin carried data. lost = this frame never arrived (zeros).
  void frame(uint32_t seq, const std::complex<float> *X, const uint8_t *have, int N, float out_scale);
  std::string table();   // current assignment, human readable
  // test hook: samples produced per slot are also appended to these files (cf32)
  void set_dump(const std::string &prefix) { dump_prefix_ = prefix; }

 private:
  void assign_locked();
  void write_list_locked();
  ChannelConfig c_;
  std::mutex mtx_;
  std::vector<Slot *> slots_;
  std::vector<TrackInfo> tracks_;
  double lo_ = 0, fs_ = 0;
  int N_ = 0;
  std::string dump_prefix_;
  std::string last_table_;
};

// parse the reply of sgchd "TRACKS"
bool parse_tracks(const std::string &reply, std::vector<TrackInfo> &out, double *lo, double *fs, double *binw);

}  // namespace sgch
