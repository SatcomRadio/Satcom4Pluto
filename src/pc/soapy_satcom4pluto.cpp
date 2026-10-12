// SPDX-License-Identifier: MIT
//
// SoapySDR device "satcom4pluto": a Pluto running the satellite-downlink
// channelizer, seen as an ordinary wideband receiver (CF32 / CS16).
//   driver=satcom4pluto,uri=ip:192.168.5.5[,ctl=HOST][,ctl_port=30470][,noise_fill=1]
//   [,noise_db=0][,buffer=65536][,transport=stream|auto|iiod][,bandwidth=Hz (default: follows the rate)]
//   [,options=<sgchd options>]          also settable as Soapy setting "sgchd" (OpenWebRX: live)
//
// Self-healing (OpenWebRX writes a source off when its connector process dies, so the
// device must never fail): it opens even when the Pluto is not there yet, keeps the wanted
// radio settings, and a supervisor thread
//   * connects / reconnects to the Pluto,
//   * re-applies LO, rate, gain, analog bandwidth when the Pluto lost them (power cycle,
//     satcom4pluto reload: its "initialize" resets the AD9363),
//   * re-sends the detector options to sgchd after every (re)connection.
// While no data arrive, readStream() delivers zeros in real time (blank waterfall, no errors).
#include "sgch_rx.hpp"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

namespace {

using clk = std::chrono::steady_clock;
static double secs(clk::duration d) { return std::chrono::duration<double>(d).count(); }

struct Stream {
  bool cs16 = false;
  std::vector<std::complex<float>> tmp;
};

class Satcom4PlutoDevice : public SoapySDR::Device {
 public:
  explicit Satcom4PlutoDevice(const SoapySDR::Kwargs &args) {
    auto get = [&](const char *k, const std::string &d) {
      auto it = args.find(k);
      return it == args.end() ? d : it->second;
    };
    cfg_.uri = get("uri", "ip:192.168.5.5");
    cfg_.ctl_host = get("ctl", "");
    cfg_.ctl_port = std::stoi(get("ctl_port", "30470"));
    cfg_.noise_fill = get("noise_fill", "1") != "0";
    cfg_.noise_db = std::stod(get("noise_db", "0"));
    cfg_.buffer_words = std::stoul(get("buffer", "65536"));
    std::string tr = get("transport", "stream");   // stream only: keeps retrying sgchd's port
    cfg_.transport = tr == "auto" ? 0 : tr == "iiod" ? 2 : 1;
    std::string bw = get("bandwidth", "");
    if (!bw.empty()) { bw_explicit_ = true; want_bw_ = std::stod(bw); }
    options_ = get("options", "-f 240:270");
    connect();   // may fail: the supervisor keeps trying
    sup_ = std::thread(&Satcom4PlutoDevice::supervisor, this);
  }
  ~Satcom4PlutoDevice() override {
    quit_ = true;
    if (sup_.joinable()) sup_.join();
    std::lock_guard<std::mutex> g(mtx_);
    if (rx_) rx_->stop();
    rx_.reset();
  }

  std::string getDriverKey() const override { return "satcom4pluto"; }
  std::string getHardwareKey() const override { return "Plutosatcom4pluto"; }
  SoapySDR::Kwargs getHardwareInfo() const override {
    SoapySDR::Kwargs k;
    auto r = cur();
    k["connected"] = r ? "yes" : "no";
    if (r) k["bins"] = std::to_string(r->bins());
    return k;
  }

  size_t getNumChannels(const int dir) const override { return dir == SOAPY_SDR_RX ? 1 : 0; }
  bool getFullDuplex(const int, const size_t) const override { return false; }

  // ---------------------------------------------------------------- stream
  std::vector<std::string> getStreamFormats(const int, const size_t) const override {
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16};
  }
  std::string getNativeStreamFormat(const int, const size_t, double &fullScale) const override {
    fullScale = 1.0;
    return SOAPY_SDR_CF32;
  }
  SoapySDR::Stream *setupStream(const int dir, const std::string &format, const std::vector<size_t> &,
                                const SoapySDR::Kwargs &) override {
    if (dir != SOAPY_SDR_RX) throw std::runtime_error("satcom4pluto is receive-only");
    if (format != SOAPY_SDR_CF32 && format != SOAPY_SDR_CS16)
      throw std::runtime_error("satcom4pluto: format must be CF32 or CS16");
    auto *s = new Stream;
    s->cs16 = (format == SOAPY_SDR_CS16);
    return reinterpret_cast<SoapySDR::Stream *>(s);
  }
  void closeStream(SoapySDR::Stream *st) override {
    deactivateStream(st, 0, 0);
    delete reinterpret_cast<Stream *>(st);
  }
  size_t getStreamMTU(SoapySDR::Stream *) const override { return 65536; }
  int activateStream(SoapySDR::Stream *, const int, const long long, const size_t) override {
    std::lock_guard<std::mutex> g(mtx_);
    active_ = true;
    if (rx_) rx_->start();
    t_data_ = t_zero_ = clk::now();
    return 0;
  }
  int deactivateStream(SoapySDR::Stream *, const int, const long long) override {
    std::lock_guard<std::mutex> g(mtx_);
    active_ = false;
    if (rx_) rx_->stop();
    return 0;
  }
  int readStream(SoapySDR::Stream *st, void *const *buffs, const size_t numElems, int &flags,
                 long long &timeNs, const long timeoutUs) override {
    auto *s = reinterpret_cast<Stream *>(st);
    flags = 0;
    timeNs = 0;
    size_t n = std::min<size_t>(numElems, 65536);
    s->tmp.resize(n);
    auto r = cur();
    size_t k = 0;
    auto now = clk::now();
    bool outage = secs(now - t_data_) > 1.0;
    if (r && !outage) k = r->read(s->tmp.data(), n, timeoutUs);
    else if (r) k = r->read(s->tmp.data(), n, 0);   // poll: data may be back
    now = clk::now();
    if (k) {
      t_data_ = t_zero_ = now;
    } else if (secs(now - t_data_) > 1.0) {
      // no data from the Pluto: keep the stream alive with zeros in real time
      double fs = want_rate_.load() > 0 ? want_rate_.load() : 1e6;
      double due = secs(now - t_zero_) * fs;
      if (due < (double)n) {   // wait (at most the caller's timeout) until a full buffer is due
        double w = std::min(((double)n - due) / fs, timeoutUs / 1e6);
        std::this_thread::sleep_for(std::chrono::duration<double>(w));
        due = secs(clk::now() - t_zero_) * fs;
      }
      if (due > 4.0 * n) { t_zero_ = clk::now(); due = n; }   // never burst after a long pause
      k = std::min(n, (size_t)due);
      if (!k) return SOAPY_SDR_TIMEOUT;
      t_zero_ += std::chrono::duration_cast<clk::duration>(std::chrono::duration<double>(k / fs));
      std::fill(s->tmp.begin(), s->tmp.begin() + k, std::complex<float>(0, 0));
    } else {
      if (!r) std::this_thread::sleep_for(std::chrono::milliseconds(50));   // never spin
      return SOAPY_SDR_TIMEOUT;
    }
    if (!s->cs16) {
      memcpy(buffs[0], s->tmp.data(), k * sizeof(std::complex<float>));
    } else {
      int16_t *o = reinterpret_cast<int16_t *>(buffs[0]);
      for (size_t i = 0; i < k; i++) {
        float re = s->tmp[i].real() * 32767.0f, im = s->tmp[i].imag() * 32767.0f;
        o[2 * i] = (int16_t)std::max(-32768.0f, std::min(32767.0f, re));
        o[2 * i + 1] = (int16_t)std::max(-32768.0f, std::min(32767.0f, im));
      }
    }
    return (int)k;
  }

  // ---------------------------------------------------------------- frequency
  std::vector<std::string> listFrequencies(const int, const size_t) const override { return {"RF"}; }
  void setFrequency(const int dir, const size_t, const std::string &, const double f,
                    const SoapySDR::Kwargs &) override {
    if (dir != SOAPY_SDR_RX) return;
    want_freq_ = f;
    with_rx([&](sgch::Receiver &r) { r.set_frequency(f); });
  }
  double getFrequency(const int, const size_t, const std::string &) const override { return want_freq_; }
  SoapySDR::RangeList getFrequencyRange(const int, const size_t, const std::string &) const override {
    return {SoapySDR::Range(70e6, 6000e6)};
  }

  // ---------------------------------------------------------------- sample rate / bandwidth
  void setSampleRate(const int dir, const size_t, const double r) override {
    if (dir != SOAPY_SDR_RX) return;
    want_rate_ = r;
    // OpenWebRX & co. never set the analog bandwidth: follow the rate (36.864 MS/s -> 40 MHz,
    // measured flat to +-15 MHz within 2.2 dB) unless bandwidth= or setBandwidth() was used
    if (!bw_explicit_) want_bw_ = auto_bw(r);
    with_rx([&](sgch::Receiver &x) {
      x.set_sample_rate(r);
      x.set_bandwidth(want_bw_);
    });
  }
  double getSampleRate(const int, const size_t) const override { return want_rate_; }
  std::vector<double> listSampleRates(const int, const size_t) const override {
    return {10e6, 15.36e6, 20e6, 25e6, 30e6, 30.72e6, 36.864e6, 40e6, 50e6, 61.44e6};
  }
  SoapySDR::RangeList getSampleRateRange(const int, const size_t) const override {
    return {SoapySDR::Range(2.5e6, 61.44e6)};
  }
  void setBandwidth(const int, const size_t, const double bw) override {
    if (bw <= 0) return;
    bw_explicit_ = true;
    want_bw_ = bw;
    with_rx([&](sgch::Receiver &r) { r.set_bandwidth(bw); });
  }
  double getBandwidth(const int, const size_t) const override { return want_bw_; }
  SoapySDR::RangeList getBandwidthRange(const int, const size_t) const override {
    return {SoapySDR::Range(200e3, 56e6)};
  }

  // ---------------------------------------------------------------- gain
  std::vector<std::string> listGains(const int, const size_t) const override { return {"PGA"}; }
  bool hasGainMode(const int, const size_t) const override { return true; }
  void setGainMode(const int, const size_t, const bool automatic) override {
    want_agc_ = automatic;
    with_rx([&](sgch::Receiver &r) { r.set_agc(automatic); });
  }
  bool getGainMode(const int, const size_t) const override { return want_agc_; }
  void setGain(const int, const size_t, const double v) override { set_gain(v); }
  void setGain(const int, const size_t, const std::string &, const double v) override { set_gain(v); }
  double getGain(const int, const size_t) const override { return want_gain_; }
  double getGain(const int, const size_t, const std::string &) const override { return want_gain_; }
  SoapySDR::Range getGainRange(const int, const size_t) const override { return SoapySDR::Range(-3, 71, 1); }
  SoapySDR::Range getGainRange(const int, const size_t, const std::string &) const override {
    return SoapySDR::Range(-3, 71, 1);
  }

  // ---------------------------------------------------------------- settings
  SoapySDR::ArgInfoList getSettingInfo() const override {
    SoapySDR::ArgInfoList l;
    SoapySDR::ArgInfo a;
    a.key = "sgchd";
    a.name = "Detector options";
    a.description = "sgchd options, e.g. -f 240:270 --max-snr-db 40 --kernel sin3";
    a.type = SoapySDR::ArgInfo::STRING;
    l.push_back(a);
    return l;
  }
  void writeSetting(const std::string &key, const std::string &value) override {
    if (key != "sgchd") return;
    {
      std::lock_guard<std::mutex> g(opt_mtx_);
      options_ = value;
      opt_gen_++;      // the supervisor sends it (now if connected, else after connecting)
    }
  }
  std::string readSetting(const std::string &key) const override {
    auto r = cur();
    if (key == "sgchd") { std::lock_guard<std::mutex> g(opt_mtx_); return options_; }
    if (!r) return "not connected";
    if (key == "status") return r->ctl_command("STATUS");
    if (key == "tracks") return r->ctl_command("TRACKS");
    return "";
  }

 private:
  static double auto_bw(double r) { return std::min(56e6, std::max(200e3, std::round(r * 1.085 / 1e5) * 1e5)); }

  std::shared_ptr<sgch::Receiver> cur() const {
    std::lock_guard<std::mutex> g(mtx_);
    return rx_;
  }
  template <class F> void with_rx(F f) {
    auto r = cur();
    if (!r) return;
    std::lock_guard<std::mutex> g(radio_mtx_);
    f(*r);
  }
  void set_gain(double v) {
    want_gain_ = v;
    want_agc_ = false;
    with_rx([&](sgch::Receiver &r) {
      r.set_agc(false);
      r.set_gain(v);
    });
  }
  void apply_radio(sgch::Receiver &r) {
    std::lock_guard<std::mutex> g(radio_mtx_);
    if (want_rate_ > 0) r.set_sample_rate(want_rate_);
    if (want_bw_ > 0) r.set_bandwidth(want_bw_);
    if (want_freq_ > 0) r.set_frequency(want_freq_);
    r.set_agc(want_agc_);
    if (!want_agc_ && want_gain_ > -100) r.set_gain(want_gain_);
  }
  // wanted settings still in place? (a Pluto power cycle / satcom4pluto reload resets the AD9363)
  bool radio_ok(sgch::Receiver &r) {
    std::lock_guard<std::mutex> g(radio_mtx_);
    double fs = r.get_sample_rate(), f = r.get_frequency();
    if (fs <= 0 || f <= 0) return false;
    if (want_rate_ > 0 && std::fabs(fs - want_rate_) > 1e-6 * want_rate_ + 1) return false;
    if (want_freq_ > 0 && std::fabs(f - want_freq_) > 1e3) return false;
    return true;
  }
  bool connect() {
    try {
      auto r = std::make_shared<sgch::Receiver>(cfg_);
      apply_radio(*r);
      {
        std::lock_guard<std::mutex> g(mtx_);
        rx_ = r;
        if (active_) rx_->start();
      }
      words_ = 0;
      t_words_ = clk::now();
      need_set_ = true;
      SoapySDR::logf(SOAPY_SDR_INFO, "satcom4pluto: connected to the Pluto at %s", cfg_.uri.c_str());
      return true;
    } catch (std::exception &e) {
      if (!warned_) SoapySDR::logf(SOAPY_SDR_WARNING, "satcom4pluto: %s -- retrying", e.what());
      warned_ = true;
      return false;
    }
  }
  void drop(const char *why) {
    SoapySDR::logf(SOAPY_SDR_WARNING, "satcom4pluto: %s -- reconnecting", why);
    std::shared_ptr<sgch::Receiver> old;
    {
      std::lock_guard<std::mutex> g(mtx_);
      old.swap(rx_);
    }
    if (old) old->stop();
    warned_ = false;
  }
  void supervisor() {
    int tick = 0;
    while (!quit_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (quit_) break;
      tick++;
      auto r = cur();
      if (!r) {
        if (tick % 4 == 0) connect();   // every 2 s
        continue;
      }
      auto now = clk::now();
      uint64_t w = r->stats().words;
      bool flowing = w != words_;
      if (flowing) { words_ = w; t_words_ = now; }
      if (tick % 4 == 0 && !radio_ok(*r)) {
        if (r->get_sample_rate() <= 0) { drop("lost the Pluto"); continue; }
        SoapySDR::logf(SOAPY_SDR_INFO, "satcom4pluto: radio settings lost (Pluto reset/reload?) -- restoring");
        apply_radio(*r);
        need_set_ = true;
      }
      if (active_ && secs(now - t_words_) > 10.0) {   // stream stalled: start over
        drop("no data from the Pluto for 10 s");
        continue;
      }
      uint64_t gen;
      {
        std::lock_guard<std::mutex> g(opt_mtx_);
        gen = opt_gen_;
      }
      if ((need_set_ || gen != sent_gen_) && (flowing || !active_)) {
        std::string o;
        {
          std::lock_guard<std::mutex> g(opt_mtx_);
          o = options_;
          gen = opt_gen_;
        }
        need_set_ = false;
        std::string rep = r->ctl_command("SET " + o);
        if (rep.compare(0, 2, "OK") == 0) {
          sent_gen_ = gen;
          SoapySDR::logf(SOAPY_SDR_INFO, "satcom4pluto: detector options set: %s", o.c_str());
        } else {
          need_set_ = true;   // retry next round
        }
      }
    }
  }

  sgch::Config cfg_;
  mutable std::mutex mtx_, opt_mtx_;
  std::mutex radio_mtx_;
  std::shared_ptr<sgch::Receiver> rx_;
  std::thread sup_;
  std::atomic<bool> quit_{false}, active_{false}, need_set_{true}, warned_{false};
  std::atomic<double> want_freq_{0}, want_rate_{0}, want_bw_{0}, want_gain_{-1000};
  std::atomic<bool> want_agc_{false}, bw_explicit_{false};
  std::string options_;
  uint64_t opt_gen_ = 1, sent_gen_ = 0;
  uint64_t words_ = 0;
  clk::time_point t_words_ = clk::now(), t_data_ = clk::now(), t_zero_ = clk::now();
};

SoapySDR::KwargsList find_satcom4pluto(const SoapySDR::Kwargs &args) {
  SoapySDR::KwargsList out;
  auto it = args.find("driver");
  if (it == args.end() || it->second != "satcom4pluto") return out;   // only on explicit request
  SoapySDR::Kwargs k = args;
  if (k.find("uri") == k.end()) k["uri"] = "ip:192.168.5.5";
  k["label"] = "Pluto satcom4pluto @ " + k["uri"];
  out.push_back(k);
  return out;
}

SoapySDR::Device *make_satcom4pluto(const SoapySDR::Kwargs &args) { return new Satcom4PlutoDevice(args); }

SoapySDR::Registry reg_satcom4pluto("satcom4pluto", &find_satcom4pluto, &make_satcom4pluto, SOAPY_SDR_ABI_VERSION);

}  // namespace
