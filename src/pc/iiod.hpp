// SPDX-License-Identifier: MIT
//
// Tiny client for the iiod text protocol (libiio 0.x network backend,
// TCP port 30431). Replaces libiio so sgch_rx builds as one static .exe.
#pragma once
#include "net.hpp"

#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <string>

namespace iiod {

class Client {
 public:
  Client() = default;
  ~Client() { close(); }

  bool open(const std::string &host, int port = 30431, int timeout_ms = 5000) {
    close();
    s_ = net::connect_tcp(host, port, timeout_ms);
    if (s_ == SGCH_BADSOCK) return false;
    net::set_nodelay(s_, true);
    rd_.reset(s_);
    host_ = host;
    port_ = port;
    return true;
  }
  void close() {
    net::close_sock(s_);
    s_ = SGCH_BADSOCK;
  }
  void abort() { net::shutdown_sock(s_); }   // unblock a READBUF from another thread
  bool ok() const { return s_ != SGCH_BADSOCK; }
  void set_bufsize(int b) { net::set_bufsize(s_, b); }
  void set_timeout(int ms) { net::set_timeout(s_, ms); }

  // "dir" is "INPUT" or "OUTPUT"
  std::string read_attr(const std::string &dev, const char *dir, const std::string &chan, const std::string &attr) {
    std::lock_guard<std::mutex> g(m_);
    if (!command("READ " + dev + " " + dir + " " + chan + " " + attr)) return "";
    long n = reply_int();
    if (n < 0) return "";
    std::string v((size_t)n + 1, '\0');   // + trailing '\n'
    if (!rd_.read(&v[0], v.size())) { close(); return ""; }
    v.resize((size_t)n);
    while (!v.empty() && (v.back() == '\0' || v.back() == '\n')) v.pop_back();
    return v;
  }
  long write_attr(const std::string &dev, const char *dir, const std::string &chan, const std::string &attr,
                  const std::string &val) {
    std::lock_guard<std::mutex> g(m_);
    std::string data = val;
    data.push_back('\0');
    if (!command("WRITE " + dev + " " + dir + " " + chan + " " + attr + " " + std::to_string(data.size())))
      return -5;
    if (!net::send_all(s_, data.data(), data.size())) { close(); return -5; }
    return reply_int();
  }

  // streaming (use a dedicated Client for this)
  long open_buffer(const std::string &dev, size_t samples, const std::string &mask_hex) {
    if (!command("OPEN " + dev + " " + std::to_string(samples) + " " + mask_hex)) return -5;
    return reply_int();
  }
  void close_buffer(const std::string &dev) {
    if (command("CLOSE " + dev)) reply_int();
  }
  // reads exactly "bytes" bytes (several chunks); returns bytes or <0 (-errno)
  long read_buffer(const std::string &dev, void *dst, size_t bytes, const volatile bool *keep) {
    if (!command("READBUF " + dev + " " + std::to_string(bytes))) return -5;
    char *p = (char *)dst;
    size_t got = 0;
    bool first = true;   // like libiio: the channel mask follows the first chunk of every READBUF
    while (got < bytes) {
      std::string l;
      if (!rd_.line(l, keep)) { close(); return -5; }
      long n = strtol(l.c_str(), nullptr, 10);
      if (n < 0) return n;
      if (n == 0) break;
      if (first) {
        std::string mask;
        if (!rd_.line(mask, keep)) { close(); return -5; }
        first = false;
      }
      if ((size_t)n > bytes - got) { close(); return -5; }
      if (!rd_.read(p + got, (size_t)n, keep)) { close(); return -5; }
      got += (size_t)n;
    }
    return (long)got;
  }

 private:
  bool command(const std::string &c) {
    if (s_ == SGCH_BADSOCK && !host_.empty()) open(host_, port_);
    if (s_ == SGCH_BADSOCK) return false;
    std::string l = c + "\r\n";
    if (!net::send_all(s_, l.data(), l.size())) { close(); return false; }
    return true;
  }
  long reply_int() {
    std::string l;
    if (!rd_.line(l)) { close(); return -5; }
    return strtol(l.c_str(), nullptr, 10);
  }

  sock_t s_ = SGCH_BADSOCK;
  net::Reader rd_;
  std::mutex m_;
  std::string host_;
  int port_ = 30431;
};

}  // namespace iiod
