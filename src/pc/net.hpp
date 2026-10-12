// SPDX-License-Identifier: MIT
//
// Minimal portable TCP helpers (Winsock / POSIX) for sgch_rx.
#pragma once
#include <cstdint>
#include <cstring>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define SGCH_BADSOCK INVALID_SOCKET
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#else
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
#define SGCH_BADSOCK (-1)
#endif

namespace net {

inline void init() {
#ifdef _WIN32
  static bool done = false;
  if (!done) {
    WSADATA w;
    WSAStartup(MAKEWORD(2, 2), &w);
    done = true;
  }
#endif
}

inline void close_sock(sock_t s) {
  if (s == SGCH_BADSOCK) return;
#ifdef _WIN32
  closesocket(s);
#else
  ::close(s);
#endif
}

inline void shutdown_sock(sock_t s) {
  if (s == SGCH_BADSOCK) return;
#ifdef _WIN32
  ::shutdown(s, SD_BOTH);
#else
  ::shutdown(s, SHUT_RDWR);
#endif
}

inline void set_timeout(sock_t s, int ms) {
#ifdef _WIN32
  DWORD t = (DWORD)ms;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&t, sizeof(t));
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char *)&t, sizeof(t));
#else
  timeval tv{ms / 1000, (ms % 1000) * 1000};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif
}

inline void set_bufsize(sock_t s, int bytes) {
  setsockopt(s, SOL_SOCKET, SO_RCVBUF, (const char *)&bytes, sizeof(bytes));
  setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char *)&bytes, sizeof(bytes));
}

inline void set_nodelay(sock_t s, bool on) {
  int v = on ? 1 : 0;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&v, sizeof(v));
}

// true when the last recv/send failed only because of the timeout / a signal
inline bool timed_out() {
#ifdef _WIN32
  int e = WSAGetLastError();
  return e == WSAETIMEDOUT || e == WSAEWOULDBLOCK || e == WSAEINTR;
#else
  return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
#endif
}

// timeout_ms <= 0: no timeout
inline sock_t connect_tcp(const std::string &host, int port, int timeout_ms) {
  init();
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) || !res) return SGCH_BADSOCK;
  sock_t fd = SGCH_BADSOCK;
  for (addrinfo *a = res; a; a = a->ai_next) {
    fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (fd == SGCH_BADSOCK) continue;
    if (timeout_ms > 0) set_timeout(fd, timeout_ms);
    if (::connect(fd, a->ai_addr, (int)a->ai_addrlen) == 0) break;
    close_sock(fd);
    fd = SGCH_BADSOCK;
  }
  freeaddrinfo(res);
  return fd;
}

inline sock_t listen_tcp(const std::string &bind_addr, int port) {
  init();
  sock_t s = ::socket(AF_INET, SOCK_STREAM, 0);
  if (s == SGCH_BADSOCK) return s;
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *)&one, sizeof(one));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  if (inet_pton(AF_INET, bind_addr.c_str(), &a.sin_addr) != 1) a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(s, (sockaddr *)&a, sizeof(a)) || ::listen(s, 1)) {
    close_sock(s);
    return SGCH_BADSOCK;
  }
  return s;
}

inline long recv_some(sock_t s, void *b, size_t n) { return (long)::recv(s, (char *)b, (int)n, 0); }

inline bool send_all(sock_t s, const void *b, size_t n) {
  const char *p = (const char *)b;
  while (n) {
    long k = (long)::send(s, p, (int)(n > (1u << 30) ? (1u << 30) : n), MSG_NOSIGNAL);
    if (k <= 0) {
      if (k < 0 && timed_out()) continue;
      return false;
    }
    p += k;
    n -= (size_t)k;
  }
  return true;
}

// Buffered reader for line / binary protocols
class Reader {
 public:
  explicit Reader(sock_t s = SGCH_BADSOCK) : s_(s) {}
  void reset(sock_t s) { s_ = s; pos_ = len_ = 0; }
  // false on EOF / error (a receive timeout is retried while *keep is true)
  bool fill(const volatile bool *keep = nullptr) {
    for (;;) {
      if (pos_ == len_) pos_ = len_ = 0;
      long k = recv_some(s_, buf_ + len_, sizeof(buf_) - len_);
      if (k > 0) { len_ += (size_t)k; return true; }
      if (k < 0 && timed_out() && (!keep || *keep)) { if (!keep) return false; continue; }
      return false;
    }
  }
  bool line(std::string &out, const volatile bool *keep = nullptr) {
    out.clear();
    for (;;) {
      while (pos_ < len_) {
        char c = buf_[pos_++];
        if (c == '\n') {
          if (!out.empty() && out.back() == '\r') out.pop_back();
          return true;
        }
        out.push_back(c);
        if (out.size() > 65536) return false;
      }
      if (!fill(keep)) return false;
    }
  }
  bool read(void *dst, size_t n, const volatile bool *keep = nullptr) {
    char *d = (char *)dst;
    while (n) {
      if (pos_ == len_) {
        // large reads go straight into the destination
        if (n >= sizeof(buf_) / 2) {
          long k = recv_some(s_, d, n);
          if (k > 0) { d += k; n -= (size_t)k; continue; }
          if (k < 0 && timed_out() && keep && *keep) continue;
          return false;
        }
        if (!fill(keep)) return false;
      }
      size_t k = len_ - pos_ < n ? len_ - pos_ : n;
      memcpy(d, buf_ + pos_, k);
      pos_ += k; d += k; n -= k;
    }
    return true;
  }

 private:
  sock_t s_;
  char buf_[65536];
  size_t pos_ = 0, len_ = 0;
};

}  // namespace net
