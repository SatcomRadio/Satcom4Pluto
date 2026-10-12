#include "sgch_rx.hpp"
#include <cstdio>
#include <vector>
int main(int argc, char **argv) {
  sgch::Config c; c.uri = "offline"; c.noise_fill = false; c.ring_samples = 1 << 26;
  sgch::Receiver rx(c);
  FILE *f = fopen(argv[1], "rb"); std::vector<uint32_t> w(1 << 22);
  size_t n = fread(w.data(), 4, w.size(), f); w.resize(n);
  rx.feed_words(w.data(), w.size());
  FILE *o = fopen(argv[2], "wb");
  std::vector<std::complex<float>> b(1 << 16); size_t k, tot = 0;
  while ((k = rx.read(b.data(), b.size(), 1000)) > 0) { fwrite(b.data(), 8, k, o); tot += k; }
  fclose(o);
  auto st = rx.stats();
  printf("words %zu frames %llu lost %llu resyncs %llu runs %d bins %d osh %d samples %zu\n", n, (unsigned long long)st.frames, (unsigned long long)st.lost_frames, (unsigned long long)st.resyncs, st.runs, st.bins, st.osh, tot);
}
