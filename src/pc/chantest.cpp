// offline test of the channel slots: chantest words.bin schedule.txt out_prefix
//   words.bin    : DMA words (as sent by sgchd on port 30471)
//   schedule.txt : "AT <word offset>" lines, each followed by a sgchd TRACKS reply, "END"
// writes out_prefix<k>.cf32 for every slot and prints the assignment table.
#include "channels.hpp"
#include "sgch_rx.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
int main(int argc, char **argv) {
  if (argc < 4) { fprintf(stderr, "usage: chantest words.bin schedule.txt out_prefix [slot_bins]\n"); return 2; }
  FILE *f = fopen(argv[1], "rb");
  if (!f) return 1;
  std::vector<uint32_t> w;
  uint32_t b[65536];
  size_t k;
  while ((k = fread(b, 4, 65536, f)) > 0) w.insert(w.end(), b, b + k);
  fclose(f);
  struct Ev { size_t at; std::string text; };
  std::vector<Ev> ev;
  std::ifstream in(argv[2]);
  std::string line, cur;
  size_t at = 0;
  bool inblk = false;
  while (std::getline(in, line)) {
    if (line.rfind("AT ", 0) == 0) { at = std::stoull(line.substr(3)); cur.clear(); inblk = true; }
    else if (line == "END") { ev.push_back({at, cur}); inblk = false; }
    else if (inblk) cur += line + "\n";
  }
  sgch::ChannelConfig cc;
  cc.nslots = 8;
  cc.slot_bins = argc > 4 ? atoi(argv[4]) : 16;
  sgch::ChannelMux mux(cc);
  mux.set_dump(argv[3]);
  sgch::Config c;
  c.uri = "offline";
  c.wideband = false;
  c.on_frame = [&](uint32_t seq, const std::complex<float> *X, const uint8_t *have, int N, float sc) { mux.frame(seq, X, have, N, sc); };
  sgch::Receiver rx(c);
  size_t pos = 0, e = 0;
  while (pos < w.size()) {
    size_t next = (e < ev.size()) ? ev[e].at : w.size();
    if (next > pos) { rx.feed_words(w.data() + pos, next - pos); pos = next; }
    if (e < ev.size()) {
      std::vector<sgch::TrackInfo> t;
      double lo, fs, binw;
      if (sgch::parse_tracks(ev[e].text, t, &lo, &fs, &binw)) {
        mux.set_radio(lo, fs, (int)lround(fs / binw));
        mux.update_tracks(t);
      }
      e++;
    }
  }
  printf("%s", mux.table().c_str());
  return 0;
}
