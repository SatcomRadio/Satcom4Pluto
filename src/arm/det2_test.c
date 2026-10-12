/* host test: det2_test spectra.bin > tracks.txt
   spectra.bin: int32 N, int32 nupd, int32 kernel, float64 binw, float64 dt, then nupd*N float64 (centred) */
#include <stdio.h>
#include <stdlib.h>
#include "det2.h"
int main(int argc, char **argv) {
  FILE *f = fopen(argv[1], "rb");
  int hdr[3]; double bd[2];
  if (!f || fread(hdr, 4, 3, f) != 3 || fread(bd, 8, 2, f) != 2) return 1;
  int N = hdr[0], nu = hdr[1], kern = hdr[2];
  det2_cfg_t c; det2_cfg_default(&c);
  det2_t d;
  if (det2_init(&d, &c, N, bd[0], bd[1], kern)) return 2;
  double *P = malloc(N * 8);
  for (int u = 0; u < nu; u++) {
    if (fread(P, 8, N, f) != (size_t)N) return 3;
    det2_update(&d, P, (u + 1) * bd[1]);
    printf("U %d %d %d\n", u, d.ntr, d.nruns);
    for (int i = 0; i < d.ntr; i++) {
      det2_track_t *t = &d.tr[i];
      printf("T %d %s %.4f %.4f %d %d %d %d\n", t->id, det2_state_name(t->state), t->fc, t->bw, t->lo_bin, t->hi_bin, t->parent, t->independent);
    }
    for (int i = 0; i < d.nruns; i++) printf("R %d %d\n", d.runs[i].s, d.runs[i].e);
  }
  return 0;
}
