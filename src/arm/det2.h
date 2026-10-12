/*
 * SPDX-License-Identifier: MIT
 *
 * det2 -- satcom4pluto v2 detector (multiresolution adaptive detection, segmentation,
 * narrow-in-wide features, tracking, IQ interval allocation).
 * C port of ref/satref/det_v2.py (the reference); keep the two in step.
 *
 * Spectra are linear power in centred bin order (index N/2 = LO).
 */
#ifndef DET2_H
#define DET2_H
#include <stdint.h>

#define DET2_MAXW      8
#define DET2_MAXSPUR   32
#define DET2_MAXTRACK  512
#define DET2_MAXCAND   1024
#define DET2_HIST      64
#define DET2_NCACHE    12

enum { T_CAND = 0, T_CONF, T_ACTIVE, T_HOLD, T_EXPIRED, T_DROP };

typedef struct {
  double widths_hz[DET2_MAXW]; int nwidths;
  int seg; double q; int nbr; double base_alpha, slow_alpha;
  double z_det, z_det_slow, z_keep, z_edge, edge_frac, split_ratio, split_z;
  int narrow_max_bins; int narrow_env; double narrow_db, z_narrow;
  int confirm_m, confirm_n; double z_fast_confirm; int active_hits; double hold_s;
  int child_confirm; double child_corr_max; int stream_candidates;
  double dc_notch_hz; double spur_hz[DET2_MAXSPUR]; int nspur;   /* spurs: offsets from the LO, Hz */
  double guard_hz, bw_factor, narrow_extract_hz; int merge_gap_bins;
  int split_mature;            /* a valley splits only after this many updates of slow integration */
  double max_bw_hz;            /* a track never grows wider (dense bands: no runaway merging) */
} det2_cfg_t;

typedef struct {
  int s, e;            /* detected extent (bins) */
  double fc, bw, z, snr_db, p_ex;
  int slow, parent;    /* parent: -1 primary, -2 narrow feature inside a primary */
  int merged_into;
} det2_cand_t;

typedef struct {
  int id, state, parent, independent, merged;
  double fc, bw, snr_db, z, t_first, t_last, conf, ext;
  int sep_hits, absorb, nhits;
  uint8_t hits[16]; int nh;                  /* recent hit history (oldest first) */
  float phist[DET2_HIST]; int np_;           /* excess power history */
  float fhist[DET2_HIST], bhist[DET2_HIST]; int nfb;
  int lo_bin, hi_bin;
} det2_track_t;

typedef struct { int s, e; } det2_run_t;
typedef struct { int id, c, M, s, e, state; double fc; } det2_slot_t;

typedef struct {
  det2_cfg_t c;
  int N; double binw, dt, smear, t, cap_bw, cap_ext;
  int Wb[DET2_MAXW], nW;
  double sig_f[DET2_MAXW], sig_s[DET2_MAXW]; int have_sig;
  double *Blog, *Rs, *R, *Ef, *Es, *tmp, *tmp2, *Esm, *L;
  int *age, *age_now;
  double *cR, *cRs; int *cM;   /* prefix sums */
  double *cbuf;                 /* smoothed-profile cache (2 x DET2_NCACHE x N) */
  uint8_t *fixed_mask, *mask, *claimed, *ok;
  int fixed_pos[DET2_MAXSPUR + 1], nfixed;
  int have_B, have_Rs;
  det2_track_t tr[DET2_MAXTRACK]; int ntr; int next_id;
  det2_cand_t cand[DET2_MAXCAND]; int ncand;
  struct det2_win { double z; int ord; int16_t W; int8_t slow; int lo; } *wins; int nwins, capwins;
  det2_run_t runs[DET2_MAXTRACK]; int nruns;
  det2_run_t prev[DET2_MAXTRACK]; int nprev;   /* strong candidates of the previous update */
  det2_slot_t slots[DET2_MAXTRACK]; int nslots;
} det2_t;

void det2_cfg_default(det2_cfg_t *c);
/* kernel: 0 sin^5, 1 sin^3, 2 sin^1 (kernel smear used for bandwidth estimates) */
int  det2_init(det2_t *d, const det2_cfg_t *c, int N, double binw, double update_s, int kernel);
void det2_free(det2_t *d);
void det2_reset(det2_t *d);                 /* LO / rate / gain change: forget everything */
/* one update: P = centred linear spectrum; t = time (s). Fills tracks, runs, slots. */
void det2_update(det2_t *d, const double *P, double t);
const char *det2_state_name(int s);
/* extraction interval (centred bins) of a streaming track */
void det2_extract(const det2_t *d, const det2_track_t *t, int *s, int *e);
static inline int det2_streaming(const det2_t *d, const det2_track_t *t) {
  return t->state == T_CONF || t->state == T_ACTIVE || t->state == T_HOLD ||
         (d->c.stream_candidates && t->state == T_CAND);
}
#endif
