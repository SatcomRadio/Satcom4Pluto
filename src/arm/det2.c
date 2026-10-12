/*
 * SPDX-License-Identifier: MIT
 *
 * det2.c -- satcom4pluto v2 detector, C port of ref/satref/det_v2.py.
 * See det2.h and INSTALL.md ("Detector v2") for the algorithm.
 */
#include "det2.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const double SMEAR[3] = { 4.4, 3.5, 2.6 };   /* sin^5, sin^3, sin^1: 99 % width of a CW (bins) */

void det2_cfg_default(det2_cfg_t *c) {
  memset(c, 0, sizeof(*c));
  const double w[] = { 5000, 7000, 12500, 25000, 40000, 60000 };
  memcpy(c->widths_hz, w, sizeof(w)); c->nwidths = 6;
  c->seg = 32; c->q = 0.30; c->nbr = 3; c->base_alpha = 0.25; c->slow_alpha = 0.15;
  c->z_det = 5.5; c->z_det_slow = 6.0; c->z_keep = 3.0; c->z_edge = 2.0; c->edge_frac = 0.001;
  c->split_ratio = 0.5; c->split_z = 3.0;
  c->narrow_max_bins = 2; c->narrow_env = 7; c->narrow_db = 1.5; c->z_narrow = 6.0;
  c->confirm_m = 2; c->confirm_n = 3; c->z_fast_confirm = 15.0; c->active_hits = 5; c->hold_s = 2.0;
  c->child_confirm = 6; c->child_corr_max = 0.7; c->stream_candidates = 0;
  c->dc_notch_hz = 5000.0; c->nspur = 0;
  c->guard_hz = 4500.0; c->bw_factor = 1.25; c->narrow_extract_hz = 12500.0; c->merge_gap_bins = 2;
  c->split_mature = 7; c->max_bw_hz = 150000.0;
}

const char *det2_state_name(int s) {
  static const char *n[] = { "CANDIDATE", "CONFIRMED", "ACTIVE", "HOLD", "EXPIRED", "DROP" };
  return (s >= 0 && s <= 5) ? n[s] : "?";
}

/* ------------------------------------------------------------------ helpers */
static __attribute__((unused)) int cmp_d(const void *a, const void *b) {
  double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

/* k-th smallest (0-based), partial reorder of v */
static double kth(double *v, int n, int k) {
  int l = 0, r = n - 1;
  while (l < r) {
    double piv = v[(l + r) / 2];
    int i = l, j = r;
    while (i <= j) {
      while (v[i] < piv) i++;
      while (v[j] > piv) j--;
      if (i <= j) { double t = v[i]; v[i] = v[j]; v[j] = t; i++; j--; }
    }
    if (k <= j) r = j; else if (k >= i) l = i; else return v[k];
  }
  return v[k];
}

/* numpy median (mean of the two middle values for even n); reorders v */
static double median(double *v, int n) {
  if (n <= 0) return NAN;
  if (n & 1) return kth(v, n, n / 2);
  double a = kth(v, n, n / 2 - 1), b = kth(v, n, n / 2);
  return 0.5 * (a + b);
}

/* numpy quantile, 'linear' method; reorders v (n small) */
static double quantile(double *v, int n, double q) {
  for (int i = 1; i < n; i++) {                  /* insertion sort: n <= seg (32) */
    double t = v[i]; int k = i - 1;
    while (k >= 0 && v[k] > t) { v[k + 1] = v[k]; k--; }
    v[k + 1] = t;
  }
  double pos = q * (n - 1);
  int lo = (int)floor(pos);
  if (lo >= n - 1) return v[n - 1];
  return v[lo] + (pos - lo) * (v[lo + 1] - v[lo]);
}

/* mean over x[a .. a+W-1], a = j - (W-1)/2; out of range -> NAN */
static void movmean(const double *x, int N, int W, double *out) {
  int h = (W - 1) / 2;
  double s = 0;
  for (int j = 0; j < N; j++) out[j] = NAN;
  if (W > N) return;
  for (int i = 0; i < W; i++) s += x[i];
  for (int a = 0; a + W <= N; a++) {
    if (a > 0) s += x[a + W - 1] - x[a - 1];
    out[a + h] = s / W;
  }
}

/* numpy.interp on a uniform integer grid 0..N-1 with optional left/right values */
static double interp_grid(const double *f, int N, double x, double left, double right) {
  if (x < 0) return left;
  if (x > N - 1) return right;
  int i = (int)floor(x);
  if (i >= N - 1) return f[N - 1];
  double t = x - i;
  return f[i] + t * (f[i + 1] - f[i]);
}

/* numpy.interp(x, xp, fp) for non-decreasing xp */
static double interp_xy(double x, const double *xp, const double *fp, int n) {
  if (x <= xp[0]) return fp[0];
  if (x >= xp[n - 1]) return fp[n - 1];
  int lo = 0, hi = n - 1;
  while (hi - lo > 1) { int m = (lo + hi) / 2; if (xp[m] <= x) lo = m; else hi = m; }
  double dx = xp[hi] - xp[lo];
  if (dx <= 0) return fp[hi];
  return fp[lo] + (x - xp[lo]) * (fp[hi] - fp[lo]) / dx;
}

static double sigma_any_calc(const det2_t *d, const double *sd, double W) {
  int n = d->nW;
  if (W <= d->Wb[0]) return sd[0] * sqrt((double)d->Wb[0] / W);
  if (W >= d->Wb[n - 1]) return sd[n - 1] * sqrt((double)d->Wb[n - 1] / W);
  double lw = log(W);
  for (int i = 0; i + 1 < n; i++) {
    double a = log((double)d->Wb[i]), b = log((double)d->Wb[i + 1]);
    if (lw >= a && lw <= b) {
      double t = (lw - a) / (b - a);
      return exp(log(sd[i]) + t * (log(sd[i + 1]) - log(sd[i])));
    }
  }
  return sd[n - 1];
}

/* memoised: called ~10^4 times per update with a handful of tables and integer W */
static double sigma_any(const det2_t *d, const double *sd, double W) {
  enum { NM = 6, WM = 512 };
  static struct { double key[DET2_MAXW]; double v[WM + 1]; int used; } memo[NM];
  static int next;
  int wi = (int)W;
  if (wi != W || wi < 1 || wi > WM) return sigma_any_calc(d, sd, W);
  int m;
  for (m = 0; m < NM; m++) {
    if (!memo[m].used) continue;
    int k;
    for (k = 0; k < d->nW; k++) if (memo[m].key[k] != sd[k]) break;
    if (k == d->nW) break;
  }
  if (m == NM) {
    m = next; next = (next + 1) % NM;
    memo[m].used = 1;
    for (int k = 0; k < d->nW; k++) memo[m].key[k] = sd[k];
    for (int k = 0; k <= WM; k++) memo[m].v[k] = -1.0;
  }
  if (memo[m].v[wi] < 0) memo[m].v[wi] = sigma_any_calc(d, sd, W);
  return memo[m].v[wi];
}

/* ------------------------------------------------------------------ init */
int det2_init(det2_t *d, const det2_cfg_t *c, int N, double binw, double update_s, int kernel) {
  memset(d, 0, sizeof(*d));
  d->c = *c; d->N = N; d->binw = binw; d->dt = update_s;
  d->smear = SMEAR[(kernel >= 0 && kernel <= 2) ? kernel : 0];
  d->cap_bw = c->max_bw_hz / binw;
  d->cap_ext = d->cap_bw / 2 + d->smear / 2 + 1;
  /* sorted unique window widths in bins */
  for (int i = 0; i < c->nwidths && i < DET2_MAXW; i++) {
    int w = (int)rint(c->widths_hz[i] / binw);
    if (w < 1) w = 1;
    int dup = 0;
    for (int k = 0; k < d->nW; k++) if (d->Wb[k] == w) dup = 1;
    if (!dup) d->Wb[d->nW++] = w;
  }
  for (int i = 1; i < d->nW; i++) {
    int t = d->Wb[i], k = i - 1;
    while (k >= 0 && d->Wb[k] > t) { d->Wb[k + 1] = d->Wb[k]; k--; }
    d->Wb[k + 1] = t;
  }
#define AL(p, n, t) do { p = (t *)calloc((n), sizeof(t)); if (!p) return -1; } while (0)
  AL(d->Blog, N, double); AL(d->Rs, N, double); AL(d->R, N, double); AL(d->Ef, N, double);
  AL(d->Es, N, double); AL(d->tmp, N + 64, double); AL(d->tmp2, N + 64, double); AL(d->Esm, N, double);
  AL(d->L, N, double); AL(d->age, N, int); AL(d->cbuf, (size_t)2 * DET2_NCACHE * N, double);
  AL(d->cR, N + 1, double); AL(d->cRs, N + 1, double); AL(d->cM, N + 1, int); AL(d->age_now, N, int);
  AL(d->fixed_mask, N, uint8_t); AL(d->mask, N, uint8_t); AL(d->claimed, N, uint8_t); AL(d->ok, N, uint8_t);
  d->capwins = 4 * N;
  AL(d->wins, d->capwins, struct det2_win);
#undef AL
  int dc = (int)ceil(c->dc_notch_hz / binw);
  for (int j = N / 2 - dc; j <= N / 2 + dc; j++) if (j >= 0 && j < N) d->fixed_mask[j] = 1;
  d->nfixed = 0;
  if (c->dc_notch_hz > 0) d->fixed_pos[d->nfixed++] = N / 2;
  for (int i = 0; i < c->nspur; i++) {
    int j = (int)rint(N / 2.0 + c->spur_hz[i] / binw);
    for (int k = j - 1; k <= j + 1; k++) if (k >= 0 && k < N) d->fixed_mask[k] = 1;
    if (j >= 0 && j < N) d->fixed_pos[d->nfixed++] = j;
  }
  for (int k = 0; k < 4; k++) { d->fixed_mask[k] = 1; d->fixed_mask[N - 1 - k] = 1; }
  d->next_id = 1;
  return 0;
}

void det2_free(det2_t *d) {
  free(d->Blog); free(d->Rs); free(d->R); free(d->Ef); free(d->Es); free(d->tmp); free(d->tmp2);
  free(d->Esm); free(d->L); free(d->cbuf); free(d->cR); free(d->cRs); free(d->cM); free(d->age); free(d->age_now); free(d->fixed_mask); free(d->mask); free(d->claimed); free(d->ok); free(d->wins);
  memset(d, 0, sizeof(*d));
}

void det2_reset(det2_t *d) {
  d->have_B = d->have_Rs = d->have_sig = 0;
  memset(d->age, 0, d->N * sizeof(int));
  d->nprev = 0;
  d->ntr = 0; d->ncand = 0; d->nruns = 0; d->nslots = 0;
}

/* ------------------------------------------------------------------ baseline */
static void baseline(det2_t *d, const double *P, const uint8_t *mask, double sig1, double *out) {
  int N = d->N, S = d->c.seg, hop = S / 2;
  int nseg = (N - S) / hop + 1;
  static double lv[4096], lv2[4096], cen[4096];
  if (nseg > 4096) nseg = 4096;
  double corr = 1.0 + (-0.5244) * (sig1 < 0.5 ? sig1 : 0.5);
  int ngood = 0;
  for (int i = 0; i < nseg; i++) {
    int s = i * hop, n = 0;
    cen[i] = s + S / 2.0 - 0.5;
    for (int k = 0; k < S; k++) if (!mask[s + k]) d->tmp[n++] = P[s + k];
    if (n >= S / 4) { lv[i] = 10 * log10(quantile(d->tmp, n, d->c.q) / corr); ngood++; }
    else lv[i] = NAN;
  }
  if (ngood < 3) {
    for (int j = 0; j < N; j++) d->tmp[j] = P[j];
    double m = 10 * log10(median(d->tmp, N) + 1e-30);
    for (int i = 0; i < nseg; i++) lv[i] = m;
  } else {
    /* fill missing segments by interpolation between good ones */
    int nprev = -1;
    for (int i = 0; i < nseg; i++) {
      if (isfinite(lv[i])) { nprev = i; continue; }
      int nn = i + 1;
      while (nn < nseg && !isfinite(lv[nn])) nn++;
      if (nprev < 0) lv[i] = lv[nn];
      else if (nn >= nseg) lv[i] = lv[nprev];
      else lv[i] = lv[nprev] + (cen[i] - cen[nprev]) * (lv[nn] - lv[nprev]) / (cen[nn] - cen[nprev]);
    }
    /* the loop above writes interpolated values that later iterations would treat
       as good: harmless because numpy interpolates only between originally good points
       and a filled point lies on that same line */
  }
  int k = d->c.nbr;
  for (int i = 0; i < nseg; i++) {
    double pr[16]; int np_ = 0;
    for (int dd = 1; dd <= k && dd < 16; dd++)
      if (i - dd >= 0 && i + dd < nseg) pr[np_++] = (lv[i - dd] + lv[i + dd]) / 2;
    lv2[i] = lv[i];
    if (np_) {
      double m = median(pr, np_) + 1.0;
      if (m < lv2[i]) lv2[i] = m;
    }
  }
  for (int j = 0; j < N; j++) {
    if (j < cen[0]) out[j] = lv2[0] + (j - cen[0]) * (lv2[1] - lv2[0]) / (cen[1] - cen[0]);
    else if (j > cen[nseg - 1])
      out[j] = lv2[nseg - 1] + (j - cen[nseg - 1]) * (lv2[nseg - 1] - lv2[nseg - 2]) / (cen[nseg - 1] - cen[nseg - 2]);
    else {
      int i = (int)((j - cen[0]) / hop);
      if (i >= nseg - 1) i = nseg - 2;
      while (i + 1 < nseg - 1 && j > cen[i + 1]) i++;
      double t = (j - cen[i]) / (cen[i + 1] - cen[i]);
      out[j] = lv2[i] + t * (lv2[i + 1] - lv2[i]);
    }
  }
}

static double mad_sigma(double *v, int n, double *w) {
  if (n < 16) return NAN;
  for (int i = 0; i < n; i++) w[i] = v[i];
  double m = median(w, n);
  for (int i = 0; i < n; i++) w[i] = fabs(v[i] - m);
  return 1.4826 * median(w, n);
}

/* prefix sums: window means in O(1) (cR, cRs: R and Rs; cM: unmasked count) */
static void prefixes(det2_t *d, const uint8_t *mask) {
  int N = d->N;
  double a = 0, b = 0; int m = 0;
  d->cR[0] = d->cRs[0] = 0; d->cM[0] = 0;
  for (int j = 0; j < N; j++) {
    a += d->R[j]; b += d->Rs[j]; m += !mask[j];
    d->cR[j + 1] = a; d->cRs[j + 1] = b; d->cM[j + 1] = m;
  }
}

static void sigmas(det2_t *d, const uint8_t *mask, double *sf, double *ss) {
  int N = d->N;
  static double *vals, *work;
  static int cap;
  (void)mask;
  if (cap < N) { free(vals); free(work); vals = malloc(N * sizeof(double)); work = malloc(N * sizeof(double)); cap = N; }
  for (int wi = 0; wi < d->nW; wi++) {
    int W = d->Wb[wi], h = (W - 1) / 2;
    int st = W > 4 ? W : 4;                          /* decimate: speed, less correlation */
    for (int pass = 0; pass < 2; pass++) {
      const double *c = pass ? d->cRs : d->cR;
      int n = 0;
      for (int j = 0; j < N; j += st) {
        int a = j - h;
        if (a < 0 || a + W > N) continue;
        if (d->cM[a + W] - d->cM[a] != W) continue;  /* window fully unmasked */
        vals[n++] = (c[a + W] - c[a]) / W;
      }
      double s = mad_sigma(vals, n, work);
      double *prev = pass ? d->sig_s : d->sig_f;
      if (d->have_sig && isfinite(s)) s = prev[wi] + 0.2 * (s - prev[wi]);
      (pass ? ss : sf)[wi] = isfinite(s) ? s : 0.1;
    }
  }
}

/* ------------------------------------------------------------------ CLEAN */
static int side_peak_ge(const det2_t *d, const double *E, int j, int dd, double *mout) {
  double m = E[j];
  int k = j;
  while (k + dd >= 0 && k + dd < d->N && E[k + dd] >= m) { k += dd; m = E[k]; }
  *mout = m;
  return k;
}

static int extend(const det2_t *d, const double *E, const uint8_t *cl, int p, int dd, double thr, double sig) {
  int N = d->N, j = p, vpos = p;
  double run_max = E[p], vmin = E[p];
  while (j + dd > 0 && j + dd < N - 1 && !cl[j + dd]) {
    double v = E[j + dd];
    if (v < thr) break;
    if (v < vmin) { vmin = v; vpos = j + dd; }
    if (v > vmin && vpos != p && d->age_now[vpos] >= d->c.split_mature) {
      double rise = v - vmin, sp;
      double mn = run_max < v ? run_max : v;
      if (vmin <= d->c.split_ratio * mn && rise > d->c.split_z * sig) {
        side_peak_ge(d, E, j + dd, dd, &sp);
        if (sp > vmin / d->c.split_ratio) return vpos != p ? vpos - dd : p;
      }
    }
    if (v > run_max && vpos == p) run_max = v;
    j += dd;
  }
  return j;
}

static int is_tail(const det2_t *d, const double *E, const uint8_t *cl, int p) {
  int N = d->N;
  for (int dd = -1; dd <= 1; dd += 2) {
    int j = p;
    while (j + dd >= 0 && j + dd < N && !cl[j + dd] && E[j + dd] >= E[j] * 0.98) j += dd;
    if (j + dd >= 0 && j + dd < N && cl[j + dd]) return 1;
    if (p + dd >= 0 && p + dd < N && cl[p + dd]) return 1;
  }
  return 0;
}

static int cmp_win(const void *a, const void *b) {
  const struct det2_win *x = a, *y = b;
  if (x->z != y->z) return (x->z < y->z) - (x->z > y->z);
  return x->ord - y->ord;        /* stable, like Python's sort */
}

static int clean(det2_t *d, const double *sf, const double *ss, det2_cand_t *cands, int maxc) {
  const det2_cfg_t *c = &d->c;
  int N = d->N, nc = 0;
  double *m = d->tmp, *Ef = d->Ef, *Es = d->Es;
  for (int j = 0; j < N; j++) { Ef[j] = d->R[j] - 1.0; Es[j] = d->Rs[j] - 1.0; }
  /* candidate windows */
  d->nwins = 0;
  for (int wi = 0; wi < d->nW; wi++) {
    int W = d->Wb[wi], h = (W - 1) / 2;
    for (int sl = 0; sl < 2; sl++) {
      const double *cs = sl ? d->cRs : d->cR;
      double sd = sl ? ss[wi] : sf[wi], thr = sl ? c->z_det_slow : c->z_det, iw = 1.0 / W;
      for (int j = 0; j < N; j++) {
        int a = j - h;
        m[j] = (a < 0 || a + W > N) ? NAN : (cs[a + W] - cs[a]) * iw - 1.0;
      }
      for (int j = 0; j < N; j++) {
        double z = m[j] / sd;
        if (!isfinite(z) || !(z > thr)) continue;
        /* local maxima only (see det_v2.py) */
        if (j > 0 && isfinite(m[j - 1]) && m[j - 1] / sd > z) continue;
        if (j < N - 1 && isfinite(m[j + 1]) && m[j + 1] / sd > z) continue;
        int lo = j - h, nf = 0;
        for (int k = lo; k < lo + W; k++) nf += d->fixed_mask[k];
        if (nf * 2 > W) continue;
        if (d->nwins >= d->capwins) continue;
        struct det2_win *w = &d->wins[d->nwins++];
        w->z = z; w->ord = d->nwins - 1; w->W = W; w->lo = lo; w->slow = sl;
      }
    }
  }
  /* stable sort by z, descending (stable: same order as Python for equal z) */
  qsort(d->wins, d->nwins, sizeof(d->wins[0]), cmp_win);
  uint8_t *cl = d->claimed;
  memcpy(cl, d->fixed_mask, N);
  /* DC / spurs skirts */
  double sg1 = sigma_any(d, sf, 1);
  static uint8_t *zero;
  static int zcap;
  if (zcap < N) { free(zero); zero = calloc(N, 1); zcap = N; }
  for (int i = 0; i < d->nfixed; i++) {
    int p = d->fixed_pos[i];
    if (p > 0 && p < N - 1 && Ef[p] > c->z_edge * sg1) {
      int s0 = extend(d, Ef, zero, p, -1, c->z_edge * sg1, sg1);
      int e0 = extend(d, Ef, zero, p, +1, c->z_edge * sg1, sg1);
      for (int k = s0; k <= e0; k++) cl[k] = 1;
    }
  }
  /* smoothed profiles, cached per (spectrum, smoothing length): Z = claims zeroed
     (updated locally after every claim), T = without claims (tail test) */
  struct { int slow, sm; double *Z, *T; } cache[DET2_NCACHE];
  int ncache = 0;
  double *src = d->tmp2;
  for (int wi = 0; wi < d->nwins && nc < maxc; wi++) {
    int W = d->wins[wi].W, lo = d->wins[wi].lo, hi = lo + W, slow = d->wins[wi].slow;
    double ctr = lo + (W - 1) / 2.0;
    if (cl[(int)rint(ctr)]) continue;
    const double *E = slow ? Es : Ef;
    const double *sd = slow ? ss : sf;
    int nfree = 0; double sfree = 0;
    for (int k = lo; k < hi; k++) if (!cl[k]) { nfree++; sfree += E[k]; }
    if (nfree < (W / 2 > 1 ? W / 2 : 1)) continue;
    double zr = sfree / nfree / sigma_any(d, sd, nfree);
    if (zr < (slow ? c->z_det_slow : c->z_det)) continue;
    double zsl = 0;
    for (int k = lo; k < hi; k++) zsl += Es[k];
    zsl = zsl / W / sigma_any(d, ss, W);
    const double *Eext, *sdx; int sm;
    if (zsl > c->z_edge * 2 || slow) { Eext = Es; sdx = ss; sm = W <= 4 ? 1 : 2; }
    else { Eext = Ef; sdx = sf; sm = W / 4 > 1 ? W / 4 : 1; }
    int ci, eslow = (Eext == Es);
    for (ci = 0; ci < ncache; ci++) if (cache[ci].slow == eslow && cache[ci].sm == sm) break;
    if (ci == ncache) {
      if (ncache == DET2_NCACHE) ci = ncache - 1;      /* (never with the default widths) */
      else ncache++;
      cache[ci].slow = eslow; cache[ci].sm = sm;
      cache[ci].Z = d->cbuf + (size_t)(2 * ci) * N; cache[ci].T = d->cbuf + (size_t)(2 * ci + 1) * N;
      for (int k = 0; k < N; k++) src[k] = cl[k] ? 0.0 : Eext[k];
      movmean(src, N, sm, cache[ci].Z);
      for (int k = 0; k < N; k++) if (!isfinite(cache[ci].Z[k])) cache[ci].Z[k] = 0.0;
      movmean(Eext, N, sm, cache[ci].T);
      for (int k = 0; k < N; k++) if (!isfinite(cache[ci].T[k])) cache[ci].T[k] = 0.0;
    }
    const double *Esm = cache[ci].Z;
    int p = -1; double best = -1e300;
    for (int k = lo; k < hi; k++) { double v = cl[k] ? -1e9 : Esm[k]; if (v > best) { best = v; p = k; } }
    double pk = Esm[p];
    if (pk <= 0) continue;
    if (is_tail(d, cache[ci].T, cl, p)) continue;   /* tail test on the profile without the claims zeroed */
    double sgs = sigma_any(d, sdx, sm);
    double thr = c->z_edge * sgs;
    if (c->edge_frac * pk > thr) thr = c->edge_frac * pk;
    int s = extend(d, Esm, cl, p, -1, thr, sgs);
    int e = extend(d, Esm, cl, p, +1, thr, sgs);
    double exs = 0, fcn = 0, Em = 0, Exm = 0;
    for (int k = s; k <= e; k++) { double x = Eext[k] > 0 ? Eext[k] : 0; exs += x; fcn += k * x; Em += E[k]; Exm += Eext[k]; }
    if (exs <= 0) continue;
    int wsum = e - s + 1;
    Em /= wsum; Exm /= wsum;
    double fc = fcn / exs;
    double zt = Em / sigma_any(d, sd, wsum);
    double x0, x1;
    if (wsum > 1) {
      static double cs[DET2_MAXCAND * 8], ix[DET2_MAXCAND * 8];
      int n = wsum < (int)(sizeof(cs) / sizeof(cs[0])) ? wsum : (int)(sizeof(cs) / sizeof(cs[0]));
      double acc = 0;
      for (int k = 0; k < n; k++) { double x = Eext[s + k] > 0 ? Eext[s + k] : 0; acc += x; cs[k] = acc / exs; ix[k] = s + k + 0.5; }
      x0 = interp_xy(0.005, cs, ix, n);
      x1 = interp_xy(0.995, cs, ix, n);
    } else { x0 = s - 0.5; x1 = e + 0.5; }
    double w99 = x1 - x0 > 1.0 ? x1 - x0 : 1.0;
    double bw = 1.0;
    if (w99 > 1) { double q = w99 * w99 - d->smear * d->smear; bw = sqrt(q > 0.25 ? q : 0.25); }
    det2_cand_t *cd = &cands[nc++];
    memset(cd, 0, sizeof(*cd));
    cd->s = s; cd->e = e; cd->fc = fc; cd->bw = bw > 0.5 ? bw : 0.5;
    cd->z = zt > zr ? zt : zr;
    cd->snr_db = 10 * log10(Exm > 1e-6 ? Exm : 1e-6);
    cd->slow = slow; cd->parent = -1; cd->p_ex = Em;
    for (int k = s; k <= e; k++) cl[k] = 1;
    /* claims changed: refresh the cached zeroed profiles around [s, e] */
    for (int q = 0; q < ncache; q++) {
      int smq = cache[q].sm, h = (smq - 1) / 2;
      const double *Eq = cache[q].slow ? Es : Ef;
      int j0 = s - smq + 1 + h, j1 = e + h;
      if (j0 < 0) j0 = 0;
      if (j1 > N - 1) j1 = N - 1;
      for (int j = j0; j <= j1; j++) {
        int a = j - h;
        if (a < 0 || a + smq > N) { cache[q].Z[j] = 0.0; continue; }
        double acc = 0;
        for (int k = a; k < a + smq; k++) acc += cl[k] ? 0.0 : Eq[k];
        cache[q].Z[j] = acc / smq;
      }
    }
  }
  return nc;
}

/* ------------------------------------------------------------------ narrow in wide */
static int narrow(det2_t *d, det2_cand_t *cands, int nc, int maxc) {
  const det2_cfg_t *c = &d->c;
  int N = d->N, nout = nc;
  double sig1 = sigma_any(d, d->sig_s, 1), lim = pow(10.0, c->narrow_db / 10);
  double *L = d->L;
#define LOGR(j) do { double x_ = d->Rs[j] - 1.0; if (x_ < -0.5) x_ = -0.5; L[j] = log1p(x_); } while (0)
  LOGR(0); LOGR(N - 1);
  for (int ci = 0; ci < nc; ci++) {
    det2_cand_t *cd = &cands[ci];
    if (cd->parent != -1 || cd->e - cd->s + 1 < 5) continue;
    int s = cd->s, e = cd->e, half = (e - s) / 2 + 1;
    for (int j = (s - 3 > 0 ? s - 3 : 0); j <= (e + 3 < N - 1 ? e + 3 : N - 1); j++) LOGR(j);
    double best = 1e18, cbest = cd->fc;
    static double dif[4096];
    for (int st = 0; st <= 12; st++) {
      double cc = cd->fc - 1.5 + 0.25 * st;
      int n = 0;
      for (int ds = 1; ds <= half && n < 4096; ds++) {
        double xp = cc + ds, xm = cc - ds;
        if (xp <= e + 1 && xm >= s - 1 && xp < N - 1 && xm > 0)
          dif[n++] = fabs(interp_grid(L, N, xp, L[0], L[N - 1]) - interp_grid(L, N, xm, L[0], L[N - 1]));
      }
      if (n < 3) continue;
      double v = 0, m1 = -1, m2 = -1;              /* mean without the two largest */
      for (int k = 0; k < n; k++) {
        v += dif[k];
        if (dif[k] > m1) { m2 = m1; m1 = dif[k]; } else if (dif[k] > m2) m2 = dif[k];
      }
      if (n > 4) { v -= m1 + m2; n -= 2; }
      v /= n;
      if (v < best) { best = v; cbest = cc; }
    }
    int len = e - s + 1;
    static double Em[4096], zz[4096]; static uint8_t fl[4096];
    if (len > 4096) continue;
    for (int k = 0; k < len; k++) {
      int jj = s + k;
      Em[k] = interp_grid(d->Rs, N, 2 * cbest - jj, 1.0, 1.0);
      double ratio = d->Rs[jj] / (Em[k] > 1e-3 ? Em[k] : 1e-3);
      zz[k] = (d->Rs[jj] - Em[k]) / (sig1 * sqrt(2.0));
      fl[k] = ratio > lim && zz[k] > c->z_narrow && fabs(jj - cbest) >= 1.5;
    }
    for (int j = 0; j < len;) {
      if (!fl[j]) { j++; continue; }
      int k = j;
      while (k + 1 < len && fl[k + 1]) k++;
      if (k - j + 1 <= c->narrow_max_bins + 2 && nout < maxc) {
        double ws = 0, fcn = 0, zmax = -1e300;
        for (int q = j; q <= k; q++) {
          double w = d->Rs[s + q] - Em[q];
          ws += w; fcn += (s + q) * w;
          if (zz[q] > zmax) zmax = zz[q];
        }
        det2_cand_t *o = &cands[nout++];
        memset(o, 0, sizeof(*o));
        o->s = s + j; o->e = s + k; o->fc = fcn / ws;
        double wq = (k - j + 1) * (double)(k - j + 1) - (d->smear * 0.6) * (d->smear * 0.6);
        o->bw = sqrt(wq > 0.25 ? wq : 0.25);
        if (o->bw < 0.5) o->bw = 0.5;
        o->z = zmax;
        double wm = ws / (k - j + 1);
        o->snr_db = 10 * log10(wm > 1e-6 ? wm : 1e-6);
        o->slow = 1; o->parent = -2; o->p_ex = wm;
      }
      j = k + 1;
    }
  }
  return nout;
}

/* ------------------------------------------------------------------ tracking */
static void extent(const det2_track_t *t, int *lo, int *hi) {
  *lo = (int)floor(t->fc - t->ext + 0.5);
  *hi = (int)ceil(t->fc + t->ext - 0.5);
}

static void track_stat(const det2_t *d, const det2_track_t *t, double *z, double *pex) {
  int s = t->lo_bin < 0 ? 0 : t->lo_bin, e = t->hi_bin > d->N - 1 ? d->N - 1 : t->hi_bin;
  int W = e - s + 1;
  double a = 0, b = 0;
  for (int k = s; k <= e; k++) { a += d->R[k]; b += d->Rs[k]; }
  a /= W; b /= W;
  double zf = (a - 1) / sigma_any(d, d->sig_f, W), zs = (b - 1) / sigma_any(d, d->sig_s, W);
  *z = zf > zs ? zf : zs;
  *pex = a - 1;
}

static int is_streaming_state(int s) { return s == T_CONF || s == T_ACTIVE || s == T_HOLD; }

static void push_hit(det2_track_t *t, int h) {
  if (t->nh == 16) { memmove(t->hits, t->hits + 1, 15); t->nh = 15; }
  t->hits[t->nh++] = (uint8_t)h;
}

static void push_hist(det2_track_t *t, double pex) {
  if (t->np_ == DET2_HIST) {
    memmove(t->phist, t->phist + 1, (DET2_HIST - 1) * sizeof(float));
    memmove(t->fhist, t->fhist + 1, (DET2_HIST - 1) * sizeof(float));
    memmove(t->bhist, t->bhist + 1, (DET2_HIST - 1) * sizeof(float));
    t->np_--;
  }
  t->phist[t->np_] = (float)pex; t->fhist[t->np_] = (float)t->fc; t->bhist[t->np_] = (float)t->bw;
  t->np_++; t->nfb = t->np_;
}

static void transition(det2_t *d, det2_track_t *t) {
  const det2_cfg_t *c = &d->c;
  int n = t->nh;
  if (t->state == T_CAND) {
    int ok;
    if (t->parent >= 0) {
      int need = c->child_confirm, s = 0;
      for (int k = n - need < 0 ? 0 : n - need; k < n; k++) s += t->hits[k];
      ok = s >= need;
    } else {
      int s = 0;
      for (int k = n - c->confirm_n < 0 ? 0 : n - c->confirm_n; k < n; k++) s += t->hits[k];
      ok = s >= c->confirm_m || t->z >= c->z_fast_confirm;
    }
    if (ok) t->state = T_CONF;
    else if (n >= 2 && !t->hits[n - 1] && !t->hits[n - 2]) t->state = T_DROP;
    else if (t->parent >= 0 && !t->hits[n - 1]) t->state = T_DROP;
  } else if (is_streaming_state(t->state)) {
    if (t->hits[n - 1]) {
      if (t->state == T_HOLD || (t->state == T_CONF && t->nhits >= c->active_hits)) t->state = T_ACTIVE;
    } else {
      if (d->t - t->t_last >= c->hold_s) t->state = T_EXPIRED;
      else if (t->state != T_HOLD) t->state = T_HOLD;
    }
  } else if (t->state == T_EXPIRED && t->hits[n - 1]) t->state = T_ACTIVE;
  double cf = t->nhits / 10.0;
  if (cf > 1) cf = 1;
  t->conf = cf * ((t->state == T_CONF || t->state == T_ACTIVE) ? 1.0 : 0.5);
}

/* a streaming primary track >= 6 dB stronger whose extraction interval contains fc
   and towards which the (slow) profile of [s, e] keeps rising: [s, e] is its skirt */
static det2_track_t *host_of_tail(det2_t *d, int s, int e, double fc, double snr_db, const det2_track_t *excl) {
  for (int i = 0; i < d->ntr; i++) {
    det2_track_t *h = &d->tr[i];
    if (fabs(h->fc - fc) > 64) continue;          /* extraction intervals are < 64 bins */
    if (h == excl || h->parent >= 0 || !is_streaming_state(h->state) || h->snr_db < snr_db + 6.0) continue;
    int hs, he;
    det2_extract(d, h, &hs, &he);
    if (!(hs <= fc && fc <= he)) continue;
    if (s < 0) s = 0;
    if (e > d->N - 1) e = d->N - 1;
    if (e < s) continue;
    int pk = s;
    for (int k = s + 1; k <= e; k++) if (d->Rs[k] > d->Rs[pk]) pk = k;
    if ((h->fc < s && pk <= s + 1) || (h->fc > e && pk >= e - 1) || (s <= h->fc && h->fc <= e)) return h;
  }
  return NULL;
}

typedef struct { double d; int i, tid; } pair_t;
static int cmp_pair(const void *a, const void *b) {
  const pair_t *x = a, *y = b;
  if (x->d != y->d) return (x->d > y->d) - (x->d < y->d);
  if (x->i != y->i) return x->i - y->i;
  return x->tid - y->tid;
}

static det2_track_t *by_id(det2_t *d, int id) {
  for (int i = 0; i < d->ntr; i++) if (d->tr[i].id == id) return &d->tr[i];
  return NULL;
}

static void track(det2_t *d, det2_cand_t *cands, int nc) {
  const det2_cfg_t *c = &d->c;
  int live[DET2_MAXTRACK], nl = 0;
  for (int i = 0; i < d->ntr; i++) if (d->tr[i].state != T_EXPIRED) live[nl++] = i;
  /* compact copy of the fields the O(cands x tracks) loops need (cache friendly) */
  static struct { double fc, bw; int lo, hi, par, strm; } lt[DET2_MAXTRACK];
  for (int l = 0; l < nl; l++) {
    det2_track_t *t = &d->tr[live[l]];
    lt[l].fc = t->fc; lt[l].bw = t->bw; lt[l].lo = t->lo_bin; lt[l].hi = t->hi_bin;
    lt[l].par = t->parent; lt[l].strm = is_streaming_state(t->state);
  }
  static pair_t pairs[DET2_MAXCAND * 64];
  int np_ = 0;
  for (int i = 0; i < nc; i++)
    for (int l = 0; l < nl; l++) {
      double df = fabs(cands[i].fc - lt[l].fc);
      if (df >= 1.0 && df > 0.5 * cands[i].bw && df > 0.5 * lt[l].bw) continue;   /* dd >= 1 */
      if ((cands[i].parent == -2) != (lt[l].par >= 0)) continue;
      double mb = cands[i].bw > lt[l].bw ? cands[i].bw : lt[l].bw;
      double dd = df / (0.5 * mb > 1.0 ? 0.5 * mb : 1.0);
      int ov = (cands[i].e < lt[l].hi ? cands[i].e : lt[l].hi) - (cands[i].s > lt[l].lo ? cands[i].s : lt[l].lo);
      if (dd < 1.0 && ov >= 0 && np_ < (int)(sizeof(pairs) / sizeof(pairs[0]))) {
        pairs[np_].d = dd; pairs[np_].i = i; pairs[np_].tid = l; np_++;   /* tid: live index (same order as ids) */
      }
    }
  qsort(pairs, np_, sizeof(pair_t), cmp_pair);
  static int used[DET2_MAXCAND];
  int match_c[DET2_MAXTRACK], match_m[DET2_MAXTRACK];      /* per live index: cand, merged flag */
  for (int i = 0; i < nc; i++) used[i] = 0;
  for (int l = 0; l < nl; l++) { match_c[l] = -1; match_m[l] = 0; }
  for (int k = 0; k < np_; k++) {
    int i = pairs[k].i, l = pairs[k].tid;
    if (used[i]) continue;
    if (match_c[l] >= 0) continue;
    used[i] = 1; match_c[l] = i;
  }
  for (int i = 0; i < nc; i++) {
    if (cands[i].parent == -2) continue;
    for (int l = 0; l < nl; l++) {
      if (lt[l].fc < cands[i].s || lt[l].fc > cands[i].e) continue;
      if (match_c[l] < 0 && lt[l].par < 0 && lt[l].strm) {
        match_c[l] = i; match_m[l] = 1;   /* the candidate itself stays available */
      }
    }
  }
  const double tau = 3.0;
  for (int l = 0; l < nl; l++) {
    det2_track_t *t = &d->tr[live[l]];
    int hit = 0;
    if (match_c[l] >= 0) {
      det2_cand_t *cd = &cands[match_c[l]];
      hit = 1;
      if (!match_m[l]) {
        t->fc += 0.3 * (cd->fc - t->fc);
        double dec = 1.0 - d->dt / tau;
        t->bw = cd->bw > t->bw * dec ? cd->bw : t->bw * dec;
        if (t->bw > d->cap_bw) t->bw = d->cap_bw;
        double ex = (cd->fc - cd->s > cd->e - cd->fc ? cd->fc - cd->s : cd->e - cd->fc) + 0.5;
        t->ext = ex > t->ext * dec ? ex : t->ext * dec;
        if (t->ext > d->cap_ext) t->ext = d->cap_ext;
        t->snr_db = cd->snr_db; t->z = cd->z; t->merged = 0;
        int others = 0;
        for (int q = 0; q < nl; q++) {
          if (q == l) continue;
          double ofc = q < l ? d->tr[live[q]].fc : lt[q].fc;     /* earlier ones were updated */
          if (ofc < cd->s || ofc > cd->e) continue;
          det2_track_t *o = &d->tr[live[q]];
          if (o->parent < 0 && is_streaming_state(o->state)) { others = 1; break; }
        }
        if (!others) t->sep_hits++;
      } else t->merged = 1;
      extent(t, &t->lo_bin, &t->hi_bin);
    } else {
      double zk, pe;
      track_stat(d, t, &zk, &pe);
      t->z = zk;
      if (zk > c->z_keep && t->state != T_CAND) hit = 1;
    }
    double zz, pex;
    track_stat(d, t, &zz, &pex);
    push_hist(t, pex);
    push_hit(t, hit);
    if (hit) { t->nhits++; t->t_last = d->t; }
    transition(d, t);
  }
  for (int l = 0; l < nl; l++) {               /* refresh after the updates above */
    det2_track_t *t = &d->tr[live[l]];
    lt[l].fc = t->fc; lt[l].bw = t->bw; lt[l].lo = t->lo_bin; lt[l].hi = t->hi_bin;
    lt[l].par = t->parent; lt[l].strm = is_streaming_state(t->state);
  }
  /* new tracks */
  for (int i = 0; i < nc; i++) {
    det2_cand_t *cd = &cands[i];
    if (used[i] || cd->merged_into) continue;
    int inside = 0;
    for (int l = 0; l < nl && !inside; l++) {
      if (lt[l].lo > cd->fc || cd->fc > lt[l].hi) continue;
      if (lt[l].par < 0) inside = 1;
    }
    if (cd->parent != -2 && inside) continue;
    if (cd->parent != -2 && host_of_tail(d, cd->s, cd->e, cd->fc, cd->snr_db, NULL)) continue;
    if (d->ntr >= DET2_MAXTRACK) break;
    det2_track_t nt;
    memset(&nt, 0, sizeof(nt));
    nt.id = d->next_id; nt.fc = cd->fc; nt.bw = cd->bw; nt.snr_db = cd->snr_db; nt.z = cd->z;
    nt.t_first = nt.t_last = d->t; nt.hits[0] = 1; nt.nh = 1; nt.nhits = 1;
    nt.parent = -1; nt.independent = 1; nt.state = T_CAND;
    nt.ext = (cd->fc - cd->s > cd->e - cd->fc ? cd->fc - cd->s : cd->e - cd->fc) + 0.5;
    if (nt.ext > d->cap_ext) nt.ext = d->cap_ext;
    if (nt.bw > d->cap_bw) nt.bw = d->cap_bw;
    if (cd->parent == -2) {
      int pid = -1;
      for (int l = 0; l < nl; l++) {
        det2_track_t *p = &d->tr[live[l]];
        if (p->parent < 0 && p->lo_bin <= cd->fc && cd->fc <= p->hi_bin) { pid = p->id; break; }
      }
      if (pid < 0) continue;
      nt.parent = pid; nt.independent = 0;
    }
    extent(&nt, &nt.lo_bin, &nt.hi_bin);
    d->next_id++;
    d->tr[d->ntr] = nt;
    transition(d, &d->tr[d->ntr]);
    {
      det2_track_t *t = &d->tr[d->ntr];
      lt[nl].fc = t->fc; lt[nl].bw = t->bw; lt[nl].lo = t->lo_bin; lt[nl].hi = t->hi_bin;
      lt[nl].par = t->parent; lt[nl].strm = is_streaming_state(t->state);
    }
    live[nl++] = d->ntr;
    d->ntr++;
  }
  /* absorb */
  int prim[DET2_MAXTRACK], npr = 0;
  for (int i = 0; i < d->ntr; i++)
    if (d->tr[i].parent < 0 && d->tr[i].state != T_EXPIRED && d->tr[i].state != T_DROP) prim[npr++] = i;
  static double pfc[DET2_MAXTRACK];
  for (int a = 0; a < npr; a++) pfc[a] = d->tr[prim[a]].fc;
  for (int a = 0; a < npr; a++) {
    det2_track_t *t = &d->tr[prim[a]];
    det2_track_t *h = NULL;
    for (int b = 0; b < npr; b++) {
      det2_track_t *o = &d->tr[prim[b]];
      if (pfc[b] < t->fc - 64 || pfc[b] > t->fc + 64) continue;   /* (extents are < 64 bins) */
      if (o != t && o->t_first <= t->t_first && is_streaming_state(o->state) &&
          o->lo_bin <= t->fc && t->fc <= o->hi_bin && o->state != T_DROP) { h = o; break; }
    }
    if (!h) {
      det2_track_t *ht = host_of_tail(d, t->lo_bin, t->hi_bin, t->fc, t->snr_db, t);
      if (ht && ht->t_first <= t->t_first) { h = ht; t->sep_hits = 0; }
    }
    if (h && t->sep_hits < 3) {
      t->absorb++;
      if (t->absorb >= 2) {
        int lo = h->lo_bin < t->lo_bin ? h->lo_bin : t->lo_bin, hi = h->hi_bin > t->hi_bin ? h->hi_bin : t->hi_bin;
        double e1 = h->fc - lo + 0.5, e2 = hi - h->fc + 0.5;
        if (e1 > h->ext) h->ext = e1;
        if (e2 > h->ext) h->ext = e2;
        if (h->ext > d->cap_ext) h->ext = d->cap_ext;
        if ((hi - lo + 1) - d->smear > h->bw) h->bw = (hi - lo + 1) - d->smear;
        if (h->bw > d->cap_bw) h->bw = d->cap_bw;
        extent(h, &h->lo_bin, &h->hi_bin);
        t->state = T_DROP;
      }
    } else t->absorb = 0;
  }
  /* drop dead tracks (keep order) */
  int w = 0;
  for (int i = 0; i < d->ntr; i++) {
    det2_track_t *t = &d->tr[i];
    if ((t->state == T_EXPIRED && d->t - t->t_last > 5.0) || t->state == T_DROP) continue;
    if (w != i) d->tr[w] = *t;
    w++;
  }
  d->ntr = w;
  /* child validation */
  for (int i = 0; i < d->ntr; i++) {
    det2_track_t *t = &d->tr[i];
    if (t->parent < 0) continue;
    det2_track_t *p = by_id(d, t->parent);
    if (!p || p->state == T_EXPIRED) { t->independent = 1; continue; }
    if (t->np_ >= 8) {
      int na = t->np_ < 16 ? t->np_ : 16, nb = p->np_ < 16 ? p->np_ : 16, n = na < nb ? na : nb;
      const float *a = t->phist + t->np_ - n, *b = p->phist + p->np_ - n;
      double ma = 0, mb = 0;
      for (int k = 0; k < n; k++) { ma += a[k]; mb += b[k]; }
      ma /= n; mb /= n;
      double saa = 0, sbb = 0, sab = 0;
      for (int k = 0; k < n; k++) { saa += (a[k] - ma) * (a[k] - ma); sbb += (b[k] - mb) * (b[k] - mb); sab += (a[k] - ma) * (b[k] - mb); }
      if (sqrt(saa / n) > 1e-9 && sqrt(sbb / n) > 1e-9) {
        double r = sab / sqrt(saa * sbb);
        if (r < c->child_corr_max) t->independent = 1;
      }
    }
  }
}

/* ------------------------------------------------------------------ allocation */
void det2_extract(const det2_t *d, const det2_track_t *t, int *ps, int *pe) {
  const det2_cfg_t *c = &d->c;
  double det_hz = t->bw * d->binw;
  double ex = det_hz * c->bw_factor;
  if (det_hz < 12000 && c->narrow_extract_hz > ex) ex = c->narrow_extract_hz;
  ex += 2 * c->guard_hz;
  double half = ex / 2 / d->binw;
  int s = (int)floor(t->fc - half), e = (int)ceil(t->fc + half);
  if (s < 2) s = 2;
  if (e > d->N - 3) e = d->N - 3;
  *ps = s; *pe = e;
}

static int cmp_run(const void *a, const void *b) {
  const det2_run_t *x = a, *y = b;
  if (x->s != y->s) return x->s - y->s;
  return x->e - y->e;
}

static void allocate(det2_t *d) {
  const det2_cfg_t *c = &d->c;
  int N = d->N;
  det2_run_t iv[DET2_MAXTRACK]; int ni = 0;
  d->nslots = 0;
  for (int i = 0; i < d->ntr; i++) {
    det2_track_t *t = &d->tr[i];
    if (!det2_streaming(d, t)) continue;
    int s, e;
    det2_extract(d, t, &s, &e);
    iv[ni].s = s; iv[ni].e = e; ni++;
    if (t->parent >= 0 && !t->independent) continue;
    int need = e - s + 1 + 2, M = 8;
    while (M < need) M *= 2;
    det2_slot_t *sl = &d->slots[d->nslots++];
    sl->id = t->id; sl->c = (int)rint(t->fc); sl->M = M; sl->s = s; sl->e = e; sl->fc = t->fc; sl->state = t->state;
  }
  qsort(iv, ni, sizeof(det2_run_t), cmp_run);
  d->nruns = 0;
  for (int i = 0; i < ni; i++) {
    if (d->nruns && iv[i].s <= d->runs[d->nruns - 1].e + 1 + c->merge_gap_bins) {
      if (iv[i].e > d->runs[d->nruns - 1].e) d->runs[d->nruns - 1].e = iv[i].e;
    } else d->runs[d->nruns++] = iv[i];
  }
}

/* ------------------------------------------------------------------ main */
void det2_update(det2_t *d, const double *P, double t) {
  const det2_cfg_t *c = &d->c;
  int N = d->N;
  d->t = t;
  uint8_t *mask = d->mask;
  memcpy(mask, d->fixed_mask, N);
  for (int i = 0; i < d->ntr; i++) {
    det2_track_t *tr = &d->tr[i];
    if (tr->state == T_EXPIRED) continue;
    for (int k = tr->lo_bin - 1; k <= tr->hi_bin + 1; k++) if (k >= 0 && k < N) mask[k] = 1;
  }
  double sig1 = 1.0 / 8.0;
  if (d->have_sig) {
    sig1 = d->Wb[0] == 1 ? d->sig_f[0] : d->sig_f[0] * sqrt((double)d->Wb[0]);
  }
  static double *Bl, *Bnew, *Rs_old;
  static int cap;
  if (cap < N) { free(Bl); free(Bnew); free(Rs_old); Bl = malloc(N * 8); Bnew = malloc(N * 8); Rs_old = malloc(N * 8); cap = N; }
  memcpy(Rs_old, d->Rs, N * sizeof(double));
  double sf[DET2_MAXW], ss[DET2_MAXW];
  /* strong candidates of the previous update are masked too (one pass per update) */
  for (int i = 0; i < d->nprev; i++)
    for (int k = d->prev[i].s - 1; k <= d->prev[i].e + 1; k++) if (k >= 0 && k < N) mask[k] = 1;
  baseline(d, P, mask, sig1, Bl);
  for (int j = 0; j < N; j++) {
    Bnew[j] = d->have_B ? d->Blog[j] + c->base_alpha * (Bl[j] - d->Blog[j]) : Bl[j];
    d->R[j] = P[j] * exp(-Bnew[j] * 0.23025850929940458);      /* P / 10^(B/10) */
    d->Rs[j] = d->have_Rs ? Rs_old[j] + c->slow_alpha * (d->R[j] - Rs_old[j]) : d->R[j];
  }
  prefixes(d, mask);
  sigmas(d, mask, sf, ss);
  {
    double s1 = d->Wb[0] == 1 ? sigma_any(d, ss, 1) : ss[0] * sqrt((double)d->Wb[0]);
    for (int j = 0; j < N; j++) d->age_now[j] = ((d->Rs[j] - 1.0) / s1 > c->z_edge) ? d->age[j] + 1 : 0;
  }
  d->ncand = clean(d, sf, ss, d->cand, DET2_MAXCAND);
  d->nprev = 0;
  for (int i = 0; i < d->ncand && d->nprev < DET2_MAXTRACK; i++)
    if (d->cand[i].z > 2 * c->z_det) { d->prev[d->nprev].s = d->cand[i].s; d->prev[d->nprev].e = d->cand[i].e; d->nprev++; }
  memcpy(d->Blog, Bnew, N * sizeof(double));
  memcpy(d->age, d->age_now, N * sizeof(int));
  memcpy(d->sig_f, sf, sizeof(sf)); memcpy(d->sig_s, ss, sizeof(ss));
  d->have_B = d->have_Rs = d->have_sig = 1;
  d->ncand = narrow(d, d->cand, d->ncand, DET2_MAXCAND);
  track(d, d->cand, d->ncand);
  allocate(d);
}
