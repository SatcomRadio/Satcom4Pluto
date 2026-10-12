/*
 * SPDX-License-Identifier: MIT
 *
 * sgchd -- satellite-downlink channel daemon for the Pluto channelizer
 * (FPGA core sgc_top at 0x43C00000, bitstream "SGCH" v1.0 / v1.1).
 *
 * Every spectrum update it reads the averaged power spectrum from the FPGA and
 * runs the detector (det2.c: multiresolution adaptive detection + tracking,
 * reference src/reference/satref/det_v2.py). --all / --default switch detection
 * off for diagnostics.
 * then applies the rate cap and writes the per-bin gains and the run list back. Only the selected bins leave the Pluto (RX DMA, frames
 * described in sgc_core.v); the PC bridge rebuilds the wideband stream.
 *
 * TCP control on port 30470 (one command per line):
 *   SET <options>      same options as the command line (replace all)
 *   GET                current options
 *   STATUS             key=value status
 *   CHANNELS           open windows: centre MHz, width kHz, peak SNR dB, state
 *   TRACKS             v2 tracks: id state fc_MHz bw_kHz snr_dB z conf age_s last_s parent indep
 *                      ext_lo_MHz ext_hi_MHz (extraction interval) streaming peak_dB
 *   SUBSCRIBE          push "SPEC" blocks (noise-fill spectrum) every update
 *   MODE 0|1           RX DMA: 0 = normal samples, 1 = channel frames
 *   QUIT               close the connection
 * TCP port 30471: raw stream of the RX DMA words (no request/response round
 * trips like iiod, so more of the USB/Ethernet bandwidth is usable). One
 * client at a time; while it is connected the DMA carries channel frames.
 * The last SET is stored in /mnt/jffs2/sgchd.conf and used at start-up.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <poll.h>
#include <sys/ioctl.h>

#define SGC_BASE      0x43C00000u
#define SGC_SIZE      0x10000u
#define FOR_MAJOR     1
#define FOR_MINOR     1       /* 1.1: selectable detection kernel (PARAM[25:24]); 1.0 works with sin^5 only */
#include "det2.h"
#define CONF_FILE     "/mnt/jffs2/sgchd.conf"
#define DEF_PORT      30470
#define STREAM_PORT_OFS 1

/* registers */
#define R_ID      0x0000
#define R_VER     0x0004
#define R_CONFIG  0x0008
#define R_CORE_HZ 0x000C
#define R_CTRL    0x0010
#define R_PARAM   0x0014
#define R_STATUS  0x0018
#define R_SPEC    0x001C
#define R_FRAMES  0x0020
#define R_DROPS   0x0024
#define R_SAT     0x0028
#define R_WORDS   0x002C
#define R_BANK    0x0030
#define R_NRUNS   0x0034
#define R_SAMPLES 0x0038
#define R_INOVF   0x003C
#define R_RATE    0x0040
#define R_GAIN    0x4000
#define R_RUNS    0x6000
#define R_SPECRAM 0x8000

static volatile uint32_t *regs;
static int L, N, H, RUNS_MAX;
static double core_hz = 100e6;

static inline uint32_t rd(uint32_t a) { return regs[a >> 2]; }
static inline void wr(uint32_t a, uint32_t v) { regs[a >> 2] = v; }

/* ------------------------------------------------------------------ */
/* options */
#define MAX_F 32
#define MAX_PASS 16
typedef struct {
  int nf; double flo[MAX_F], fhi[MAX_F];   /* MHz */
  int hold_s_set; double hold_s;
  int has_max_snr; double max_snr_db;
  int dflt;                                /* --default: pass nothing, DMA normal */
  double max_rate;                         /* MS/s of output words */
  double min_rate;                         /* keep-alive output (zero bins), MS/s (fixed) */
  double dc_notch_khz;                     /* LO leakage: no detection within +-this */
  int osh, psh, avg;                       /* output / power shift, log2 averaging: -1 = auto (fixed) */
  int mode;                                /* 1: DMA carries channel frames */
  int all;                                 /* --all: pass the whole -f ranges (no detection) */
  int kernel;                              /* detection kernel 0 sin^5, 1 sin^3, 2 sin^1 */
  det2_cfg_t d2;                           /* v2 parameters (spur_hz holds absolute MHz here) */
  int npass; double plo[MAX_PASS], phi[MAX_PASS];   /* --pass: always-streamed ranges, MHz */
} opts_t;

static void opts_default(opts_t *o) {
  memset(o, 0, sizeof(*o));
  o->hold_s = 2.0;
  o->max_rate = 6.5;   /* measured USB limit ~6.8 MS/s */ o->min_rate = 0.25; o->dc_notch_khz = 5.0; o->osh = -1; o->psh = -1; o->avg = -1; o->mode = 1;
  o->kernel = 1;
  det2_cfg_default(&o->d2);
}

static const char *USAGE =
"sgchd -- Pluto satellite-downlink channelizer daemon\n"
"Detection is always on: only the downlinks it finds (plus --pass ranges) are streamed.\n"
"  -f LO:HI            search this range in MHz (repeat, max 32; default: whole band)\n"
"  --pass LO:HI        always stream this range in MHz, detected or not (repeat, max 16)\n"
"  --max-snr-db DB     block signals peaking more than DB above the noise floor (jammers);\n"
"                      released 3 dB below the limit (default: off)\n"
"  --max-bw-khz K      widest single detection, 5..2000 kHz (default 150)\n"
"  --spur MHZ          ignore a known birdie at this frequency (repeat, max 32)\n"
"  --dc-notch-khz K    ignore detections within K kHz of the LO (default 5, 0 = off)\n"
"  --max-rate MSPS     output budget in MS/s; weakest detections dropped first\n"
"                      (default 6.5; the Pluto's USB link carries about 6.8 at most)\n"
"  --kernel sin1|sin3|sin5  FPGA detection smoothing (default sin3)\n"
"sensitivity / tracking:\n"
"  --z-det Z           detection threshold, fast spectrum, sigma (default 5.5)\n"
"  --z-slow Z          detection threshold, slow averaged spectrum, sigma (default 6.0)\n"
"  --z-keep Z          a found signal is kept while above this, sigma (default 3.0)\n"
"  --confirm M/N       report a signal after M hits in N updates (default 2/3, N max 16)\n"
"  --hold-s S          keep streaming S seconds after the signal disappears (default 2)\n"
"  --widths K,K,..     detection widths in kHz (default 5,7,12.5,25,40,60; max 8)\n"
"  --stream-candidates stream signals before they are confirmed (faster, more false bins)\n"
"extraction (how much around each signal is sent):\n"
"  --guard-khz K       extra each side (default 4.5)\n"
"  --bw-factor F       sent width = detected width x F + 2 guards (default 1.25)\n"
"  --narrow-extract-khz K  minimum sent width for narrow signals (default 12.5)\n"
"diagnostics:\n"
"  --all               no detection: stream the -f ranges completely (minus --max-snr-db)\n"
"  --default           stream nothing; the RX DMA carries normal samples again\n"
"start-up only (command line, not SET):\n"
"  --port P            TCP control port (default 30470; stream port = P + 1)\n"
"  --no-conf           ignore " CONF_FILE "\n"
"  --rx-delay 0xCD|off AD936x RX interface delay held while running (default 0x06)\n";

static int parse_opts(opts_t *o, int argc, char **argv, char *err, size_t errlen) {
  opts_default(o);
  for (int i = 0; i < argc; i++) {
    const char *a = argv[i];
    const char *v = (i + 1 < argc) ? argv[i + 1] : NULL;
#define NEEDV() do { if (!v) { snprintf(err, errlen, "%s needs a value", a); return -1; } i++; } while (0)
    if (!strcmp(a, "-f") || !strcmp(a, "--filter")) {
      NEEDV(); double lo, hi;
      if (sscanf(v, "%lf:%lf", &lo, &hi) != 2) { snprintf(err, errlen, "bad -f %s (LO:HI in MHz)", v); return -1; }
      if (o->nf >= MAX_F) { snprintf(err, errlen, "at most %d -f ranges", MAX_F); return -1; }
      if (lo > hi) { double t = lo; lo = hi; hi = t; }
      o->flo[o->nf] = lo; o->fhi[o->nf] = hi; o->nf++;
    }
    else if (!strcmp(a, "--pass")) {
      NEEDV(); double lo, hi;
      if (sscanf(v, "%lf:%lf", &lo, &hi) != 2 || lo == hi) { snprintf(err, errlen, "bad --pass %s (LO:HI in MHz)", v); return -1; }
      if (o->npass >= MAX_PASS) { snprintf(err, errlen, "at most %d --pass ranges", MAX_PASS); return -1; }
      if (lo > hi) { double t = lo; lo = hi; hi = t; }
      o->plo[o->npass] = lo; o->phi[o->npass] = hi; o->npass++;
    }
    else if (!strcmp(a, "--max-snr-db")) { NEEDV(); o->has_max_snr = 1; o->max_snr_db = atof(v); }
    else if (!strcmp(a, "--max-bw-khz")) {
      NEEDV(); double k = atof(v);
      if (k < 5 || k > 2000) { snprintf(err, errlen, "--max-bw-khz must be 5..2000"); return -1; }
      o->d2.max_bw_hz = k * 1e3;
    }
    else if (!strcmp(a, "--spur")) {
      NEEDV();
      if (o->d2.nspur >= DET2_MAXSPUR) { snprintf(err, errlen, "at most %d --spur values", DET2_MAXSPUR); return -1; }
      o->d2.spur_hz[o->d2.nspur++] = atof(v);
    }
    else if (!strcmp(a, "--dc-notch-khz")) { NEEDV(); o->dc_notch_khz = atof(v); }
    else if (!strcmp(a, "--max-rate")) { NEEDV(); o->max_rate = atof(v); }
    else if (!strcmp(a, "--kernel")) { NEEDV(); if (!strcmp(v, "sin5")) o->kernel = 0; else if (!strcmp(v, "sin3")) o->kernel = 1; else if (!strcmp(v, "sin1")) o->kernel = 2; else { snprintf(err, errlen, "--kernel sin1|sin3|sin5"); return -1; } }
    else if (!strcmp(a, "--z-det")) { NEEDV(); o->d2.z_det = atof(v); }
    else if (!strcmp(a, "--z-slow")) { NEEDV(); o->d2.z_det_slow = atof(v); }
    else if (!strcmp(a, "--z-keep")) { NEEDV(); o->d2.z_keep = atof(v); }
    else if (!strcmp(a, "--confirm")) { NEEDV(); if (sscanf(v, "%d/%d", &o->d2.confirm_m, &o->d2.confirm_n) != 2 || o->d2.confirm_m < 1 || o->d2.confirm_n < o->d2.confirm_m || o->d2.confirm_n > 16) { snprintf(err, errlen, "--confirm M/N (1 <= M <= N <= 16)"); return -1; } }
    else if (!strcmp(a, "--hold-s")) { NEEDV(); o->hold_s = atof(v); o->hold_s_set = 1; }
    else if (!strcmp(a, "--widths")) {
      NEEDV(); char tmp[256]; snprintf(tmp, sizeof(tmp), "%s", v); o->d2.nwidths = 0;
      for (char *t = strtok(tmp, ","); t; t = strtok(NULL, ",")) {
        if (o->d2.nwidths >= DET2_MAXW) { snprintf(err, errlen, "at most %d --widths", DET2_MAXW); return -1; }
        o->d2.widths_hz[o->d2.nwidths++] = atof(t) * 1e3;
      }
      if (!o->d2.nwidths) { snprintf(err, errlen, "bad --widths"); return -1; }
    }
    else if (!strcmp(a, "--stream-candidates")) o->d2.stream_candidates = 1;
    else if (!strcmp(a, "--guard-khz")) { NEEDV(); o->d2.guard_hz = atof(v) * 1e3; }
    else if (!strcmp(a, "--bw-factor")) { NEEDV(); o->d2.bw_factor = atof(v); }
    else if (!strcmp(a, "--narrow-extract-khz")) { NEEDV(); o->d2.narrow_extract_hz = atof(v) * 1e3; }
    else if (!strcmp(a, "--all")) o->all = 1;
    else if (!strcmp(a, "--default")) o->dflt = 1;
    else { snprintf(err, errlen, "unknown option %s", a); return -1; }
#undef NEEDV
  }
  if (o->dflt && (o->nf || o->has_max_snr || o->all || o->npass)) {
    snprintf(err, errlen, "--default cannot be combined with other options"); return -1;
  }
  if (o->hold_s < 0 || o->max_rate <= 0 || o->dc_notch_khz < 0 || o->d2.guard_hz < 0 || o->d2.narrow_extract_hz < 0) {
    snprintf(err, errlen, "bad value (--hold-s, --dc-notch-khz, --guard-khz, --narrow-extract-khz must be >= 0, --max-rate > 0)"); return -1;
  }
  if (o->d2.z_det <= 0 || o->d2.z_det_slow <= 0 || o->d2.z_keep <= 0 || o->d2.bw_factor <= 0) {
    snprintf(err, errlen, "bad value (--z-det, --z-slow, --z-keep, --bw-factor must be > 0)"); return -1;
  }
  for (int i = 0; i < o->d2.nwidths; i++)
    if (o->d2.widths_hz[i] <= 0) { snprintf(err, errlen, "bad --widths (kHz values > 0)"); return -1; }
  o->d2.hold_s = o->hold_s;
  o->d2.dc_notch_hz = o->dc_notch_khz * 1e3;
  o->mode = o->dflt ? 0 : 1;
  return 0;
}

static void opts_string(const opts_t *o, char *s, size_t n) {
  size_t k = 0;
#define AP(...) k += snprintf(s + k, k < n ? n - k : 0, __VA_ARGS__)
  s[0] = 0;
  if (o->dflt) { AP("--default"); return; }
  static const char *kn[] = { "sin5", "sin3", "sin1" };
  for (int i = 0; i < o->nf; i++) AP("-f %.6f:%.6f ", o->flo[i], o->fhi[i]);
  for (int i = 0; i < o->npass; i++) AP("--pass %.6f:%.6f ", o->plo[i], o->phi[i]);
  if (o->has_max_snr) AP("--max-snr-db %g ", o->max_snr_db);
  AP("--max-bw-khz %g ", o->d2.max_bw_hz / 1e3);
  for (int i = 0; i < o->d2.nspur; i++) AP("--spur %.6f ", o->d2.spur_hz[i]);
  AP("--dc-notch-khz %g --max-rate %g --kernel %s ", o->dc_notch_khz, o->max_rate, kn[o->kernel]);
  AP("--z-det %g --z-slow %g --z-keep %g --confirm %d/%d --hold-s %g --widths ",
     o->d2.z_det, o->d2.z_det_slow, o->d2.z_keep, o->d2.confirm_m, o->d2.confirm_n, o->hold_s);
  for (int i = 0; i < o->d2.nwidths; i++) AP("%s%g", i ? "," : "", o->d2.widths_hz[i] / 1e3);
  if (o->d2.stream_candidates) AP(" --stream-candidates");
  AP(" --guard-khz %g --bw-factor %g --narrow-extract-khz %g",
     o->d2.guard_hz / 1e3, o->d2.bw_factor, o->d2.narrow_extract_hz / 1e3);
  if (o->all) AP(" --all");
#undef AP
}

static int split_args(char *line, char **av, int max) {
  int n = 0;
  char *p = line;
  while (*p && n < max) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (!*p) break;
    av[n++] = p;
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
    if (*p) *p++ = 0;
  }
  return n;
}

/* ------------------------------------------------------------------ */
/* IIO: sample rate and LO from the ad9361-phy sysfs node */
static char phy_path[256];

static int find_phy(void) {
  DIR *d = opendir("/sys/bus/iio/devices");
  if (!d) return -1;
  struct dirent *e;
  while ((e = readdir(d))) {
    if (strncmp(e->d_name, "iio:device", 10)) continue;
    char p[300], name[64] = "";
    snprintf(p, sizeof(p), "/sys/bus/iio/devices/%s/name", e->d_name);
    FILE *f = fopen(p, "r");
    if (!f) continue;
    if (fgets(name, sizeof(name), f)) name[strcspn(name, "\n")] = 0;
    fclose(f);
    if (!strcmp(name, "ad9361-phy")) {
      snprintf(phy_path, sizeof(phy_path), "/sys/bus/iio/devices/%s", e->d_name);
      closedir(d);
      return 0;
    }
  }
  closedir(d);
  return -1;
}

static double read_attr(const char *name, double dflt) {
  char p[400], buf[64];
  snprintf(p, sizeof(p), "%s/%s", phy_path, name);
  FILE *f = fopen(p, "r");
  if (!f) return dflt;
  double v = dflt;
  if (fgets(buf, sizeof(buf), f)) v = atof(buf);
  fclose(f);
  return v;
}

/* ------------------------------------------------------------------ */
/* detection state (centred bin order j: j = N/2 is the LO) */
static double *P;        /* per-bin power, per frame (|y|^2 units) */
static double *lev;      /* 10 log10 P (--all mode) */
static double *flo_j;    /* floor per bin (dB) */
static uint8_t *filt;    /* -f map */
static uint8_t *hit;
static uint8_t *hang;
static float  *prio;     /* priority (peak SNR of the window that opened the bin) */
static uint8_t *gst;     /* gain state */
static uint8_t *gout;    /* gain written to the FPGA (centred order) */
static uint8_t *gprev;   /* previous FPGA gains (b order) */

/* published state for STATUS / CHANNELS */
typedef struct { int js, je, open; float prio; } win_t;
#define MAXWIN 1024
static win_t wins[MAXWIN];
static int nwins;
static double st_rate, st_fs, st_lo, st_update_s;
static int st_nruns, st_nbins, st_osh, st_psh, st_avg, st_dropped_windows;
static uint32_t st_updates;

static opts_t cur;
static int conf_dirty;

static int cmp_double(const void *a, const void *b) {
  double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

typedef struct { int s, e; float prio; } ival_t;
static int cmp_ival_prio(const void *a, const void *b) {
  float x = ((const ival_t *)a)->prio, y = ((const ival_t *)b)->prio;
  return (x < y) - (x > y);   /* descending */
}

/* --all (diagnostic) and --default: no detection. --all streams the -f ranges completely,
   except signals peaking more than --max-snr-db above the local floor (with 2 guard bins and
   a short hang so a jammer does not flicker in and out). Fills gout[] (centred order). */
static void filt_map(const opts_t *o, double fs, double lo);
static void all_update(const opts_t *o, double fs, double lo) {
  const int seg_l = 8, nseg = N >> seg_l, guard = 2, hang_n = 10;
  const double t_strong = 6.0;           /* a run starts where a bin is 6 dB over the floor */
  filt_map(o, fs, lo);
  if (o->dflt) { memset(gout, 0, N); nwins = 0; return; }
  memset(hit, 0, N);
  if (o->has_max_snr) {
    for (int j = 0; j < N; j++) lev[j] = 10 * log10(P[j] > 1e-30 ? P[j] : 1e-30);
    /* noise floor: 25 % percentile per 256-bin segment, min over +-2 segments */
    double segfl[64], tmp[256];
    for (int sg = 0; sg < nseg; sg++) {
      for (int i = 0; i < 256; i++) tmp[i] = lev[sg * 256 + i];
      qsort(tmp, 256, sizeof(double), cmp_double);
      segfl[sg] = tmp[64];
    }
    for (int j = 0; j < N; j++) {
      int sg = j >> seg_l; double m = segfl[sg];
      for (int d = -2; d <= 2; d++) { int t = sg + d; if (t >= 0 && t < nseg && segfl[t] < m) m = segfl[t]; }
      flo_j[j] = m;
    }
    int in_run = 0, rs = 0, re = 0; double peak = 0, fl = 0;
    for (int j = 0; j < N; j++) {
      int c = filt[j] && j > 2 && j < N - 3 && lev[j] >= flo_j[j] + t_strong;
      if (c) {
        if (!in_run) { in_run = 1; rs = re = j; peak = lev[j]; fl = flo_j[j]; }
        else { re = j; if (lev[j] > peak) peak = lev[j]; if (flo_j[j] < fl) fl = flo_j[j]; }
      }
      if (in_run && ((!c && j - re > 1) || j == N - 1)) {
        in_run = 0;
        if (peak - fl > o->max_snr_db)
          for (int k = rs - guard; k <= re + guard; k++) if (k >= 0 && k < N) hit[k] = 1;
      }
    }
  }
  for (int j = 0; j < N; j++) {
    if (hit[j]) hang[j] = hang_n; else if (hang[j]) hang[j]--;
    int blocked = hit[j] || hang[j] > 1;
    gout[j] = (filt[j] && !blocked) ? 255 : 0;
    prio[j] = blocked ? -1e30f : 0.0f;
  }
  gout[0] = gout[1] = gout[N - 1] = gout[N - 2] = 0;
  memcpy(gst, gout, N);
}

/* intervals of nonzero gain in b order (FPGA runs), with rate cap */
static int build_runs(const opts_t *o, double fs, uint8_t *gb, uint16_t *rs, uint16_t *rl, int *nbins_out, double *rate_out) {
  /* centred -> b order */
  for (int b = 0; b < N; b++) gb[b] = gout[b ^ (N / 2)];
  /* intervals with priority */
  static ival_t iv[8192];
  int niv = 0;
  for (int b = 0; b < N;) {
    if (!gb[b]) { b++; continue; }
    int s = b;
    float pr = -1e30f;
    while (b < N && gb[b]) { float p = prio[b ^ (N / 2)]; if (p > pr) pr = p; b++; }
    iv[niv].s = s; iv[niv].e = b - 1; iv[niv].prio = pr; niv++;
  }
  /* cap: output words per frame = 3 + sum(len + 1) */
  double frames_s = fs / H;
  double budget = o->max_rate * 1e6 / frames_s;   /* words per frame */
  qsort(iv, niv, sizeof(ival_t), cmp_ival_prio);
  double words = 3;
  int keep = 0;
  st_dropped_windows = 0;
  for (int i = 0; i < niv; i++) {
    double w = iv[i].e - iv[i].s + 2;
    if (keep < RUNS_MAX && words + w <= budget) { words += w; iv[keep++] = iv[i]; }
    else {
      st_dropped_windows++;
      for (int b = iv[i].s; b <= iv[i].e; b++) {
        gb[b] = 0;
        int j = b ^ (N / 2);
        gst[j] = 0; gout[j] = 0;
      }
    }
  }
  /* keep-alive: a run of zero-gain bins just below +fs/2 (outside the analog
     filter), so the DMA buffers keep filling when little is open */
  double min_words = o->min_rate * 1e6 / frames_s;
  if (words < min_words && keep < RUNS_MAX) {
    int pad = (int)ceil(min_words - words) - 1;
    if (pad < 1) pad = 1;
    if (pad > N / 64) pad = N / 64;
    int e = N / 2 - 3, s = e - pad + 1, ok = 1;
    for (int b = s - 1; b <= e + 1; b++) if (gb[b]) ok = 0;
    if (ok) { iv[keep].s = s; iv[keep].e = e; iv[keep].prio = -1e30f; keep++; words += pad + 1; }
  }
  /* sort kept by start */
  for (int i = 1; i < keep; i++) {
    ival_t t = iv[i]; int k = i - 1;
    while (k >= 0 && iv[k].s > t.s) { iv[k + 1] = iv[k]; k--; }
    iv[k + 1] = t;
  }
  int nb = 0;
  for (int i = 0; i < keep; i++) { rs[i] = iv[i].s; rl[i] = iv[i].e - iv[i].s + 1; nb += rl[i]; }
  *nbins_out = nb;
  *rate_out = words * frames_s;
  return keep;
}

static void publish_windows(void) {
  nwins = 0;
  for (int j = 0; j < N && nwins < MAXWIN;) {
    if (!gout[j]) { j++; continue; }
    int s = j; float pr = -1e30f; int open = 0;
    while (j < N && gout[j]) { if (prio[j] > pr) pr = prio[j]; if (gout[j] == 255) open = 1; j++; }
    wins[nwins].js = s; wins[nwins].je = j - 1; wins[nwins].open = open; wins[nwins].prio = pr;
    nwins++;
  }
}

/* ------------------------------------------------------------------ */
/* detector v2 glue */
static double now_s(void);
static det2_t D2;
static int d2_ok, d2_N, d2_kern = -1, d2_skip;
static double d2_fs, d2_lo, d2_gain = -1e9, t_start;
static char d2_sig[2048];
static int hw_kernel_ok;          /* bitstream >= 1.1 */
static int st_ntracks, st_nstream;
static double st_proc_ms;
static int d2_extra;          /* extra log2 averaging when the detector is the bottleneck */

static int use_det2(const opts_t *o) { return !o->all && !o->dflt; }

/* (re)initialise when the options, the rate or the kernel change; forget the state
   when the LO or the gain change */
static void d2_prepare(const opts_t *o, double fs, double lo, double gain) {
  char sig[2048];
  opts_string(o, sig, sizeof(sig));
  int kern = hw_kernel_ok ? o->kernel : 0;
  if (!d2_ok || strcmp(sig, d2_sig) || fs != d2_fs || N != d2_N || kern != d2_kern) {
    if (d2_ok) det2_free(&D2);
    det2_cfg_t c = o->d2;
    for (int i = 0; i < c.nspur; i++) c.spur_hz[i] = o->d2.spur_hz[i] * 1e6 - lo;   /* MHz -> offset */
    double upd = (1 << st_avg) * (N / 2) / fs;
    d2_ok = det2_init(&D2, &c, N, fs / N, upd, kern) == 0;
    snprintf(d2_sig, sizeof(d2_sig), "%s", sig);
    d2_fs = fs; d2_N = N; d2_lo = lo; d2_gain = gain;
    if (kern != d2_kern) d2_skip = 1;            /* this spectrum was accumulated with the old kernel */
    d2_kern = kern;
    printf("sgchd: detector v2 (re)started, kernel %s, %d bins of %.1f Hz, update %.1f ms\n",
           kern == 0 ? "sin5" : kern == 1 ? "sin3" : "sin1", N, fs / N, upd * 1e3);
  } else if (lo != d2_lo || gain != d2_gain) {
    det2_reset(&D2);
    if (lo != d2_lo) {          /* spur offsets follow the LO */
      det2_cfg_t c = o->d2;
      for (int i = 0; i < c.nspur; i++) c.spur_hz[i] = o->d2.spur_hz[i] * 1e6 - lo;
      det2_free(&D2);
      double upd = (1 << st_avg) * (N / 2) / fs;
      d2_ok = det2_init(&D2, &c, N, fs / N, upd, kern) == 0;
    }
    d2_lo = lo; d2_gain = gain;
  }
}

static void filt_map(const opts_t *o, double fs, double lo) {
  const double binw = fs / N;
  if (o->nf == 0) { memset(filt, 1, N); return; }
  memset(filt, 0, N);
  for (int i = 0; i < o->nf; i++) {
    int jlo = N / 2 + (int)floor((o->flo[i] * 1e6 - lo) / binw + 0.5);
    int jhi = N / 2 + (int)ceil((o->fhi[i] * 1e6 - lo) / binw - 0.5);
    if (jhi < 0 || jlo > N - 1) continue;
    if (jlo < 0) jlo = 0;
    if (jhi > N - 1) jhi = N - 1;
    memset(filt + jlo, 1, jhi - jlo + 1);
  }
}

/* --max-snr-db with detector v2: a track whose peak (fast spectrum / local floor, over the
 * track's extent) exceeds the limit is blocked -- not streamed, no slot -- and stays blocked
 * until its peak drops 3 dB below the limit (hysteresis: a fading jammer does not flicker).
 * Weaker skirts of a strong signal are absorbed into its track by the detector. */
static int blk_id[DET2_MAXTRACK], nblk;
static int st_nblocked, st_npass_bins;
static double d2_peak_db(const det2_track_t *t) {
  double m = 0;
  int s = t->lo_bin < 0 ? 0 : t->lo_bin, e = t->hi_bin > N - 1 ? N - 1 : t->hi_bin;
  for (int j = s; j <= e; j++) if (D2.R[j] > m) m = D2.R[j];
  return 10 * log10(m > 1e-6 ? m : 1e-6);
}
static int d2_blocked(const det2_track_t *t) {
  for (int k = 0; k < nblk; k++) if (blk_id[k] == t->id) return 1;
  return 0;
}
static void d2_block_update(const opts_t *o) {
  int keep[DET2_MAXTRACK], nk = 0;
  st_nblocked = 0;
  if (!o->has_max_snr) { nblk = 0; return; }
  for (int i = 0; i < D2.ntr && nk < DET2_MAXTRACK; i++) {
    const det2_track_t *t = &D2.tr[i];
    if (t->state == T_DROP) continue;
    double pk = d2_peak_db(t);
    int was = d2_blocked(t);
    if (pk > o->max_snr_db || (was && pk > o->max_snr_db - 3.0)) { keep[nk++] = t->id; st_nblocked++; }
  }
  memcpy(blk_id, keep, nk * sizeof(int)); nblk = nk;
}

/* v2 result -> centred gains + per-bin priority (for the rate cap) */
static void d2_gains(const opts_t *o, double fs, double lo) {
  filt_map(o, fs, lo);
  d2_block_update(o);
  memset(gout, 0, N);
  for (int j = 0; j < N; j++) prio[j] = -1e30f;
  st_ntracks = 0; st_nstream = 0;
  for (int i = 0; i < D2.ntr; i++) {
    det2_track_t *t = &D2.tr[i];
    st_ntracks++;
    if (!det2_streaming(&D2, t)) continue;
    int fcj = (int)lround(t->fc);
    if (fcj < 0 || fcj >= N || !filt[fcj]) continue;          /* outside the -f ranges */
    if (d2_blocked(t)) continue;                               /* --max-snr-db */
    st_nstream++;
    float pr = (float)t->snr_db + (t->state == T_ACTIVE || t->state == T_CONF ? 100.0f : t->state == T_HOLD ? 50.0f : 0.0f);
    int s, e;
    det2_extract(&D2, t, &s, &e);
    for (int j = s; j <= e; j++) { gout[j] = 255; if (pr > prio[j]) prio[j] = pr; }
  }
  /* --pass: fixed ranges always streamed (e.g. a block of 5 kHz channels the detector cannot
     separate); top priority under the rate cap */
  st_npass_bins = 0;
  const double binw = fs / N;
  for (int i = 0; i < o->npass; i++) {
    int jlo = N / 2 + (int)floor((o->plo[i] * 1e6 - lo) / binw + 0.5);
    int jhi = N / 2 + (int)ceil((o->phi[i] * 1e6 - lo) / binw - 0.5);
    if (jhi < 0 || jlo > N - 1) continue;
    if (jlo < 2) jlo = 2;
    if (jhi > N - 3) jhi = N - 3;
    for (int j = jlo; j <= jhi; j++) {
      if (!gout[j]) st_npass_bins++;
      gout[j] = 255;
      prio[j] = 1e4f;
    }
  }
  /* merge gaps <= merge_gap_bins between intervals (fewer run headers) */
  int last = -1000;
  for (int j = 0; j < N; j++) {
    if (!gout[j]) continue;
    if (j - last > 1 && j - last <= 1 + o->d2.merge_gap_bins)
      for (int k = last + 1; k < j; k++) { gout[k] = 255; prio[k] = prio[j] < prio[last] ? prio[j] : prio[last]; }
    last = j;
  }
  /* --max-snr-db: a blocked signal's own bins stay closed, whatever else would open them
     (a --pass range, a neighbouring downlink's guard band, the gap merging above) */
  if (st_nblocked) {
    for (int i = 0; i < D2.ntr; i++) {
      const det2_track_t *t = &D2.tr[i];
      if (t->state == T_DROP || !d2_blocked(t)) continue;
      for (int j = t->lo_bin < 0 ? 0 : t->lo_bin; j <= t->hi_bin && j < N; j++) gout[j] = 0;
    }
  }
  gout[0] = gout[1] = gout[N - 1] = gout[N - 2] = 0;
  memcpy(gst, gout, N);
}

/* ------------------------------------------------------------------ */
/* TCP */
#define MAXCLI 8
typedef struct { int fd; int sub; char buf[2048]; int len; } cli_t;
static cli_t cli[MAXCLI];
static int lfd = -1;

static void cli_send(cli_t *c, const void *d, size_t n) {
  const char *p = d;
  while (n > 0 && c->fd >= 0) {
    ssize_t k = send(c->fd, p, n, MSG_NOSIGNAL);
    if (k <= 0) { if (errno == EINTR) continue; close(c->fd); c->fd = -1; return; }
    p += k; n -= k;
  }
}
static void cli_printf(cli_t *c, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#include <stdarg.h>
static void cli_printf(cli_t *c, const char *fmt, ...) {
  char b[4096];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  if (n > (int)sizeof(b) - 1) n = sizeof(b) - 1;
  cli_send(c, b, n);
}

static void set_mode(int m);

static void handle_cmd(cli_t *c, char *line) {
  char *av[128];
  char copy[2048];
  snprintf(copy, sizeof(copy), "%s", line);
  int n = split_args(copy, av, 128);
  if (n == 0) return;
  if (!strcasecmp(av[0], "SET")) {
    opts_t o; char err[200];
    if (parse_opts(&o, n - 1, av + 1, err, sizeof(err))) { cli_printf(c, "ERR %s\n", err); return; }
    cur = o; conf_dirty = 1;
    set_mode(cur.mode);
    char s[1024]; opts_string(&cur, s, sizeof(s));
    cli_printf(c, "OK %s\n", s);
  } else if (!strcasecmp(av[0], "GET")) {
    char s[1024]; opts_string(&cur, s, sizeof(s));
    cli_printf(c, "OK %s\n", s);
  } else if (!strcasecmp(av[0], "STATUS")) {
    cli_printf(c, "OK fs=%.0f lo=%.0f bins=%d bin_hz=%.3f update_s=%.3f updates=%u runs=%d sel_bins=%d "
               "rate=%.0f osh=%d psh=%d avg=%d mode=%u frames=%u drops=%u sat=%u words=%u inovf=%u "
               "windows=%d capped=%d meas_fs=%.0f det=%s kernel=%s tracks=%d streaming=%d blocked=%d pass_bins=%d det_ms=%.1f\n",
               st_fs, st_lo, N, st_fs / N, st_update_s, st_updates, st_nruns, st_nbins, st_rate,
               st_osh, st_psh, st_avg, (rd(R_CTRL) >> 1) & 1, rd(R_FRAMES), rd(R_DROPS), rd(R_SAT),
               rd(R_WORDS), rd(R_INOVF), nwins, st_dropped_windows,
               rd(R_RATE) * core_hz / 16777216.0, use_det2(&cur) ? "v2" : "v15",
               d2_kern == 2 ? "sin1" : d2_kern == 1 ? "sin3" : "sin5", st_ntracks, st_nstream, st_nblocked, st_npass_bins, st_proc_ms);
  } else if (!strcasecmp(av[0], "CHANNELS")) {
    double binw = st_fs / N;
    cli_printf(c, "OK %d\n", nwins);
    for (int i = 0; i < nwins; i++) {
      double f0 = st_lo + (wins[i].js - N / 2 - 0.5) * binw, f1 = st_lo + (wins[i].je - N / 2 + 0.5) * binw;
      cli_printf(c, "%.6f %.3f %.1f %s\n", (f0 + f1) / 2e6, (f1 - f0) / 1e3,
                 wins[i].prio > -1e29f ? wins[i].prio : 0.0, wins[i].open ? "open" : "fading");
    }
  } else if (!strcasecmp(av[0], "TRACKS")) {
    double binw = st_fs / N, now = now_s() - t_start;
    int n = 0;
    for (int i = 0; d2_ok && i < D2.ntr; i++) if (D2.tr[i].state != T_DROP) n++;
    cli_printf(c, "OK %d fs=%.0f lo=%.0f bin_hz=%.3f t=%.3f\n", n, st_fs, st_lo, binw, now);
    for (int i = 0; d2_ok && i < D2.ntr; i++) {
      det2_track_t *t = &D2.tr[i];
      if (t->state == T_DROP) continue;
      int s, e;
      det2_extract(&D2, t, &s, &e);
      cli_printf(c, "%d %s %.6f %.3f %.1f %.1f %.2f %.2f %.2f %d %d %.6f %.6f %d %.1f\n", t->id, det2_state_name(t->state),
                 (st_lo + (t->fc - N / 2) * binw) / 1e6, t->bw * binw / 1e3, t->snr_db, t->z, t->conf,
                 now - t->t_first, now - t->t_last, t->parent, t->independent,
                 (st_lo + (s - N / 2 - 0.5) * binw) / 1e6, (st_lo + (e - N / 2 + 0.5) * binw) / 1e6,
                 det2_streaming(&D2, t) && lround(t->fc) >= 0 && lround(t->fc) < N && filt[lround(t->fc)] &&
                 !d2_blocked(t), d2_peak_db(t));
    }
  } else if (!strcasecmp(av[0], "SUBSCRIBE")) {
    c->sub = 1; cli_printf(c, "OK\n");
  } else if (!strcasecmp(av[0], "MODE") && n > 1) {
    set_mode(atoi(av[1]) ? 1 : 0); cli_printf(c, "OK\n");
  } else if (!strcasecmp(av[0], "QUIT")) {
    cli_printf(c, "OK bye\n"); close(c->fd); c->fd = -1;
  } else cli_printf(c, "ERR unknown command (SET GET STATUS CHANNELS TRACKS SUBSCRIBE MODE QUIT)\n");
}

static void tcp_poll(int timeout_ms) {
  fd_set rf; FD_ZERO(&rf);
  int mx = lfd;
  FD_SET(lfd, &rf);
  for (int i = 0; i < MAXCLI; i++) if (cli[i].fd >= 0) { FD_SET(cli[i].fd, &rf); if (cli[i].fd > mx) mx = cli[i].fd; }
  struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
  if (select(mx + 1, &rf, NULL, NULL, &tv) <= 0) return;
  if (FD_ISSET(lfd, &rf)) {
    int fd = accept(lfd, NULL, NULL);
    if (fd >= 0) {
      int one = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      int k;
      for (k = 0; k < MAXCLI; k++) if (cli[k].fd < 0) break;
      if (k == MAXCLI) close(fd);
      else { cli[k].fd = fd; cli[k].sub = 0; cli[k].len = 0; }
    }
  }
  for (int i = 0; i < MAXCLI; i++) {
    cli_t *c = &cli[i];
    if (c->fd < 0 || !FD_ISSET(c->fd, &rf)) continue;
    ssize_t k = recv(c->fd, c->buf + c->len, sizeof(c->buf) - 1 - c->len, 0);
    if (k <= 0) { close(c->fd); c->fd = -1; continue; }
    c->len += k; c->buf[c->len] = 0;
    char *nl;
    while (c->fd >= 0 && (nl = strchr(c->buf, '\n'))) {
      *nl = 0;
      handle_cmd(c, c->buf);
      if (c->fd < 0) break;
      int rest = c->len - (int)(nl + 1 - c->buf);
      memmove(c->buf, nl + 1, rest); c->len = rest; c->buf[c->len] = 0;
    }
    if (c->len >= (int)sizeof(c->buf) - 1) c->len = 0;
  }
}

/* noise-fill spectrum for subscribers: per-bin mean |X|^2 estimate in b order,
   in units of output samples (after osh), float32 little endian */
static void push_spectrum(int osh) {
  int any = 0;
  for (int i = 0; i < MAXCLI; i++) if (cli[i].fd >= 0 && cli[i].sub) any = 1;
  if (!any) return;
  static float *sp;
  if (!sp) sp = malloc(N * sizeof(float));
  /* kernel noise gain: sum(c^2)/256 = 70/256 */
  double k = 256.0 / 70.0 / pow(4.0, osh);
  for (int b = 0; b < N; b++) sp[b] = (float)(P[b ^ (N / 2)] * k);
  char hdr[256];
  int hn = snprintf(hdr, sizeof(hdr), "SPEC %d %.0f %.0f %d %u\n", N, st_fs, st_lo, osh, st_updates);
  for (int i = 0; i < MAXCLI; i++)
    if (cli[i].fd >= 0 && cli[i].sub) { cli_send(&cli[i], hdr, hn); cli_send(&cli[i], sp, N * sizeof(float)); }
}

/* ------------------------------------------------------------------ */
static volatile int stop;
static void on_sig(int s) { (void)s; stop = 1; }

static uint32_t ctrl_val(int mode) { return 1u | ((uint32_t)mode << 1) | (1u << 2); }
static void set_mode(int m) { wr(R_CTRL, ctrl_val(m)); }

static void save_conf(void) {
  char s[1024]; opts_string(&cur, s, sizeof(s));
  FILE *f = fopen(CONF_FILE ".tmp", "w");
  if (!f) return;
  fprintf(f, "%s\n", s);
  fclose(f);
  rename(CONF_FILE ".tmp", CONF_FILE);
}


/* AD936x digital interface delay (register 0x006: [7:4] RX data-clock delay,
 * [3:0] RX data delay). The driver's tuning runs with the FPGA idle; on this
 * clone the value it picks (0x20) sits at the edge of the window, and once the
 * channelizer runs ~2 % of the samples are lost (RX_FRAME errors). Measured
 * window with the core busy: 0x10 .. 0x0E (data - clock delay = -1 .. +14),
 * so 0x06 is the centre. Re-applied after every retune (rate changes). */
static int rx_delay = 0x06;            /* -1: leave the driver's value */
static char dbg_regs[300];

static int dbg_reg_read(int reg) {
  if (!dbg_regs[0]) return -1;
  char b[32];
  snprintf(b, sizeof(b), "0x%x", reg);
  int fd = open(dbg_regs, O_RDWR);
  if (fd < 0) return -1;
  int v = -1;
  if (write(fd, b, strlen(b)) > 0) {
    lseek(fd, 0, SEEK_SET);
    ssize_t k = read(fd, b, sizeof(b) - 1);
    if (k > 0) { b[k] = 0; v = (int)strtol(b, NULL, 0); }
  }
  close(fd);
  return v;
}

static void dbg_reg_write(int reg, int val) {
  if (!dbg_regs[0]) return;
  char b[32];
  snprintf(b, sizeof(b), "0x%x 0x%x", reg, val);
  int fd = open(dbg_regs, O_WRONLY);
  if (fd < 0) return;
  if (write(fd, b, strlen(b)) < 0) {}
  close(fd);
}

static void rx_delay_check(void) {
  if (rx_delay < 0) return;
  int v = dbg_reg_read(0x006);
  if (v >= 0 && v != rx_delay) {
    dbg_reg_write(0x006, rx_delay);
    printf("sgchd: AD936x RX interface delay 0x%02X -> 0x%02X\n", v, rx_delay);
  }
}

/* ------------------------------------------------------------------------ */
/* raw DMA stream (port + 1)                                                 */
static int stream_port;
static volatile int stream_clients;
static uint64_t st_stream_bytes;

static int sysfs_write(const char *path, const char *v) {
  int fd = open(path, O_WRONLY);
  if (fd < 0) return -1;
  int r = write(fd, v, strlen(v)) < 0 ? -1 : 0;
  close(fd);
  return r;
}

static int find_iio_dev(const char *want, char *dir, size_t n) {
  DIR *d = opendir("/sys/bus/iio/devices");
  if (!d) return -1;
  struct dirent *e;
  int found = -1;
  while ((e = readdir(d))) {
    if (strncmp(e->d_name, "iio:device", 10)) continue;
    char p[300], name[64] = "";
    snprintf(p, sizeof(p), "/sys/bus/iio/devices/%s/name", e->d_name);
    FILE *f = fopen(p, "r");
    if (!f) continue;
    if (fgets(name, sizeof(name), f)) name[strcspn(name, "\n")] = 0;
    fclose(f);
    if (!strcmp(name, want)) { snprintf(dir, n, "%s", e->d_name); found = 0; break; }
  }
  closedir(d);
  return found;
}

/* ADI IIO high-speed (mmap block) interface, as used by libiio's local backend */
struct blk_alloc_req { uint32_t type, size, count, id; };
struct blk { uint32_t id, size, bytes_used, type, flags, offset; uint64_t timestamp; };
#define BLK_ALLOC   _IOWR('i', 0xa0, struct blk_alloc_req)
#define BLK_FREE    _IO('i', 0xa1)
#define BLK_QUERY   _IOWR('i', 0xa2, struct blk)
#define BLK_ENQUEUE _IOWR('i', 0xa3, struct blk)
#define BLK_DEQUEUE _IOWR('i', 0xa4, struct blk)
#define NBLK 16
#define BLK_WORDS (1 << 17)                /* 512 KB per block, 8 MB queued */

typedef struct {
  char dev[64];
  int fd, mmapped, nblk;
  struct blk b[NBLK];
  void *addr[NBLK];
} rxbuf_t;

static int rx_buffer_open(rxbuf_t *r) {
  memset(r, 0, sizeof(*r));
  r->fd = -1;
  if (find_iio_dev("cf-ad9361-lpc", r->dev, sizeof(r->dev))) return -1;
  char base[200], p[300];
  snprintf(base, sizeof(base), "/sys/bus/iio/devices/%s", r->dev);
  snprintf(p, sizeof(p), "%s/buffer/enable", base);
  sysfs_write(p, "0");
  snprintf(p, sizeof(p), "%s/scan_elements", base);
  DIR *d = opendir(p);
  if (d) {
    struct dirent *e;
    while ((e = readdir(d))) {
      size_t l = strlen(e->d_name);
      if (l > 3 && !strcmp(e->d_name + l - 3, "_en")) {
        char q[400];
        snprintf(q, sizeof(q), "%s/%s", p, e->d_name);
        sysfs_write(q, (!strcmp(e->d_name, "in_voltage0_en") || !strcmp(e->d_name, "in_voltage1_en")) ? "1" : "0");
      }
    }
    closedir(d);
  }
  char v[32];
  snprintf(v, sizeof(v), "%d", BLK_WORDS);
  snprintf(p, sizeof(p), "%s/buffer/length", base);
  sysfs_write(p, v);
  snprintf(p, sizeof(p), "/dev/%s", r->dev);
  r->fd = open(p, O_RDWR);
  if (r->fd < 0) return -1;
  /* try the block interface (many queued blocks: no gaps when the link stalls briefly) */
  if (ioctl(r->fd, BLK_FREE, 0) == 0) {
    struct blk_alloc_req q = { 0, BLK_WORDS * 4, NBLK, 0 };
    if (ioctl(r->fd, BLK_ALLOC, &q) == 0 && q.count > 0) {
      r->nblk = q.count;
      int ok = 1;
      for (int i = 0; i < r->nblk && ok; i++) {
        r->b[i].id = i;
        if (ioctl(r->fd, BLK_QUERY, &r->b[i]) || ioctl(r->fd, BLK_ENQUEUE, &r->b[i])) { ok = 0; break; }
        r->addr[i] = mmap(0, r->b[i].size, PROT_READ | PROT_WRITE, MAP_SHARED, r->fd, r->b[i].offset);
        if (r->addr[i] == MAP_FAILED) { r->addr[i] = NULL; ok = 0; }
      }
      if (ok) r->mmapped = 1;
      else {
        for (int i = 0; i < r->nblk; i++) if (r->addr[i]) munmap(r->addr[i], r->b[i].size);
        ioctl(r->fd, BLK_FREE, 0);
        r->nblk = 0;
      }
    }
  }
  if (!r->mmapped) {   /* plain read() interface */
    snprintf(v, sizeof(v), "%d", 1 << 20);
    snprintf(p, sizeof(p), "%s/buffer/length", base);
    sysfs_write(p, v);
  }
  snprintf(p, sizeof(p), "%s/buffer/enable", base);
  if (sysfs_write(p, "1")) return -1;
  return 0;
}

static void rx_buffer_close(rxbuf_t *r) {
  char p[300];
  if (r->dev[0]) {
    snprintf(p, sizeof(p), "/sys/bus/iio/devices/%s/buffer/enable", r->dev);
    sysfs_write(p, "0");
  }
  if (r->mmapped) {
    for (int i = 0; i < r->nblk; i++) if (r->addr[i]) munmap(r->addr[i], r->b[i].size);
    ioctl(r->fd, BLK_FREE, 0);
  }
  if (r->fd >= 0) close(r->fd);
  r->fd = -1;
}

static int send_all(int c, const char *p, size_t n) {
  while (n) {
    ssize_t w = send(c, p, n, MSG_NOSIGNAL);
    if (w <= 0) { if (w < 0 && errno == EINTR) continue; return -1; }
    p += w; n -= (size_t)w;
  }
  return 0;
}

static void *stream_thread(void *arg) {
  (void)arg;
  int ls = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(stream_port), .sin_addr.s_addr = INADDR_ANY };
  if (bind(ls, (struct sockaddr *)&sa, sizeof(sa)) || listen(ls, 1)) { perror("sgchd: stream bind"); return NULL; }
  const size_t BUF = 1 << 20;
  char *buf = malloc(BUF);
  static rxbuf_t r;
  while (!stop) {
    int c = accept(ls, NULL, NULL);
    if (c < 0) continue;
    int sb = 4 << 20;
    setsockopt(c, SOL_SOCKET, SO_SNDBUF, &sb, sizeof(sb));
    if (rx_buffer_open(&r)) {
      const char *msg = "ERR cannot open the RX DMA buffer (in use by iiod / another program?)\n";
      if (write(c, msg, strlen(msg)) < 0) {}
      close(c);
      rx_buffer_close(&r);
      continue;
    }
    stream_clients = 1;
    set_mode(1);
    printf("sgchd: stream client connected (%s)\n", r.mmapped ? "mmap blocks" : "read()");
    while (!stop) {
      if (r.mmapped) {
        struct pollfd pf = { r.fd, POLLIN, 0 };
        if (poll(&pf, 1, 1000) <= 0) {
          /* nothing for 1 s: is the client still there? */
          char t;
          ssize_t k = recv(c, &t, 1, MSG_DONTWAIT);
          if (k == 0 || (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) break;
          continue;
        }
        struct blk b;
        memset(&b, 0, sizeof(b));
        if (ioctl(r.fd, BLK_DEQUEUE, &b)) { if (errno == EAGAIN || errno == EINTR) continue; break; }
        int bad = send_all(c, (const char *)r.addr[b.id], b.bytes_used);
        st_stream_bytes += b.bytes_used;
        r.b[b.id].bytes_used = r.b[b.id].size;
        ioctl(r.fd, BLK_ENQUEUE, &r.b[b.id]);
        if (bad) break;
      } else {
        ssize_t k = read(r.fd, buf, BUF);
        if (k < 0) { if (errno == EINTR || errno == EAGAIN) continue; break; }
        if (k == 0) { usleep(1000); continue; }
        if (send_all(c, buf, (size_t)k)) break;
        st_stream_bytes += k;
      }
    }
    rx_buffer_close(&r);
    close(c);
    stream_clients = 0;
    printf("sgchd: stream client gone\n");
  }
  free(buf);
  return NULL;
}

static double now_s(void) {
  struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
  int port = DEF_PORT, use_conf = 1, have_cli_opts = 0;
  setvbuf(stdout, NULL, _IOLBF, 0);
  char *oargv[256]; int oargc = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--no-conf")) use_conf = 0;
    else if (!strcmp(argv[i], "--rx-delay") && i + 1 < argc) {
      i++;
      rx_delay = strcmp(argv[i], "off") ? (int)strtol(argv[i], NULL, 0) & 0xFF : -1;
    }
    else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { fputs(USAGE, stdout); return 0; }
    else { oargv[oargc++] = argv[i]; have_cli_opts = 1; }
  }
  char err[200];
  if (have_cli_opts) {
    if (parse_opts(&cur, oargc, oargv, err, sizeof(err))) { fprintf(stderr, "sgchd: %s\n%s", err, USAGE); return 2; }
    conf_dirty = 1;
  } else {
    opts_default(&cur);
    if (use_conf) {
      FILE *f = fopen(CONF_FILE, "r");
      char line[2048];
      if (f && fgets(line, sizeof(line), f)) {
        char *av[128]; int n = split_args(line, av, 128);
        opts_t o;
        if (!parse_opts(&o, n, av, err, sizeof(err))) cur = o;
        else fprintf(stderr, "sgchd: ignoring %s: %s\n", CONF_FILE, err);
      }
      if (f) fclose(f);
    }
  }

  int mfd = open("/dev/mem", O_RDWR | O_SYNC);
  if (mfd < 0) { perror("sgchd: /dev/mem"); return 1; }
  void *m = mmap(NULL, SGC_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, SGC_BASE);
  if (m == MAP_FAILED) { perror("sgchd: mmap"); return 1; }
  regs = m;
  uint32_t id = rd(R_ID), ver = rd(R_VER), cfg = rd(R_CONFIG);
  if (id != 0x53474348) { fprintf(stderr, "sgchd: channelizer not found at 0x%08X (id 0x%08X): wrong bitstream\n", SGC_BASE, id); return 1; }
  if ((ver >> 16) != FOR_MAJOR) { fprintf(stderr, "sgchd: bitstream v%u.%u, this daemon is for v%d.x\n", ver >> 16, ver & 0xFFFF, FOR_MAJOR); return 1; }
  hw_kernel_ok = (ver & 0xFFFF) >= FOR_MINOR;
  if (!hw_kernel_ok) printf("sgchd: bitstream v1.0: detection kernel fixed to sin^5 (load v1.1 for --kernel)\n");
  L = cfg >> 24; N = 1 << L; H = N / 2; RUNS_MAX = 1 << ((cfg >> 16) & 0xFF);
  core_hz = rd(R_CORE_HZ);
  printf("sgchd: SGCH v%u.%u, %d bins, %d runs max\n", ver >> 16, ver & 0xFFFF, N, RUNS_MAX);
  if (find_phy()) { fprintf(stderr, "sgchd: ad9361-phy not found\n"); return 1; }
  snprintf(dbg_regs, sizeof(dbg_regs), "/sys/kernel/debug/iio/%s/direct_reg_access", strrchr(phy_path, '/') + 1);
  if (access(dbg_regs, W_OK)) dbg_regs[0] = 0;
  rx_delay_check();

  P = calloc(N, sizeof(double)); lev = calloc(N, sizeof(double));
  flo_j = calloc(N, sizeof(double)); filt = calloc(N, 1); hit = calloc(N, 1); hang = calloc(N, 1);
  prio = calloc(N, sizeof(float)); gst = calloc(N, 1); gout = calloc(N, 1); gprev = calloc(N, 1);
  uint8_t *gb = calloc(N, 1);
  uint16_t *rs = calloc(RUNS_MAX, 2), *rl = calloc(RUNS_MAX, 2);

  signal(SIGINT, on_sig); signal(SIGTERM, on_sig); signal(SIGPIPE, SIG_IGN);
  for (int i = 0; i < MAXCLI; i++) cli[i].fd = -1;
  lfd = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1; setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in sa = { .sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = INADDR_ANY };
  if (bind(lfd, (struct sockaddr *)&sa, sizeof(sa)) || listen(lfd, 4)) { perror("sgchd: bind"); return 1; }
  stream_port = port + STREAM_PORT_OFS;
  pthread_t sth;
  pthread_create(&sth, NULL, stream_thread, NULL);
  pthread_detach(sth);

  /* start: empty selection, core running */
  wr(R_NRUNS, 0); for (int a = 0; a < N / 4; a++) wr(R_GAIN + 4 * a, 0);
  wr(R_BANK, 1);
  int psh = cur.psh >= 0 ? cur.psh : 0, osh = cur.osh >= 0 ? cur.osh : 0;
  int avg = 10;
  wr(R_CTRL, ctrl_val(0) | 0x80000000u);
  wr(R_CTRL, ctrl_val(cur.mode));
  double last_conf_save = 0;
  st_fs = read_attr("in_voltage_sampling_frequency", 30.72e6);
  t_start = now_s();

  while (!stop) {
    /* parameters that depend on fs */
    st_fs = read_attr("in_voltage_sampling_frequency", st_fs);
    rx_delay_check();
    st_lo = read_attr("out_altvoltage0_RX_LO_frequency", 0);
    double frames_s = st_fs / H;
    if (cur.avg >= 0) avg = cur.avg > 12 ? 12 : cur.avg;
    else {
      avg = (int)lround(log2((use_det2(&cur) ? 0.028 : 0.15) * frames_s));
      /* v2: if the ARM cannot keep up, average longer (and come back when it can) */
      if (use_det2(&cur) && st_proc_ms > 0) {
        double upd_ms = (1 << (avg + d2_extra)) / frames_s * 1e3;
        if (st_proc_ms > 0.85 * upd_ms && d2_extra < 2) d2_extra++;
        else if (d2_extra > 0 && st_proc_ms < 0.35 * upd_ms) d2_extra--;
        avg += d2_extra;
      }
      if (avg < 0) avg = 0;
      if (avg > 12) avg = 12;
    }
    st_avg = avg;
    if (cur.psh >= 0) psh = cur.psh;
    if (cur.osh >= 0) osh = cur.osh;
    wr(R_PARAM, (uint32_t)avg | ((uint32_t)psh << 8) | ((uint32_t)osh << 16) |
                ((uint32_t)(hw_kernel_ok && use_det2(&cur) ? cur.kernel : 0) << 24));
    st_update_s = (1 << avg) / frames_s;

    /* wait for the spectrum, serving TCP meanwhile (and at least once per update,
       so that control commands are answered even when the detector is the bottleneck) */
    tcp_poll(0);
    double t0 = now_s();
    int ok = 0;
    while (!stop) {
      if (rd(R_STATUS) & 1) { ok = 1; break; }
      tcp_poll(3);
      if (now_s() - t0 > 3.0 + 2 * st_update_s) break;
    }
    if (stop) break;
    if (!ok) {
      /* no samples (RX not streaming?) -- restart the average and retry */
      wr(R_SPEC, 1);
      continue;
    }
    int nsat = 0;
    double scale = pow(2.0, psh) / (double)(1 << avg);
    double pmax_sel = 0;
    for (int b = 0; b < N; b++) {
      uint32_t v = rd(R_SPECRAM + 4 * b);
      if (v == 0xFFFFFFFFu) nsat++;
      P[b ^ (N / 2)] = v * scale;
    }
    wr(R_SPEC, 1);   /* next average starts while we compute */

    if (use_det2(&cur)) {
      d2_prepare(&cur, st_fs, st_lo, read_attr("in_voltage0_hardwaregain", 0));
      if (d2_skip) { d2_skip = 0; continue; }
      double tp = now_s();
      if (d2_ok) det2_update(&D2, P, now_s() - t_start);
      double dtp = now_s() - tp;
      st_proc_ms = st_proc_ms ? 0.9 * st_proc_ms + 0.1 * dtp * 1e3 : dtp * 1e3;
      d2_gains(&cur, st_fs, st_lo);
    } else {
      all_update(&cur, st_fs, st_lo);
    }
    int nb = 0; double rate = 0;
    int nr = build_runs(&cur, st_fs, gb, rs, rl, &nb, &rate);
    publish_windows();

    /* automatic output shift: strongest selected bin below -12 dBFS of int16 */
    for (int b = 0; b < N; b++) if (gb[b]) { double p = P[b ^ (N / 2)] * 4.0; if (p > pmax_sel) pmax_sel = p; }
    if (cur.osh < 0) {
      double amp = sqrt(pmax_sel);
      int want = 0;
      while (want < 8 && amp / pow(2.0, want) > 8192) want++;
      if (want > osh || want < osh - 1) osh = want;        /* hysteresis on the way down */
    }
    if (cur.psh < 0) {
      if (nsat > 0 && psh < 15) psh++;
      else if (nsat == 0 && psh > 0) {
        uint32_t mx = 0;
        for (int b = 0; b < N; b++) { double v = P[b] / scale; if (v > mx) mx = (uint32_t)fmin(v, 4294967295.0); }
        if (mx < 0x10000000u) psh--;
      }
    }

    /* write the shadow bank and swap */
    for (int a = 0; a < N / 4; a++) {
      uint32_t w = gb[4 * a] | (gb[4 * a + 1] << 8) | (gb[4 * a + 2] << 16) | ((uint32_t)gb[4 * a + 3] << 24);
      wr(R_GAIN + 4 * a, w);
    }
    for (int i = 0; i < nr; i++) wr(R_RUNS + 4 * i, ((uint32_t)rl[i] << 16) | rs[i]);
    wr(R_NRUNS, nr);
    wr(R_BANK, 1);
    double ts = now_s();
    while ((rd(R_BANK) & 1) && now_s() - ts < 0.5) usleep(200);

    st_nruns = nr; st_nbins = nb; st_rate = rate; st_osh = osh; st_psh = psh; st_avg = avg;
    st_updates++;
    push_spectrum(osh);
    if (conf_dirty && now_s() - last_conf_save > 2.0) { save_conf(); conf_dirty = 0; last_conf_save = now_s(); }
  }
  /* leave the radio as a normal Pluto */
  wr(R_CTRL, 0);
  printf("sgchd: stopped, RX DMA back to normal samples\n");
  return 0;
}
