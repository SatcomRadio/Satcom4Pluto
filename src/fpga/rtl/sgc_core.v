//
// SPDX-License-Identifier: MIT
//
// sgc_core.v -- satellite-downlink channelizer core (one clock domain).
//
// Input: complex samples (IN_W bits I/Q) at up to clk/2.6 samples/s.
// Analysis filter bank: N = 2^L bins, sqrt-Hann window w[n] = sin(pi n/N),
// hop N/2 (50 % overlap). With a sqrt-Hann synthesis window and overlap-add
// on the host this is perfect reconstruction for any selection of bins.
//
//   samples -> ring buffer (2N) -> windowed frame, read in bit-reversed order
//           -> DIT FFT (natural-order bins b = 0..N-1, b = 0 is DC)
//           -> (a) detection: kernel [1 -4 6 -4 1]/16 (sin^5 window, cfg_kern = 0),
//                  [-1 2 -1]/4 (sin^3, cfg_kern = 1) or [1] (sin^1, cfg_kern = 2),
//                  |.|^2 >> PSH, summed over 2^AVG frames into spec[b]
//           -> (b) selected runs of bins, per-bin 8-bit gain, packed into
//                  32-bit words for the DMA
//
// Output frame (one per hop, only when the run list is not empty or always,
// see cfg_hdr_always):
//   H0 = 0x53474348 ("SGCH")
//   H1 = frame sequence number k (frame k covers input samples kN/2 .. kN/2+N-1)
//   H2 = {osh[3:0], L[3:0], flags[7:0], nruns[15:0]}  flags[0]: frames dropped before this one
//   then for each run: R = {len[15:0], start[15:0]} followed by len words
//   {Q[15:0], I[15:0]} = X[b] * gain/256 / 2^osh (bins start .. start+len-1)
// Bin b is at f = f_LO + (b < N/2 ? b : b - N) * fs / N.
//
// Runs (written by the ARM, double-buffered with the gains): sorted by start,
// not wrapping past N-1, at least one bin apart.

module sgc_core #(
  parameter L      = 13,          // N = 2^L bins
  parameter IN_W   = 12,          // input sample width
  parameter RUNS_L = 8            // up to 2^RUNS_L runs per frame
) (
  input                   clk,
  input                   rst,            // synchronous, resets the stream state only

  // input samples
  input                   in_valid,
  input  [IN_W-1:0]       in_i,
  input  [IN_W-1:0]       in_q,

  // output words
  output reg              out_push = 1'b0,
  output reg [31:0]       out_word = 32'd0,
  input                   out_afull,

  // configuration
  input  [3:0]            cfg_avg_l,      // log2 frames per spectrum (0..12)
  input  [3:0]            cfg_psh,        // power right shift
  input  [3:0]            cfg_osh,        // output right shift (0..8)
  input  [1:0]            cfg_kern,       // detection kernel: 0 sin^5, 1 sin^3, 2/3 sin^1 (v1.1)
  input                   cfg_out_en,     // emit output words

  // shadow gain bank write: word = gains of bins 4a .. 4a+3 (byte k = bin 4a+k)
  input                   g_we,
  input  [L-3:0]          g_addr,
  input  [31:0]           g_wdata,
  // shadow run list write
  input                   r_we,
  input  [RUNS_L-1:0]     r_addr,
  input  [15:0]           r_start,
  input  [15:0]           r_len,
  input  [RUNS_L:0]       r_nruns,        // number of runs in the shadow bank
  input                   swap_req,       // pulse: make the shadow bank active at the next frame
  output reg              swap_pending = 1'b0,
  output reg              act_bank = 1'b0,

  // power spectrum
  output reg              spec_ready = 1'b0,
  input                   spec_release,   // pulse: start the next average
  input  [L-1:0]          spec_raddr,
  output [31:0]           spec_rdata,     // 1 clk after spec_raddr (while spec_ready)
  output reg [31:0]       spec_seq = 32'd0,

  // status
  output reg [31:0]       st_frames = 32'd0,   // output frames
  output reg [31:0]       st_drops = 32'd0,    // input frames skipped (overrun)
  output reg [31:0]       st_sat = 32'd0,      // saturated output samples
  output reg [31:0]       st_words = 32'd0,    // output words
  output reg [31:0]       st_samples = 32'd0   // input samples
);
  localparam N  = 1 << L;
  localparam H  = N >> 1;
  localparam XW = L + 2;                       // FFT index: {frame[1:0], position}

  function [L-1:0] bitrev(input [L-1:0] v);
    integer k;
    begin for (k = 0; k < L; k = k + 1) bitrev[k] = v[L-1-k]; end
  endfunction

  // =====================================================================
  // Ring buffer of 2N input samples
  reg  [31:0] wcnt = 32'd0;
  wire [2*IN_W-1:0] ring_q;
  wire              ring_re;
  wire [L:0]        ring_ra;
  sgc_sdpram #(.DW(2*IN_W), .AW(L+1)) ring (
    .clk(clk), .we(in_valid), .waddr(wcnt[L:0]), .wdata({in_i, in_q}),
    .re(ring_re), .raddr(ring_ra), .rdata(ring_q));
  always @(posedge clk)
    if (rst) wcnt <= 32'd0;
    else if (in_valid) wcnt <= wcnt + 1'b1;
  always @(posedge clk) if (in_valid) st_samples <= st_samples + 1'b1;

  // =====================================================================
  // Frame scheduler / reader
  wire        en;                               // pipeline advance
  reg  [31:0] kend = N;                         // end sample (exclusive) of the next frame
  reg  [31:0] kseq = 32'd0;                     // its sequence number
  reg         active = 1'b0;
  reg  [L-1:0] p = 0;                           // stream position in the frame
  reg  [L:0]  base = 0;
  reg  [1:0]  ffed = 2'd0;                      // frames fed (mod 4)
  reg  [31:0] seq_tab [0:3];
  reg  [3:0]  drop_tab = 4'd0;
  reg         drop_flag = 1'b0;
  wire [31:0] lag   = wcnt - kend;              // samples since the next frame completed
  wire        avail = ~lag[31];
  wire        late  = avail && (lag >= H);      // ring would be overrun -> skip ahead
  wire [31:0] kend_new = wcnt & ~(H - 1);       // latest complete frame end (multiple of H)
  integer qi_;
  initial for (qi_ = 0; qi_ < 4; qi_ = qi_ + 1) seq_tab[qi_] = 32'd0;

  always @(posedge clk) begin
    if (rst) begin
      kend <= N; kseq <= 32'd0; active <= 1'b0; p <= 0; ffed <= 2'd0; drop_flag <= 1'b0;
    end else begin
      if (!active && avail) begin
        if (late) begin
          // skip to the latest complete frame
          kseq <= kseq + ((kend_new - kend) >> (L - 1));
          st_drops <= st_drops + ((kend_new - kend) >> (L - 1));
          kend <= kend_new;
          drop_flag <= 1'b1;
        end else begin
          active <= 1'b1;
          p <= 0;
          base <= kend[L:0] - N[L:0];
          seq_tab[ffed] <= kseq;
          drop_tab[ffed] <= drop_flag;
          drop_flag <= 1'b0;
          kseq <= kseq + 1'b1;
          kend <= kend + H;
        end
      end else if (active && en) begin
        p <= p + 1'b1;
        if (p == N - 1) begin active <= 1'b0; ffed <= ffed + 1'b1; end
      end
    end
  end

  // ring read + window (sqrt-Hann, Q17: w[n] = sin(pi n/N), n = bitrev(p))
  wire [L-1:0] n_w  = bitrev(p);
  wire [L:0]   raddr = base + {1'b0, n_w};
  wire [L-1:0] widx  = n_w[L-1] ? (N - n_w) : n_w;      // 0..N/2
  (* rom_style = "block" *)
  reg  [16:0]  wrom [0:H-1];
  integer wi_, wv_;
  initial for (wi_ = 0; wi_ < H; wi_ = wi_ + 1) begin
    wv_ = $rtoi($floor($sin(3.14159265358979323846 * wi_ / N) * 131072.0 + 0.5));
    if (wv_ > 131071) wv_ = 131071;
    wrom[wi_] = wv_[16:0];
  end
  reg [2*IN_W-1:0] r1_x = 0;
  reg [16:0]       r1_w = 0;
  reg              r1_one = 1'b0;
  reg [XW-1:0]     r1_p = 0;
  assign ring_re = en;
  assign ring_ra = raddr;
  always @(*) r1_x = ring_q;
  always @(posedge clk) if (en) begin
    r1_w <= wrom[widx[L-2:0]];
    r1_one <= widx[L-1];                       // widx == N/2 -> w = 1.0
    r1_p <= {ffed, p};
  end
  wire signed [IN_W-1:0] r1_i = r1_x[2*IN_W-1:IN_W];
  wire signed [IN_W-1:0] r1_q = r1_x[IN_W-1:0];
  wire signed [18:0]     r1_ws = r1_one ? 19'sd131071 : $signed({2'b00, r1_w});
  reg  signed [IN_W+18:0] r2_i = 0, r2_q = 0;
  reg  [XW-1:0]           r2_p = 0;
  always @(posedge clk) if (en) begin
    r2_i <= r1_i * r1_ws;
    r2_q <= r1_q * r1_ws;
    r2_p <= r1_p;
  end
  // x * w >> (IN_W + 17 - 16) -> 16 bits (input full scale -> 2^15)
  localparam WSH = IN_W + 1;
  function [15:0] wrnd(input signed [IN_W+18:0] v);
    reg signed [IN_W+18:0] t;
    begin t = (v + (1 <<< (WSH - 1))) >>> WSH; wrnd = t[15:0]; end
  endfunction
  reg [15:0]   r3_i = 0, r3_q = 0;
  reg [XW-1:0] r3_p = 0;
  always @(posedge clk) if (en) begin
    r3_i <= wrnd(r2_i); r3_q <= wrnd(r2_q); r3_p <= r2_p;
  end

  // =====================================================================
  // FFT: 6 grow stages then scale by 1/2 -> 22-bit bins
  function [2*L-1:0] fmodes(input integer d);
    integer k;
    begin fmodes = 0; for (k = 0; k < L; k = k + 1) fmodes[2*k +: 2] = (k < 6) ? 2'd0 : 2'd1; end
  endfunction
  localparam FW = 22;
  wire [FW-1:0] Y_r, Y_i;
  wire [XW-1:0] Y_x;
  sgc_fft_dit #(.L(L), .IW(16), .STAGE_MODE(fmodes(0)), .INV(0), .OW(FW), .XW(XW)) fft (
    .clk(clk), .en(en), .xr(r3_i), .xi(r3_q), .xidx(r3_p), .yr(Y_r), .yi(Y_i), .yidx(Y_x));
  wire [L-1:0] yb = Y_x[L-1:0];
  wire [1:0]   yf = Y_x[L+1:L];

  // pipeline primed: the first frame has passed through the FFT
  localparam PRIME = 2 * N + 256;
  reg [L+1:0] pcnt = 0;
  always @(posedge clk)
    if (rst) pcnt <= 0;
    else if (en && pcnt != PRIME) pcnt <= pcnt + 1'b1;
  wire primed = (pcnt == PRIME);

  // =====================================================================
  // Gain RAM (2 banks x N/4 words) and run lists (2 banks, LUT RAM)
  (* ram_style = "block" *)
  reg  [31:0] gram [0:(N/2)-1];
  reg  [31:0] g_q = 32'd0;
  (* ram_style = "distributed" *)
  reg  [2*L-1:0] rram [0:(2 << RUNS_L)-1];     // {start, end}
  reg  [RUNS_L:0] nruns_b [0:1];
  initial begin nruns_b[0] = 0; nruns_b[1] = 0; end
  integer gi_;
  initial begin
    for (gi_ = 0; gi_ < N/2; gi_ = gi_ + 1) gram[gi_] = 32'd0;
    for (gi_ = 0; gi_ < (2 << RUNS_L); gi_ = gi_ + 1) rram[gi_] = 0;
  end
  wire [15:0] r_end = r_start + r_len - 1'b1;
  always @(posedge clk) begin
    if (g_we) gram[{~act_bank, g_addr}] <= g_wdata;
    if (r_we) rram[{~act_bank, r_addr}] <= {r_start[L-1:0], r_end[L-1:0]};
  end
  always @(posedge clk) nruns_b[~act_bank] <= r_nruns;   // shadow count follows the register

  // =====================================================================
  // Output path. Stage A: run tracking (prefetched run list), gain read.
  // The bank swap and the run pointer reset happen with the last bin
  // (N-1) of a frame, so bin 0 of the next frame already uses the new bank.
  reg              started = 1'b0;
  wire             fstart = (yb == 0);
  wire             flast  = (yb == N - 1);
  wire             fvalid = started | (primed & fstart);
  wire             bank_x = swap_pending ? ~act_bank : act_bank;
  reg  [RUNS_L:0]  rr = 0;
  reg  [RUNS_L:0]  nr_cur = 0;
  reg  [L-1:0]     cur_s = 0, cur_e = 0;
  reg              cur_ok = 1'b0;
  reg  [2*L-1:0]   nxt = 0;
  wire [RUNS_L:0]  rr1 = rr + 1'b1;
  wire             inrun = cur_ok && (yb >= cur_s) && (yb <= cur_e);
  wire             isend = inrun && (yb == cur_e);

  reg signed [FW-1:0] a_r = 0, a_i = 0;
  reg [L-1:0]      a_b = 0;
  reg              a_v = 1'b0, a_fs = 1'b0, a_in = 1'b0, a_st = 1'b0;
  reg [L:0]        a_len = 0;
  reg [31:0]       a_seq = 32'd0;
  reg              a_drop = 1'b0;
  reg [RUNS_L:0]   a_nr = 0;
  reg [3:0]        a_osh = 4'd0;
  always @(posedge clk) begin
    if (rst) begin
      started <= 1'b0; rr <= 0; cur_ok <= 1'b0;
    end else if (en) begin
      if (primed && fstart) started <= 1'b1;
      if (flast) begin
        act_bank <= bank_x;
        rr <= 0;
        {cur_s, cur_e} <= rram[{bank_x, {RUNS_L{1'b0}}}];
        nxt <= rram[{bank_x, {{(RUNS_L-1){1'b0}}, 1'b1}}];
        nr_cur <= nruns_b[bank_x];
        cur_ok <= (nruns_b[bank_x] != 0);
      end else if (isend) begin
        rr <= rr1;
        {cur_s, cur_e} <= nxt;
        cur_ok <= (rr1 < nr_cur);
      end else begin
        nxt <= rram[{act_bank, rr1[RUNS_L-1:0]}];
      end
    end
    if (en) begin
      a_r <= Y_r; a_i <= Y_i; a_b <= yb;
      a_v <= fvalid; a_fs <= fstart;
      a_in <= inrun; a_st <= inrun && (yb == cur_s);
      a_len <= {1'b0, cur_e} - {1'b0, cur_s} + 1'b1;
      if (fstart) begin a_seq <= seq_tab[yf]; a_drop <= drop_tab[yf]; a_nr <= nr_cur; a_osh <= cfg_osh; end
      g_q <= gram[{act_bank, yb[L-1:2]}];
    end
  end
  // swap request: taken at the end of the current frame
  always @(posedge clk)
    if (swap_req) swap_pending <= 1'b1;
    else if (en && flast) swap_pending <= 1'b0;

  // Stage A2 (v1.1): gain byte select registered -- the block-RAM clock-to-out,
  // the byte mux and the multiplier no longer share one 10 ns cycle
  reg [7:0]           a2_g = 8'd0;
  reg signed [FW-1:0] a2_r = 0, a2_i = 0;
  reg [L-1:0]         a2_b = 0;
  reg                 a2_v = 1'b0, a2_fs = 1'b0, a2_in = 1'b0, a2_st = 1'b0;
  reg [L:0]           a2_len = 0;
  reg [31:0]          a2_seq = 32'd0;
  reg                 a2_drop = 1'b0;
  reg [RUNS_L:0]      a2_nr = 0;
  reg [3:0]           a2_osh = 4'd0;
  always @(posedge clk) if (en) begin
    a2_g <= g_q[8*a_b[1:0] +: 8];
    a2_r <= a_r; a2_i <= a_i; a2_b <= a_b;
    a2_v <= a_v; a2_fs <= a_fs; a2_in <= a_in; a2_st <= a_st; a2_len <= a_len;
    a2_seq <= a_seq; a2_drop <= a_drop; a2_nr <= a_nr; a2_osh <= a_osh;
  end

  // Stage B: gain multiply
  wire [7:0]  g_b = a2_g;
  reg signed [FW+8:0] b_pr = 0, b_pi = 0;
  reg         b_u = 1'b0, b_v = 1'b0, b_fs = 1'b0, b_in = 1'b0, b_st = 1'b0;
  reg [L-1:0] b_b = 0;
  reg [L:0]   b_len = 0;
  reg [31:0]  b_seq = 32'd0;
  reg         b_drop = 1'b0;
  reg [RUNS_L:0] b_nr = 0;
  reg [3:0]   b_osh = 4'd0;
  always @(posedge clk) if (en) begin
    b_pr <= (g_b == 8'd255) ? ($signed(a2_r) <<< 8) : $signed(a2_r) * $signed({1'b0, g_b});
    b_pi <= (g_b == 8'd255) ? ($signed(a2_i) <<< 8) : $signed(a2_i) * $signed({1'b0, g_b});
    b_v <= a2_v; b_fs <= a2_fs; b_in <= a2_in; b_st <= a2_st; b_b <= a2_b; b_len <= a2_len;
    b_seq <= a2_seq; b_drop <= a2_drop; b_nr <= a2_nr; b_osh <= a2_osh;
  end

  // Stage S: >> (8 + osh) with rounding
  reg signed [FW+9:0] s_r = 0, s_i = 0;
  reg         s_v = 1'b0, s_fs = 1'b0, s_in = 1'b0, s_st = 1'b0;
  reg [L-1:0] s_b = 0;
  reg [L:0]   s_len = 0;
  reg [31:0]  s_seq = 32'd0;
  reg         s_drop = 1'b0;
  reg [RUNS_L:0] s_nr = 0;
  reg [3:0]   s_osh = 4'd0;
  wire [4:0]  shv = 5'd8 + b_osh;
  wire signed [FW+9:0] rnd_c = $signed({{(FW+9){1'b0}}, 1'b1}) <<< (shv - 1'b1);
  always @(posedge clk) if (en) begin
    s_r <= ($signed({b_pr[FW+8], b_pr}) + rnd_c) >>> shv;
    s_i <= ($signed({b_pi[FW+8], b_pi}) + rnd_c) >>> shv;
    s_v <= b_v; s_fs <= b_fs; s_in <= b_in; s_st <= b_st; s_b <= b_b; s_len <= b_len;
    s_seq <= b_seq; s_drop <= b_drop; s_nr <= b_nr; s_osh <= b_osh;
  end

  // Stage C: saturate to 16 bits
  function [16:0] sat16(input signed [FW+9:0] q);
    begin
      if (q > 32767)       sat16 = {1'b1, 16'h7FFF};
      else if (q < -32768) sat16 = {1'b1, 16'h8000};
      else                 sat16 = {1'b0, q[15:0]};
    end
  endfunction
  wire [16:0] cr_ = sat16(s_r), ci_ = sat16(s_i);
  reg  [31:0] c_w = 32'd0;
  reg         c_sat = 1'b0, c_v = 1'b0, c_fs = 1'b0, c_in = 1'b0, c_st = 1'b0;
  reg [L-1:0] c_b = 0;
  reg [L:0]   c_len = 0;
  reg [31:0]  c_seq = 32'd0;
  reg         c_drop = 1'b0;
  reg [RUNS_L:0] c_nr = 0;
  reg [3:0]   c_osh = 4'd0;
  always @(posedge clk) if (en) begin
    c_w <= {ci_[15:0], cr_[15:0]};
    c_sat <= cr_[16] | ci_[16];
    c_v <= s_v & cfg_out_en; c_fs <= s_fs; c_in <= s_in; c_st <= s_st; c_b <= s_b; c_len <= s_len;
    c_seq <= s_seq; c_drop <= s_drop; c_nr <= s_nr; c_osh <= s_osh;
  end

  // Stage D: header insertion and push
  reg  [1:0] hcnt = 2'd0;
  reg        rh_done = 1'b0;
  wire       need_fh = c_v && c_fs && (hcnt != 2'd3);
  wire       need_rh = c_v && c_st && !rh_done && !need_fh;
  wire       need_h  = need_fh | need_rh;
  wire [3:0] l4 = L;
  wire [15:0] nr16 = c_nr;
  wire [15:0] len16 = c_len;
  wire [15:0] b16 = c_b;
  assign en = active && !out_afull && !need_h;

  always @(posedge clk) begin
    out_push <= 1'b0;
    if (rst) begin
      hcnt <= 2'd0; rh_done <= 1'b0;
    end else if (!out_afull && need_fh) begin
      out_push <= 1'b1;
      out_word <= (hcnt == 2'd0) ? 32'h53474348 :
                  (hcnt == 2'd1) ? c_seq : {c_osh, l4, 7'd0, c_drop, nr16};
      hcnt <= hcnt + 1'b1;
      if (hcnt == 2'd0) st_frames <= st_frames + 1'b1;
    end else if (!out_afull && need_rh) begin
      out_push <= 1'b1;
      out_word <= {len16, b16};
      rh_done <= 1'b1;
    end else if (en) begin
      hcnt <= 2'd0; rh_done <= 1'b0;
      if (c_v && c_in) begin
        out_push <= 1'b1;
        out_word <= c_w;
        if (c_sat) st_sat <= st_sat + 1'b1;
      end
    end
    if (out_push) st_words <= st_words + 1'b1;
  end

  // =====================================================================
  // Detection spectrum: kernel [1 -4 6 -4 1]/16 over bins, |.|^2, average
  reg signed [FW-1:0] k_r [0:4];
  reg signed [FW-1:0] k_i [0:4];
  reg [L-1:0]         k_b = 0;
  integer ki_;
  initial for (ki_ = 0; ki_ < 5; ki_ = ki_ + 1) begin k_r[ki_] = 0; k_i[ki_] = 0; end
  always @(posedge clk) if (en) begin
    k_r[0] <= Y_r; k_i[0] <= Y_i; k_b <= yb;
    for (ki_ = 1; ki_ < 5; ki_ = ki_ + 1) begin k_r[ki_] <= k_r[ki_-1]; k_i[ki_] <= k_i[ki_-1]; end
  end
  // e0: partial sums (k0 + k4, k1 + k3, k2) of the kernel
  reg signed [FW:0]   e0_ar = 0, e0_ai = 0, e0_br = 0, e0_bi = 0;
  reg signed [FW-1:0] e0_cr = 0, e0_ci = 0;
  reg [L-1:0]         e0_b = 0;
  always @(posedge clk) if (en) begin
    e0_ar <= $signed(k_r[4]) + $signed(k_r[0]); e0_ai <= $signed(k_i[4]) + $signed(k_i[0]);
    e0_br <= $signed(k_r[3]) + $signed(k_r[1]); e0_bi <= $signed(k_i[3]) + $signed(k_i[1]);
    e0_cr <= k_r[2]; e0_ci <= k_i[2];
    e0_b <= k_b - 2'd2;
  end
  // kernel select (latched while no spectrum is being accumulated):
  //   sin^5: k0 - 4 k1 + 6 k2 - 4 k3 + k4   sin^3: 8 k2 - 4 (k1 + k3)   sin^1: 16 k2
  // all scaled by 1/16 in e1, so sin^3 = [-1 2 -1]/4 and sin^1 = [1]
  reg  [1:0]          kern_l = 2'd0;
  wire signed [FW+3:0] cv5_r = e0_ar - 4 * e0_br + 6 * $signed(e0_cr);
  wire signed [FW+3:0] cv5_i = e0_ai - 4 * e0_bi + 6 * $signed(e0_ci);
  wire signed [FW+3:0] cv3_r = 8 * $signed(e0_cr) - 4 * e0_br;
  wire signed [FW+3:0] cv3_i = 8 * $signed(e0_ci) - 4 * e0_bi;
  wire signed [FW+3:0] cv1_r = 16 * $signed(e0_cr);
  wire signed [FW+3:0] cv1_i = 16 * $signed(e0_ci);
  wire signed [FW+3:0] cv_r = (kern_l == 2'd0) ? cv5_r : (kern_l == 2'd1) ? cv3_r : cv1_r;
  wire signed [FW+3:0] cv_i = (kern_l == 2'd0) ? cv5_i : (kern_l == 2'd1) ? cv3_i : cv1_i;
  // e1: kernel output (centre bin k_b - 2)
  reg signed [FW-1:0] e1_r = 0, e1_i = 0;
  reg [L-1:0]         e1_b = 0;
  reg                 e1_t = 1'b0, e1_f = 1'b0;        // tagged for accumulation, first frame
  localparam A_WAIT = 2'd0, A_ACC = 2'd1, A_DRAIN = 2'd2, A_HOLD = 2'd3;
  reg [1:0]   ast = A_WAIT;
  reg [12:0]  fcnt = 0;
  reg [2:0]   dcnt = 0;
  wire [L-1:0] cb = e0_b;
  wire [12:0]  nfr = 13'd1 << ((cfg_avg_l > 4'd12) ? 4'd12 : cfg_avg_l);
  always @(posedge clk)
    if (ast == A_WAIT || ast == A_HOLD) kern_l <= cfg_kern;
  always @(posedge clk) begin
    if (rst) begin
      ast <= A_WAIT; spec_ready <= 1'b0; e1_t <= 1'b0;
    end else begin
      if (spec_release && ast == A_HOLD) begin ast <= A_WAIT; spec_ready <= 1'b0; end
      if (en) begin
        e1_r <= (cv_r + 8) >>> 4; e1_i <= (cv_i + 8) >>> 4;
        e1_b <= cb;
        e1_t <= 1'b0; e1_f <= 1'b0;
        case (ast)
          A_WAIT:
            if (started && cb == 0 && !(spec_release)) begin
              ast <= A_ACC; fcnt <= 0; e1_t <= 1'b1; e1_f <= 1'b1;
            end
          A_ACC:
            if (cb == 0) begin
              if (fcnt + 1'b1 == nfr) begin ast <= A_DRAIN; dcnt <= 0; end
              else begin fcnt <= fcnt + 1'b1; e1_t <= 1'b1; e1_f <= 1'b0; end
            end else begin
              e1_t <= 1'b1; e1_f <= (fcnt == 0);
            end
          A_DRAIN: begin
            dcnt <= dcnt + 1'b1;
            if (dcnt == 3'd7) begin ast <= A_HOLD; spec_ready <= 1'b1; spec_seq <= spec_seq + 1'b1; end
          end
          default: ;
        endcase
      end
    end
  end
  // e2: squares
  reg [2*FW-1:0] e2_pr = 0, e2_pi = 0;
  reg [L-1:0]    e2_b = 0;
  reg            e2_t = 1'b0, e2_f = 1'b0;
  always @(posedge clk) if (en) begin
    e2_pr <= e1_r * e1_r; e2_pi <= e1_i * e1_i;
    e2_b <= e1_b; e2_t <= e1_t; e2_f <= e1_f;
  end
  // e2s: sum
  reg  [2*FW:0]  e2s_s = 0;
  reg  [L-1:0]   e2s_b = 0;
  reg            e2s_t = 1'b0, e2s_f = 1'b0;
  always @(posedge clk) if (en) begin
    e2s_s <= {1'b0, e2_pr} + {1'b0, e2_pi};
    e2s_b <= e2_b; e2s_t <= e2_t; e2s_f <= e2_f;
  end
  // e3: >> psh, saturate 32; read accumulator
  wire [2*FW:0]  e2_sh = e2s_s >> cfg_psh;
  reg  [31:0]    e3_p = 32'd0;
  reg  [L-1:0]   e3_b = 0;
  reg            e3_t = 1'b0, e3_f = 1'b0;
  (* ram_style = "block" *)
  reg  [31:0]    spec [0:N-1];
  reg  [31:0]    spec_q = 32'd0;
  integer si_;
  initial for (si_ = 0; si_ < N; si_ = si_ + 1) spec[si_] = 32'd0;
  wire           hold = (ast == A_HOLD);
  wire [L-1:0]   sp_ra = hold ? spec_raddr : e2s_b;
  always @(posedge clk) if (en || hold) spec_q <= spec[sp_ra];
  assign spec_rdata = spec_q;
  always @(posedge clk) if (en) begin
    e3_p <= (|e2_sh[2*FW:32]) ? 32'hFFFFFFFF : e2_sh[31:0];
    e3_b <= e2s_b; e3_t <= e2s_t; e3_f <= e2s_f;
  end
  // e4: write
  wire [32:0] acc_sum = {1'b0, spec_q} + {1'b0, e3_p};
  always @(posedge clk)
    if (en && e3_t && !hold)
      spec[e3_b] <= e3_f ? e3_p : (acc_sum[32] ? 32'hFFFFFFFF : acc_sum[31:0]);
endmodule
