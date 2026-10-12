//
// SPDX-License-Identifier: MIT
//
// sgc_fft.v -- streaming radix-2 DIT FFT for the Pluto channelizer (sgc_core).
// Derived from the B210 sat_gate sg_fft.v blocks, reworked for a small Zynq:
//   * complex multiply with 3 multipliers (Gauss), 3 DSP48 per stage
//   * quarter-wave twiddle ROM (one dual-port ROM per stage, D/2 entries)
//   * stage 1 (twiddles 1 and -/+j) has no multiplier
// Every register advances only on `en`, so the transform can be stalled at
// any point. Input in bit-reversed order (xidx = stream position 0..N-1),
// output in natural order (yidx = bin).
//
// STAGE_MODE: 2 bits per stage (stage 0 in [1:0]): 0 grow 1 bit,
// 1 scale by 1/2 (round, saturate), 2 keep width and saturate.
// Twiddles are 18-bit Q16 (1.0 = 65536).

module sgc_delay #(
  parameter W = 32,
  parameter D = 1
) (
  input              clk,
  input              en,
  input  [W-1:0]     din,
  output [W-1:0]     dout
);
  generate if (D == 1) begin : g_reg
    reg [W-1:0] q = {W{1'b0}};
    always @(posedge clk) if (en) q <= din;
    assign dout = q;
  end else if (D < 256) begin : g_mem
    localparam AW = $clog2(D);
    (* ram_style = "distributed" *)
    reg [W-1:0] mem [0:D-1];
    reg [W-1:0] q = {W{1'b0}};
    reg [AW-1:0] wp = {AW{1'b0}};
    wire [AW-1:0] rp = wp + 1'b1;          // D is a power of two
    integer i;
    initial for (i = 0; i < D; i = i + 1) mem[i] = {W{1'b0}};
    always @(posedge clk)
      if (en) begin
        mem[wp] <= din;
        q       <= mem[rp];
        wp      <= wp + 1'b1;
      end
    assign dout = q;
  end else begin : g_bram
    // block RAM with its output register: read 2 ahead, 2 register stages
    localparam AW = $clog2(D);
    (* ram_style = "block" *)
    reg [W-1:0] mem [0:D-1];
    reg [W-1:0] q1 = {W{1'b0}}, q = {W{1'b0}};
    reg [AW-1:0] wp = {AW{1'b0}};
    wire [AW-1:0] rp = wp + 2'd2;
    integer i;
    initial for (i = 0; i < D; i = i + 1) mem[i] = {W{1'b0}};
    always @(posedge clk)
      if (en) begin
        mem[wp] <= din;
        q1      <= mem[rp];
        q       <= q1;
        wp      <= wp + 1'b1;
      end
    assign dout = q;
  end endgenerate
endmodule

// ---------------------------------------------------------------------------
// Twiddle for index m (0..D-1): angle th = pi*m/D, outputs cos(th), sin(th)
// in Q16, one en-step after addr. Quarter-wave table T[k] = sin(pi*k/D),
// k = 0..D/2-1 (T[D/2] = 1.0 is implied).
module sgc_twiddle #(
  parameter D = 4
) (
  input                         clk,
  input                         en,
  input      [$clog2(D)-1:0]    addr,
  output signed [17:0]          c,
  output signed [17:0]          s
);
  localparam AW = $clog2(D);
  localparam Q  = D / 2;
  localparam QW = (AW > 1) ? AW - 1 : 1;
  (* rom_style = (Q >= 512) ? "block" : "distributed" *)
  reg [16:0] rom [0:Q-1];
  integer i, v;
  initial
    for (i = 0; i < Q; i = i + 1) begin
      v = $rtoi($floor($sin(3.14159265358979323846 * i / D) * 65536.0 + 0.5));
      rom[i] = v[16:0];
    end
  wire          hi = addr[AW-1];                    // m >= D/2
  wire [AW-1:0] mm = {1'b0, addr[AW-2:0]};          // m mod D/2
  wire [AW-1:0] qm = Q - mm;                        // D/2 - m'
  // sin: m < D/2 -> T[m], else T[D/2 - m']
  // cos: m < D/2 -> T[D/2 - m], else -T[m']
  wire [AW-1:0] a_s = hi ? qm : mm;
  wire [AW-1:0] a_c = hi ? mm : qm;
  reg  [16:0] rs = 0, rc = 0;
  reg         one_s = 1'b0, one_c = 1'b0, neg_c = 1'b0;
  always @(posedge clk) if (en) begin
    rs <= rom[a_s[QW-1:0]];
    rc <= rom[a_c[QW-1:0]];
    one_s <= (a_s == Q);
    one_c <= (a_c == Q);
    neg_c <= hi;
  end
  wire signed [17:0] sv = one_s ? 18'sd65536 : $signed({1'b0, rs});
  wire signed [17:0] cv = one_c ? 18'sd65536 : $signed({1'b0, rc});
  assign s = sv;
  assign c = neg_c ? -cv : cv;
endmodule

// ---------------------------------------------------------------------------
// y = x * (c -/+ j s), 3 multipliers, 4 en-steps latency, rounded (Q16) and
// saturated to W bits. INV = 0: (c - j s) (forward), INV = 1: (c + j s).
module sgc_cmul3 #(
  parameter W   = 22,
  parameter INV = 0
) (
  input                 clk,
  input                 en,
  input  signed [W-1:0] xr,
  input  signed [W-1:0] xi,
  input  signed [17:0]  c,
  input  signed [17:0]  s,
  output reg [W-1:0]    yr = 0,
  output reg [W-1:0]    yi = 0
);
  localparam PW = W + 19;
  reg signed [W:0]    a_sum = 0;          // xr + xi
  reg signed [W-1:0]  a_r = 0, a_i = 0;
  reg signed [17:0]   a_c = 0, a_m = 0, a_p = 0;
  reg signed [PW-1:0] m1 = 0, m2 = 0, m3 = 0;
  always @(posedge clk) if (en) begin
    a_sum <= xr + xi;
    a_r <= xr; a_i <= xi;
    a_c <= c;
    // forward: re = t - xi(c - s), im = t - xr(c + s)
    // inverse: re = t - xi(c + s), im = t - xr(c - s)
    a_m <= (INV == 0) ? c - s : c + s;
    a_p <= (INV == 0) ? c + s : c - s;
  end
  always @(posedge clk) if (en) begin
    m1 <= a_c * a_sum;
    m2 <= a_i * a_m;
    m3 <= a_r * a_p;
  end
  reg signed [PW-1:0] dr = 0, di = 0;
  always @(posedge clk) if (en) begin
    dr <= m1 - m2 + (1 <<< 15);
    di <= m1 - m3 + (1 <<< 15);
  end
  always @(posedge clk) if (en) begin
    yr <= sat16(dr);
    yi <= sat16(di);
  end
  // >> 16, saturated to W bits
  function [W-1:0] sat16(input signed [PW-1:0] t);
    reg signed [PW-17:0] q;
    begin
      q = t >>> 16;
      if (q > $signed({1'b0, {(W-1){1'b1}}}))       sat16 = {1'b0, {(W-1){1'b1}}};
      else if (q < $signed({1'b1, {(W-1){1'b0}}}))  sat16 = {1'b1, {(W-1){1'b0}}};
      else                                         sat16 = q[W-1:0];
    end
  endfunction
endmodule

// ---------------------------------------------------------------------------
// Butterfly output word-length handling (combinational), IW+1 -> OW bits.
module sgc_wl #(
  parameter IW   = 24,
  parameter MODE = 0       // 0 grow (OW = IW+1), 1 scale 1/2 (OW = IW), 2 saturate (OW = IW)
) (
  input  signed [IW:0]           v,
  output [((MODE == 0) ? IW : IW - 1):0] o
);
  generate
    if (MODE == 0) begin : g_grow
      assign o = v;
    end else if (MODE == 1) begin : g_scale
      wire signed [IW+1:0] t = {v[IW], v} + 1'b1;      // round half up
      wire signed [IW:0]   q = t[IW+1:1];
      assign o = (q > $signed({2'b00, {(IW-1){1'b1}}})) ? {1'b0, {(IW-1){1'b1}}} : q[IW-1:0];
    end else begin : g_sat
      assign o = (v > $signed({2'b00, {(IW-1){1'b1}}})) ? {1'b0, {(IW-1){1'b1}}} :
                 (v < $signed({2'b11, {(IW-1){1'b0}}})) ? {1'b1, {(IW-1){1'b0}}} : v[IW-1:0];
    end
  endgenerate
endmodule

// ---------------------------------------------------------------------------
// DIT stage S of an N = 2^L FFT, D = 2^S. Latency: (D > 1 ? 4 : 0) + 1 en-steps.
module sgc_dit_stage #(
  parameter L    = 13,
  parameter S    = 0,
  parameter IW   = 16,
  parameter MODE = 0,
  parameter INV  = 0,
  parameter OW   = (MODE == 0) ? IW + 1 : IW,
  parameter XW   = L                // index width (>= L; upper bits: frame count)
) (
  input              clk,
  input              en,
  input  [IW-1:0]    xr,
  input  [IW-1:0]    xi,
  input  [XW-1:0]    xidx,
  output [OW-1:0]    yr,
  output [OW-1:0]    yi,
  output [XW-1:0]    yidx
);
  localparam D = 1 << S;

  // twiddle the second half of each 2D block before the butterfly
  wire [IW-1:0] br, bi;
  wire [XW-1:0] bx;
  generate if (D == 1) begin : g_nomul
    assign br = xr; assign bi = xi; assign bx = xidx;
  end else if (D == 2) begin : g_j
    // twiddles: m = 0 -> 1, m = 1 -> -j (forward) / +j (inverse)
    reg [IW-1:0] r0 = 0, i0 = 0, r1 = 0, i1 = 0, r2 = 0, i2 = 0, r3 = 0, i3 = 0;
    reg [XW-1:0] x0 = 0, x1 = 0, x2 = 0, x3 = 0;
    wire         rot = xidx[S] & xidx[0];
    wire signed [IW:0] nr = -$signed(xr), ni = -$signed(xi);
    // -j*(a + jb) = b - ja ; +j*(a + jb) = -b + ja (saturate the negation)
    function [IW-1:0] satn(input signed [IW:0] v);
      satn = (v > $signed({2'b00, {(IW-1){1'b1}}})) ? {1'b0, {(IW-1){1'b1}}} : v[IW-1:0];
    endfunction
    always @(posedge clk) if (en) begin
      if (!rot) begin r0 <= xr; i0 <= xi; end
      else if (INV == 0) begin r0 <= xi; i0 <= satn(nr); end
      else begin r0 <= satn(ni); i0 <= xr; end
      x0 <= xidx;
      r1 <= r0; i1 <= i0; x1 <= x0;
      r2 <= r1; i2 <= i1; x2 <= x1;
      r3 <= r2; i3 <= i2; x3 <= x2;
    end
    assign br = r3; assign bi = i3; assign bx = x3;
  end else begin : g_mul
    wire signed [17:0] rc, rs;
    sgc_twiddle #(.D(D)) rom (.clk(clk), .en(en), .addr(xidx[S-1:0]), .c(rc), .s(rs));
    reg [IW-1:0] r0 = 0, i0 = 0;
    reg          c0 = 1'b0;
    reg [XW-1:0] x0 = 0, x1 = 0, x2 = 0, x3 = 0, x4 = 0;
    always @(posedge clk) if (en) begin
      r0 <= xr; i0 <= xi; c0 <= xidx[S]; x0 <= xidx;
      x1 <= x0; x2 <= x1; x3 <= x2; x4 <= x3;
    end
    wire signed [17:0] c = c0 ? rc : 18'sd65536;
    wire signed [17:0] s = c0 ? rs : 18'sd0;
    sgc_cmul3 #(.W(IW), .INV(INV)) mul (.clk(clk), .en(en), .xr(r0), .xi(i0), .c(c), .s(s), .yr(br), .yi(bi));
    assign bx = x4;
  end endgenerate

  wire               ctrl = bx[S];
  wire signed [IW:0] bre = $signed(br), bie = $signed(bi);
  wire [2*IW+1:0]    h;
  wire signed [IW:0] hr = h[2*IW+1:IW+1], hi = h[IW:0];
  wire [2*IW+1:0]    push = ctrl ? {hr - bre, hi - bie} : {bre, bie};
  wire signed [IW:0] or_ = ctrl ? hr + bre : hr;
  wire signed [IW:0] oi_ = ctrl ? hi + bie : hi;

  sgc_delay #(.W(2*IW+2), .D(D)) dl (.clk(clk), .en(en), .din(push), .dout(h));

  wire [OW-1:0] wr_, wi_;
  sgc_wl #(.IW(IW), .MODE(MODE)) wlr (.v(or_), .o(wr_));
  sgc_wl #(.IW(IW), .MODE(MODE)) wli (.v(oi_), .o(wi_));

  reg [OW-1:0] yr_r = 0, yi_r = 0;
  reg [XW-1:0] yx_r = 0;
  always @(posedge clk) if (en) begin yr_r <= wr_; yi_r <= wi_; yx_r <= bx - D; end
  assign yr = yr_r; assign yi = yi_r; assign yidx = yx_r;
endmodule

// ---------------------------------------------------------------------------
module sgc_fft_dit #(
  parameter L          = 13,
  parameter IW         = 16,
  parameter [2*L-1:0] STAGE_MODE = 0,
  parameter INV        = 0,
  parameter OW         = 22,         // IW + number of grow stages
  parameter XW         = L
) (
  input              clk,
  input              en,
  input  [IW-1:0]    xr,
  input  [IW-1:0]    xi,
  input  [XW-1:0]    xidx,
  output [OW-1:0]    yr,
  output [OW-1:0]    yi,
  output [XW-1:0]    yidx
);
  localparam MW = IW + L;
  function integer wid(input integer s);
    integer k;
    begin
      wid = IW;
      for (k = 0; k < s; k = k + 1) if (STAGE_MODE[2*k +: 2] == 2'd0) wid = wid + 1;
    end
  endfunction
  wire signed [MW-1:0] dr [0:L];
  wire signed [MW-1:0] di [0:L];
  wire [XW-1:0]        dx [0:L];
  assign dr[0] = $signed(xr); assign di[0] = $signed(xi); assign dx[0] = xidx;
  genvar s;
  generate for (s = 0; s < L; s = s + 1) begin : st
    localparam WI = wid(s), WO = wid(s + 1);
    wire signed [WO-1:0] yr_s, yi_s;
    sgc_dit_stage #(.L(L), .S(s), .IW(WI), .MODE(STAGE_MODE[2*s +: 2]), .INV(INV), .OW(WO), .XW(XW)) u (
      .clk(clk), .en(en), .xr(dr[s][WI-1:0]), .xi(di[s][WI-1:0]), .xidx(dx[s]),
      .yr(yr_s), .yi(yi_s), .yidx(dx[s+1]));
    assign dr[s+1] = yr_s; assign di[s+1] = yi_s;
  end endgenerate
  assign yr = dr[L][OW-1:0]; assign yi = di[L][OW-1:0]; assign yidx = dx[L];
endmodule
