//
// SPDX-License-Identifier: MIT
//
// sgc_top.v -- satellite-downlink channelizer for the ADALM-Pluto style HDL.
//
// Sits between the RX decimator (rx_fir_decimator) and util_cpack2.
//   mode 0 (default): the samples pass unchanged -> stock Pluto behaviour
//   mode 1: the RX DMA receives the channelizer frames (see sgc_core.v)
//           instead of the samples; enable only voltage0/voltage1 on the host.
// AXI4-Lite registers (64 KB, base 0x43C00000 in the Pluto address map):
//   0x0000 R  ID       0x53474348 "SGCH"
//   0x0004 R  VERSION  {major, minor}
//   0x0008 R  CONFIG   {L, RUNS_L, IN_W, 0}
//   0x000C R  CORE_HZ  core clock (Hz, nominal)
//   0x0010 RW CTRL     [0] run  [1] DMA mode (1 = frames)  [2] emit words  [31] soft reset (W)
//   0x0014 RW PARAM    [3:0] AVG (log2 frames / spectrum)  [11:8] PSH  [19:16] OSH
//                      [25:24] KERN detection kernel 0 sin^5 (v1.0), 1 sin^3, 2 sin^1 (v1.1)
//   0x0018 R  STATUS   [0] spectrum ready  [1] swap pending  [2] active bank
//   0x001C RW SPEC     R: spectra completed   W: release (start the next average)
//   0x0020 R  FRAMES   output frames
//   0x0024 R  DROPS    input frames skipped
//   0x0028 R  SAT      saturated output samples
//   0x002C R  WORDS    output words
//   0x0030 RW BANK     W [0]: swap request   R: [0] pending [1] active bank
//   0x0034 RW NRUNS    runs in the shadow bank
//   0x0038 R  SAMPLES  input samples
//   0x003C R  IN_OVF   input samples lost (input FIFO full)
//   0x0040 R  RATE     input samples in the last 2^24 core clocks
//   0x4000 W  GAIN     shadow gains: word a = bins 4a..4a+3 (byte k = bin 4a+k), 255 = unity
//   0x6000 W  RUNS     shadow run r: {len[15:0], start[15:0]}
//   0x8000 R  SPEC     power spectrum bin b at 0x8000 + 4b (valid while ready)

module sgc_top #(
  parameter L       = 13,
  parameter IN_W    = 12,
  parameter RUNS_L  = 8,
  parameter CORE_HZ = 100000000
) (
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input             s_axi_aclk,
  input             s_axi_aresetn,
  input      [15:0] s_axi_awaddr,
  input      [2:0]  s_axi_awprot,
  input             s_axi_awvalid,
  output            s_axi_awready,
  input      [31:0] s_axi_wdata,
  input      [3:0]  s_axi_wstrb,
  input             s_axi_wvalid,
  output            s_axi_wready,
  output     [1:0]  s_axi_bresp,
  output reg        s_axi_bvalid = 1'b0,
  input             s_axi_bready,
  input      [15:0] s_axi_araddr,
  input      [2:0]  s_axi_arprot,
  input             s_axi_arvalid,
  output            s_axi_arready,
  output reg [31:0] s_axi_rdata = 32'd0,
  output     [1:0]  s_axi_rresp,
  output reg        s_axi_rvalid = 1'b0,
  input             s_axi_rready,

  // ADC side (axi_ad9361 l_clk domain)
  input             adc_clk,
  input             adc_rst,
  input             in_valid_0,
  input             in_enable_0,
  input      [15:0] in_data_0,
  input             in_valid_1,
  input             in_enable_1,
  input      [15:0] in_data_1,
  input             in_enable_2,
  input      [15:0] in_data_2,
  input             in_enable_3,
  input      [15:0] in_data_3,

  output            out_valid,
  output            out_enable_0,
  output     [15:0] out_data_0,
  output            out_enable_1,
  output     [15:0] out_data_1,
  output            out_enable_2,
  output     [15:0] out_data_2,
  output            out_enable_3,
  output     [15:0] out_data_3
);
  localparam N = 1 << L;
  localparam [15:0] VMAJ = 16'd1, VMIN = 16'd1;
  wire clk = s_axi_aclk;

  // ---------------------------------------------------------------------
  // registers
  reg        c_run = 1'b0, c_mode = 1'b0, c_emit = 1'b1;
  reg [3:0]  c_avg = 4'd10, c_psh = 4'd0, c_osh = 4'd0;
  reg [1:0]  c_kern = 2'd0;
  reg [RUNS_L:0] c_nruns = 0;
  reg        soft_rst = 1'b0, swap_req = 1'b0, spec_rel = 1'b0;
  reg        g_we = 1'b0, r_we = 1'b0;
  reg [L-3:0] g_addr = 0;
  reg [RUNS_L-1:0] r_addr = 0;
  reg [31:0] wd = 32'd0;

  // AXI write: accept address and data together
  wire aw_ok = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
  assign s_axi_awready = aw_ok;
  assign s_axi_wready  = aw_ok;
  assign s_axi_bresp   = 2'b00;
  always @(posedge clk) begin
    soft_rst <= 1'b0; swap_req <= 1'b0; spec_rel <= 1'b0; g_we <= 1'b0; r_we <= 1'b0;
    if (!s_axi_aresetn) begin
      s_axi_bvalid <= 1'b0;
      c_run <= 1'b0; c_mode <= 1'b0; c_emit <= 1'b1;
      c_avg <= 4'd10; c_psh <= 4'd0; c_osh <= 4'd0; c_kern <= 2'd0; c_nruns <= 0;
    end else begin
      if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
      if (aw_ok) begin
        s_axi_bvalid <= 1'b1;
        wd <= s_axi_wdata;
        casez (s_axi_awaddr)
          16'h0010: begin c_run <= s_axi_wdata[0]; c_mode <= s_axi_wdata[1]; c_emit <= s_axi_wdata[2];
                          soft_rst <= s_axi_wdata[31]; end
          16'h0014: begin c_avg <= s_axi_wdata[3:0]; c_psh <= s_axi_wdata[11:8]; c_osh <= s_axi_wdata[19:16];
                          c_kern <= s_axi_wdata[25:24]; end
          16'h001C: spec_rel <= 1'b1;
          16'h0030: swap_req <= s_axi_wdata[0];
          16'h0034: c_nruns <= s_axi_wdata[RUNS_L:0];
          16'b010?_????_????_??00: begin g_we <= 1'b1; g_addr <= s_axi_awaddr[L:2]; end
          16'b0110_????_????_??00: begin r_we <= 1'b1; r_addr <= s_axi_awaddr[RUNS_L+1:2]; end
          default: ;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------
  // resets: core stream reset = !run | soft reset | ADC reset (synchronised)
  (* ASYNC_REG = "TRUE" *) reg ar1 = 1'b1, ar2 = 1'b1;
  always @(posedge clk) begin ar1 <= adc_rst; ar2 <= ar1; end
  reg [3:0] rst_cnt = 4'hF;
  always @(posedge clk)
    if (!s_axi_aresetn || !c_run || soft_rst || ar2) rst_cnt <= 4'hF;
    else if (rst_cnt != 0) rst_cnt <= rst_cnt - 1'b1;
  wire core_rst = (rst_cnt != 0);
  (* ASYNC_REG = "TRUE" *) reg cr1 = 1'b1, cr2 = 1'b1;
  always @(posedge adc_clk) begin cr1 <= core_rst; cr2 <= cr1; end
  wire adc_side_rst = cr2 | adc_rst;
  (* ASYNC_REG = "TRUE" *) reg m1 = 1'b0, m2 = 1'b0;
  always @(posedge adc_clk) begin m1 <= c_mode; m2 <= m1; end

  // ---------------------------------------------------------------------
  // input: ADC domain -> core domain
  function [IN_W-1:0] sat_in(input [15:0] v);
    reg signed [15:0] s;
    begin
      s = v;
      if (s > $signed(16'd1 << (IN_W - 1)) - 1)       sat_in = {1'b0, {(IN_W-1){1'b1}}};
      else if (s < -$signed(16'd1 << (IN_W - 1)))     sat_in = {1'b1, {(IN_W-1){1'b0}}};
      else                                            sat_in = s[IN_W-1:0];
    end
  endfunction
  wire ififo_full, ififo_empty;
  wire [2*IN_W-1:0] ififo_q;
  reg  [31:0] in_ovf_a = 32'd0;
  wire        in_we = in_valid_0 && !adc_side_rst;
  always @(posedge adc_clk) if (in_we && ififo_full) in_ovf_a <= in_ovf_a + 1'b1;
  sgc_afifo #(.DW(2*IN_W), .AW(4)) ififo (
    .wclk(adc_clk), .wrst(adc_side_rst), .we(in_we), .wdata({sat_in(in_data_0), sat_in(in_data_1)}),
    .wfull(ififo_full), .wlevel(),
    .rclk(clk), .rrst(core_rst), .re(!ififo_empty), .rdata(ififo_q), .rempty(ififo_empty));
  reg ivld = 1'b0;
  always @(posedge clk) ivld <= !ififo_empty && !core_rst;

  // ---------------------------------------------------------------------
  // core
  wire        o_push;
  wire [31:0] o_word;
  wire [9:0]  ofifo_lvl;
  reg         o_afull = 1'b0;
  wire        sp_ready, sw_pend, act;
  wire [31:0] sp_rdata, sp_seq, st_frames, st_drops, st_sat, st_words, st_samples;
  reg  [L-1:0] sp_raddr = 0;
  always @(posedge clk) o_afull <= (ofifo_lvl >= 10'd480);
  sgc_core #(.L(L), .IN_W(IN_W), .RUNS_L(RUNS_L)) core (
    .clk(clk), .rst(core_rst),
    .in_valid(ivld), .in_i(ififo_q[2*IN_W-1:IN_W]), .in_q(ififo_q[IN_W-1:0]),
    .out_push(o_push), .out_word(o_word), .out_afull(o_afull),
    .cfg_avg_l(c_avg), .cfg_psh(c_psh), .cfg_osh(c_osh), .cfg_kern(c_kern), .cfg_out_en(c_emit),
    .g_we(g_we), .g_addr(g_addr), .g_wdata(wd),
    .r_we(r_we), .r_addr(r_addr), .r_start(wd[15:0]), .r_len(wd[31:16]), .r_nruns(c_nruns),
    .swap_req(swap_req), .swap_pending(sw_pend), .act_bank(act),
    .spec_ready(sp_ready), .spec_release(spec_rel), .spec_raddr(sp_raddr), .spec_rdata(sp_rdata),
    .spec_seq(sp_seq),
    .st_frames(st_frames), .st_drops(st_drops), .st_sat(st_sat), .st_words(st_words),
    .st_samples(st_samples));

  // ---------------------------------------------------------------------
  // output: core domain -> ADC domain, at most one word every 2 adc clocks
  wire        ofifo_empty;
  wire [31:0] ofifo_q;
  reg         slot = 1'b0, ov = 1'b0;
  wire        o_re = m2 && slot && !ofifo_empty;
  always @(posedge adc_clk) begin
    slot <= ~slot;
    ov <= o_re;
  end
  sgc_afifo #(.DW(32), .AW(9)) ofifo (
    .wclk(clk), .wrst(core_rst), .we(o_push), .wdata(o_word), .wfull(), .wlevel(ofifo_lvl),
    .rclk(adc_clk), .rrst(adc_side_rst), .re(o_re), .rdata(ofifo_q), .rempty(ofifo_empty));

  assign out_valid    = m2 ? ov : in_valid_0;
  assign out_data_0   = m2 ? ofifo_q[15:0]  : in_data_0;
  assign out_data_1   = m2 ? ofifo_q[31:16] : in_data_1;
  assign out_data_2   = m2 ? 16'd0 : in_data_2;
  assign out_data_3   = m2 ? 16'd0 : in_data_3;
  assign out_enable_0 = in_enable_0;
  assign out_enable_1 = in_enable_1;
  assign out_enable_2 = in_enable_2;
  assign out_enable_3 = in_enable_3;

  // ---------------------------------------------------------------------
  // input rate (samples per 2^24 core clocks)
  reg [23:0] rt_cnt = 0;
  reg [31:0] rt_s0 = 0, rt_val = 0;
  always @(posedge clk) begin
    rt_cnt <= rt_cnt + 1'b1;
    if (rt_cnt == 0) begin rt_val <= st_samples - rt_s0; rt_s0 <= st_samples; end
  end
  (* ASYNC_REG = "TRUE" *) reg [31:0] iovf1 = 0, iovf2 = 0;   // slow counter, sampled
  always @(posedge clk) begin iovf1 <= in_ovf_a; iovf2 <= iovf1; end

  // ---------------------------------------------------------------------
  // AXI read
  reg [15:0] ra = 0;
  reg [1:0]  rph = 2'd0;
  reg        rbusy = 1'b0;
  assign s_axi_arready = !rbusy && !s_axi_rvalid;
  assign s_axi_rresp   = 2'b00;
  wire [7:0] l8 = L, rl8 = RUNS_L, iw8 = IN_W;
  always @(posedge clk) begin
    if (!s_axi_aresetn) begin
      rbusy <= 1'b0; s_axi_rvalid <= 1'b0; rph <= 2'd0;
    end else begin
      if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
      if (s_axi_arvalid && s_axi_arready) begin
        ra <= s_axi_araddr; rbusy <= 1'b1; rph <= 2'd0;
        sp_raddr <= s_axi_araddr[L+1:2];
      end else if (rbusy) begin
        rph <= rph + 1'b1;
        if (rph == 2'd2) begin
          rbusy <= 1'b0; s_axi_rvalid <= 1'b1;
          if (ra[15]) s_axi_rdata <= sp_rdata;
          else case (ra[7:0])
            8'h00: s_axi_rdata <= 32'h53474348;
            8'h04: s_axi_rdata <= {VMAJ, VMIN};
            8'h08: s_axi_rdata <= {l8, rl8, iw8, 8'd0};
            8'h0C: s_axi_rdata <= CORE_HZ;
            8'h10: s_axi_rdata <= {29'd0, c_emit, c_mode, c_run};
            8'h14: s_axi_rdata <= {6'd0, c_kern, 4'd0, c_osh, 4'd0, c_psh, 4'd0, c_avg};
            8'h18: s_axi_rdata <= {29'd0, act, sw_pend, sp_ready};
            8'h1C: s_axi_rdata <= sp_seq;
            8'h20: s_axi_rdata <= st_frames;
            8'h24: s_axi_rdata <= st_drops;
            8'h28: s_axi_rdata <= st_sat;
            8'h2C: s_axi_rdata <= st_words;
            8'h30: s_axi_rdata <= {30'd0, act, sw_pend};
            8'h34: s_axi_rdata <= c_nruns;
            8'h38: s_axi_rdata <= st_samples;
            8'h3C: s_axi_rdata <= iovf2;
            8'h40: s_axi_rdata <= rt_val;
            default: s_axi_rdata <= 32'd0;
          endcase
        end
      end
    end
  end
endmodule
