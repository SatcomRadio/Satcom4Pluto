//
// SPDX-License-Identifier: MIT
//
// sgc_afifo.v -- asynchronous FIFO (gray-code pointers, 2-FF synchronisers).
// Read data appears one rclk after re (registered). wlevel is the write-side
// fill level (pessimistic, may lag reads by a few wclk).

module sgc_afifo #(
  parameter DW = 32,
  parameter AW = 4
) (
  input               wclk,
  input               wrst,
  input               we,
  input  [DW-1:0]     wdata,
  output              wfull,
  output reg [AW:0]   wlevel = 0,

  input               rclk,
  input               rrst,
  input               re,
  output reg [DW-1:0] rdata = 0,
  output              rempty
);
  localparam D = 1 << AW;
  reg [DW-1:0] mem [0:D-1];

  function [AW:0] b2g(input [AW:0] b); b2g = b ^ (b >> 1); endfunction
  function [AW:0] g2b(input [AW:0] g);
    integer k;
    begin
      g2b[AW] = g[AW];
      for (k = AW - 1; k >= 0; k = k - 1) g2b[k] = g2b[k+1] ^ g[k];
    end
  endfunction

  // pointers (declared before use: xvlog requires it)
  reg  [AW:0] wb = 0, wg = 0;
  reg  [AW:0] rb = 0, rg = 0;

  // write side
  (* ASYNC_REG = "TRUE" *) reg [AW:0] rg_w1 = 0, rg_w2 = 0;
  wire [AW:0] rb_w = g2b(rg_w2);
  assign wfull = (wg == {~rg_w2[AW:AW-1], rg_w2[AW-2:0]});
  always @(posedge wclk) begin
    rg_w1 <= rg; rg_w2 <= rg_w1;
    if (wrst) begin
      wb <= 0; wg <= 0; wlevel <= 0;
    end else begin
      if (we && !wfull) begin
        mem[wb[AW-1:0]] <= wdata;
        wb <= wb + 1'b1;
        wg <= b2g(wb + 1'b1);
      end
      wlevel <= wb - rb_w;
    end
  end

  // read side
  (* ASYNC_REG = "TRUE" *) reg [AW:0] wg_r1 = 0, wg_r2 = 0;
  assign rempty = (rg == wg_r2);
  always @(posedge rclk) begin
    wg_r1 <= wg; wg_r2 <= wg_r1;
    if (rrst) begin
      rb <= 0; rg <= 0;
    end else if (re && !rempty) begin
      rdata <= mem[rb[AW-1:0]];
      rb <= rb + 1'b1;
      rg <= b2g(rb + 1'b1);
    end
  end
endmodule

// ---------------------------------------------------------------------------
// Simple dual-port block RAM (one clock): write port, registered read port
// with read enable. Written in the Vivado template style so it maps to RAMB.
module sgc_sdpram #(
  parameter DW = 24,
  parameter AW = 14
) (
  input               clk,
  input               we,
  input  [AW-1:0]     waddr,
  input  [DW-1:0]     wdata,
  input               re,
  input  [AW-1:0]     raddr,
  output reg [DW-1:0] rdata = 0
);
  (* ram_style = "block" *) reg [DW-1:0] mem [0:(1<<AW)-1];
  always @(posedge clk) if (we) mem[waddr] <= wdata;
  always @(posedge clk) if (re) rdata <= mem[raddr];
endmodule
