`timescale 1ns/1ps
module tb_fft;
  parameter L = 6;
  parameter NG = 3;           // grow stages first
  localparam N = 1 << L;
  localparam IW = 16, OW = IW + NG;
  function [2*L-1:0] modes(input integer d);
    integer k; begin modes = 0; for (k = 0; k < L; k = k + 1) modes[2*k +: 2] = (k < NG) ? 2'd0 : 2'd1; end
  endfunction
  reg clk = 0; always #5 clk = ~clk;
  reg en = 0;
  reg [IW-1:0] xr = 0, xi = 0; reg [L-1:0] xidx = 0;
  wire [OW-1:0] yr, yi; wire [L-1:0] yidx;
  sgc_fft_dit #(.L(L), .IW(IW), .STAGE_MODE(modes(0)), .INV(0), .OW(OW)) dut (
    .clk(clk), .en(en), .xr(xr), .xi(xi), .xidx(xidx), .yr(yr), .yi(yi), .yidx(yidx));
  integer fi, fo, n, k, r1, r2, NF, total;
  reg signed [31:0] a, b;
  initial begin
    fi = $fopen("fft_in.txt", "r"); fo = $fopen("fft_out.txt", "w");
    r1 = $fscanf(fi, "%d\n", NF);
    total = 0;
    while (total < (NF + 3) * N) begin
      @(negedge clk);
      if ($random % 3 != 0) begin
        if (total < NF * N) begin r1 = $fscanf(fi, "%d %d\n", a, b); xr = a; xi = b; end
        else begin xr = 0; xi = 0; end
        xidx = total % N; en = 1; total = total + 1;
      end else en = 0;
      @(posedge clk); #1;
      if (en) $fwrite(fo, "%0d %0d %0d\n", $signed(yr), $signed(yi), yidx);
    end
    $finish;
  end
endmodule
