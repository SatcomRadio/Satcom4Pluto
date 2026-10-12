`timescale 1ns/1ps
module tb_top;
  parameter L = 8;
  parameter RATIO = 2;          // adc clocks per sample
  localparam N = 1 << L;
  reg clk = 0;  always #5 clk = ~clk;          // 100 MHz core
  reg aclk = 0; always #8.138 aclk = ~aclk;    // 61.44 MHz l_clk
  reg aresetn = 0;
  reg [15:0] awaddr = 0, araddr = 0; reg awvalid = 0, wvalid = 0, arvalid = 0; reg [31:0] wdata = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [31:0] rdata; wire [1:0] bresp, rresp;
  reg in_valid = 0; reg [15:0] di = 0, dq = 0;
  wire ov; wire [15:0] o0, o1, o2, o3; wire e0, e1, e2, e3;
  sgc_top #(.L(L), .RUNS_L(8)) dut (
    .s_axi_aclk(clk), .s_axi_aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(1'b1),
    .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(1'b1),
    .adc_clk(aclk), .adc_rst(1'b0),
    .in_valid_0(in_valid), .in_enable_0(1'b1), .in_data_0(di), .in_valid_1(in_valid), .in_enable_1(1'b1), .in_data_1(dq),
    .in_enable_2(1'b0), .in_data_2(16'd0), .in_enable_3(1'b0), .in_data_3(16'd0),
    .out_valid(ov), .out_enable_0(e0), .out_data_0(o0), .out_enable_1(e1), .out_data_1(o1),
    .out_enable_2(e2), .out_data_2(o2), .out_enable_3(e3), .out_data_3(o3));

  task axw(input [15:0] a, input [31:0] d);
    begin
      @(negedge clk); awaddr = a; wdata = d; awvalid = 1; wvalid = 1;
      @(posedge clk); while (!(awready)) @(posedge clk);
      @(negedge clk); awvalid = 0; wvalid = 0;
      @(posedge clk);
    end
  endtask
  task axr(input [15:0] a, output [31:0] d);
    begin
      @(negedge clk); araddr = a; arvalid = 1;
      @(posedge clk); while (!arready) @(posedge clk);
      @(negedge clk); arvalid = 0;
      while (!rvalid) @(posedge clk);
      d = rdata; @(posedge clk);
    end
  endtask

  // input samples from file
  integer fin, fout, fspec, fcmd, r, ns, k, cnt, t;
  reg signed [31:0] a, b;
  reg [31:0] v;
  reg [8*16-1:0] cmd;
  initial begin
    fin = $fopen("in.txt", "r"); r = $fscanf(fin, "%d\n", ns);
    cnt = 0;
    forever begin
      @(posedge aclk); #0.5;
      t = t + 1;
      if (t >= RATIO && cnt < ns) begin
        t = 0; r = $fscanf(fin, "%d %d\n", a, b); di = a; dq = b; in_valid = 1; cnt = cnt + 1;
      end else in_valid = 0;
    end
  end
  initial t = 0;
  integer fdb; reg dump = 0; initial begin dump = $test$plusargs("dump"); fdb = $fopen("fftin.txt","w"); end
  always @(posedge clk) if (dump && dut.core.en) $fwrite(fdb, "%0d %0d %0d\n", $signed(dut.core.r3_i), $signed(dut.core.r3_q), dut.core.r3_p);
  // capture DMA words
  initial begin
    fout = $fopen("out.txt", "w");
    forever begin @(posedge aclk); if (ov && dut.m2) $fwrite(fout, "%08x\n", {o1, o0}); end
  end

  // control script: lines "w ADDR DATA", "r ADDR", "wait N" (core clocks), "spec FILE", "end"
  reg [31:0] addr, data, i;
  initial begin
    fcmd = $fopen("cmd.txt", "r");
    fspec = $fopen("spec.txt", "w");
    repeat (20) @(posedge clk); aresetn = 1; repeat (5) @(posedge clk);
    forever begin
      r = $fscanf(fcmd, "%s", cmd);
      if (cmd == "w") begin r = $fscanf(fcmd, "%h %h\n", addr, data); axw(addr, data); end
      else if (cmd == "wait") begin r = $fscanf(fcmd, "%d\n", data); repeat (data) @(posedge clk); end
      else if (cmd == "waitspec") begin
        v = 0; while (!v[0]) begin axr(16'h0018, v); repeat (50) @(posedge clk); end
        for (i = 0; i < N; i = i + 1) begin axr(16'h8000 + 4*i, v); $fwrite(fspec, "%0d\n", v); end
        $fwrite(fspec, "#\n"); $fflush(fspec);
      end
      else if (cmd == "status") begin
        for (i = 0; i < 17; i = i + 1) begin axr(4*i, v); $display("reg %02x = %08x (%0d)", 4*i, v, v); end
      end
      else if (cmd == "end") begin $fflush(fout); $finish; end
    end
  end
endmodule
