//
// SPDX-License-Identifier: MIT
//
// sgc_axi_stub.v -- stand-in for the Pluto's PL I2C controller (axi_iic at
// 0x41600000), which this board has no pins for. The firmware device tree
// still probes it, so it must answer: writes are ignored, the status
// register (0x104) reads "TX and RX FIFO empty, bus idle", all else 0.

module sgc_axi_stub (
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
  output            irq
);
  assign irq = 1'b0;
  wire aw_ok = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
  assign s_axi_awready = aw_ok;
  assign s_axi_wready  = aw_ok;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_arready = !s_axi_rvalid;
  assign s_axi_rresp   = 2'b00;
  always @(posedge s_axi_aclk)
    if (!s_axi_aresetn) begin
      s_axi_bvalid <= 1'b0; s_axi_rvalid <= 1'b0;
    end else begin
      if (aw_ok) s_axi_bvalid <= 1'b1;
      else if (s_axi_bready) s_axi_bvalid <= 1'b0;
      if (s_axi_arvalid && !s_axi_rvalid) begin
        s_axi_rvalid <= 1'b1;
        s_axi_rdata <= (s_axi_araddr[11:0] == 12'h104) ? 32'h000000C0 : 32'd0;
      end else if (s_axi_rready) s_axi_rvalid <= 1'b0;
    end
endmodule
