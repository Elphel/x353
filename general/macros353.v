/*
** -----------------------------------------------------------------------------**
** macros353.v
**
** I/O pads related circuitry
**
** Copyright (C) 2002 Elphel, Inc
**
** -----------------------------------------------------------------------------**
**  This file is part of X353
**  X353 is free software - hardware description language (HDL) code.
** 
**  This program is free software: you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation, either version 3 of the License, or
**  (at your option) any later version.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.
** -----------------------------------------------------------------------------**
**
*/
// just make more convenient A[3:0] instead of 4 one-bit inputs
// TODO: Replace direct instances of SRL16 to imporve portability
module MSRL16 (Q, A, CLK, D);
    output Q;
    input  [3:0] A;
    input  CLK, D;
`ifdef SIMULATION
    SRL16_MOD #(.INVERT(1'b0)) i_q (.Q(Q), .A0(A[0]), .A1(A[1]), .A2(A[2]), .A3(A[3]), .CLK(CLK), .D(D));
`else    
    SRL16                      i_q (.Q(Q), .A0(A[0]), .A1(A[1]), .A2(A[2]), .A3(A[3]), .CLK(CLK), .D(D));
`endif    
endmodule


module MSRL16_1 (Q, A, CLK, D);
    output Q;
    input  [3:0] A;
    input  CLK, D;
`ifdef SIMULATION
    SRL16_MOD #(.INVERT(1'b1)) i_q (.Q(Q), .A0(A[0]), .A1(A[1]), .A2(A[2]), .A3(A[3]), .CLK(CLK), .D(D));
`else    
    SRL16_1                    i_q (.Q(Q), .A0(A[0]), .A1(A[1]), .A2(A[2]), .A3(A[3]), .CLK(CLK), .D(D));
`endif    
endmodule

module myRAM_WxD_D(D,WE,clk,AW,AR,QW,QR);
parameter DATA_WIDTH=16;
parameter DATA_DEPTH=4;
parameter DATA_2DEPTH=(1<<DATA_DEPTH)-1;
    input	 [DATA_WIDTH-1:0]	D;
    input				WE,clk;
    input	 [DATA_DEPTH-1:0]	AW;
    input	 [DATA_DEPTH-1:0]	AR;
    output [DATA_WIDTH-1:0]	QW;
    output [DATA_WIDTH-1:0]	QR;
    reg	 [DATA_WIDTH-1:0]	ram [0:DATA_2DEPTH];
    always @ (posedge clk) if (WE) ram[AW] <= D; 
    assign	QW= ram[AW];
    assign	QR= ram[AR];
endmodule

module myRAM_WxD_D_1(D,WE,clk,AW,AR,QW,QR);
parameter DATA_WIDTH=16;
parameter DATA_DEPTH=4;
parameter DATA_2DEPTH=(1<<DATA_DEPTH)-1;
    input	 [DATA_WIDTH-1:0]	D;
    input				WE,clk;
    input	 [DATA_DEPTH-1:0]	AW;
    input	 [DATA_DEPTH-1:0]	AR;
    output [DATA_WIDTH-1:0]	QW;
    output [DATA_WIDTH-1:0]	QR;
    reg	 [DATA_WIDTH-1:0]	ram [0:DATA_2DEPTH];
    always @ (negedge clk) if (WE) ram[AW] <= D; 
    assign	QW= ram[AW];
    assign	QR= ram[AR];
endmodule

// Fixing Xilinx SLR16_x
module SRL16_MOD #(
    parameter INIT = 16'h0000,
    parameter INVERT = 0 // *_1 - invert
) (
    output Q,
    input  A0,
    input  A1,
    input  A2,
    input  A3,
    input  CLK,
    input  D);


    reg  [15:0] data;
    wire        clk_;
    wire  [3:0] a = {A3, A2, A1, A0};


    assign Q = (data == 16'h0) ? 1'b0 :
               ((data == 16'hffff) ? 1'b1 : data[a]);
    assign clk_ = INVERT? (~CLK) : CLK;

    initial
    begin
          assign  data = INIT;
          while (clk_ === 1'b1 || clk_ === 1'bX) 
            #10; 
          deassign data;
    end


    always @(posedge clk_)
    begin
    {data[15:0]} <= #100 {data[14:0], D};
    end


endmodule
