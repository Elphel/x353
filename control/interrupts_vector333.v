/*
** -----------------------------------------------------------------------------**
** interrupts_vector333.v
**
** Interrupts controller
**
** Copyright (C) 2005-2008 Elphel, Inc
**
** -----------------------------------------------------------------------------**
**  This file is part of X333
**  X333 is free software - hardware description language (HDL) code.
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

// will support 16 priority (0 - highest) masked vector interrupts (pos. edge triggered), use 4 locations of address space,
// generate 32-bit data output (16MSBs - before mask, lower 16 bits - masked)

module interrupts_vector(sclk, // @negedge
                         pre_wen, // 1 cycle ahead of write data
                         pre_wa,  // [1:0] - write address:
                                  //   0 - reset selected interrupt bits
                                  //   1 - disable selected interrupt bits (mask)
                                  //   2 - enable selected interrupt bits
                                  //   3 - write vector number (bits [0:7], [11:8] - interrupt number (0..15)
                         di,      // [15:0] data in
                         do,      // [31:0] data out (bits {15:0} - masked interrupts, [31:0] - same before mask
                         irq,     // interrupt request (active high)
                         inta,    // interrupt acknowledge input (active low) 170ns long
                         irq_in,  // [15:0] input interrupt requests (posedge)
                         irqv);    // [7:0] interrupt vector (valid befor end on inta
    input         sclk;
    input         pre_wen;
    input   [1:0] pre_wa;
    input  [15:0] di;
    output [31:0] do;
    output        irq;
    input         inta;
    input  [15:0] irq_in;
    output  [7:0] irqv;
    
    wire   [15:0] irq_insa;
    reg    [15:0] irq_insb;
    reg    [15:0] irq_insc;  //single cycle sync interrupt request.
    wire   [15:0] irq_rst;
    reg           rst_rqs, en_rqs, dis_rqs, pre_set_irqv, set_irqv;
    reg    [ 3:0] irqv_a; // irq vectors table write address
    reg    [ 7:0] irqv_d; // irq vectors table write data
    reg    [15:0] rst_rq;
    reg    [15:0] dis_rq;
//    reg    [15:0] en_rq;
    wire   [15:0] en_rq;
    wire   [15:0] irqm;
    reg    [ 5:0] inta_s;  // inta sync to clock
//    reg    [15:0] irq_um;     // interrupts (before mask)
    wire   [15:0] irq_um;     // interrupts (before mask)
    reg    [15:0] irq_frz; // frozen masked irq-s (for the duration of the synchronized inta)
    wire   [15:0] irqp;    // one-hot irq (prioritized)
    wire   [ 3:0] irqn;    // encoded vector number;
    reg    [ 3:0] irqn_r;  // registered encoded vector number (not neded to be registered?
    wire   [15:0] rst_rq_inta; // reset interrupts from inta
    reg           pre_irq;
    reg    [15:0] did;     // did[15:0] delayed by 1 cycle
    wire          en_all_rq;   // for simulation
    
    
    assign        irq=pre_irq && inta;
    assign        do[31:0]={irq_um[15:0], irq_frz[15:0]};
    
    assign irqp[ 0] = irq_frz[ 0];
    assign irqp[ 1] = irq_frz[ 1] &&  !irq_frz[   0];
    assign irqp[ 2] = irq_frz[ 2] && !(|irq_frz[ 1:0]);
    assign irqp[ 3] = irq_frz[ 3] && !(|irq_frz[ 2:0]);
    assign irqp[ 4] = irq_frz[ 4] && !(|irq_frz[ 3:0]);
    assign irqp[ 5] = irq_frz[ 5] && !(|irq_frz[ 4:0]);
    assign irqp[ 6] = irq_frz[ 6] && !(|irq_frz[ 5:0]);
    assign irqp[ 7] = irq_frz[ 7] && !(|irq_frz[ 6:0]);
    assign irqp[ 8] = irq_frz[ 8] && !(|irq_frz[ 7:0]);
    assign irqp[ 9] = irq_frz[ 9] && !(|irq_frz[ 8:0]);
    assign irqp[10] = irq_frz[10] && !(|irq_frz[ 9:0]);
    assign irqp[11] = irq_frz[11] && !(|irq_frz[10:0]);
    assign irqp[12] = irq_frz[12] && !(|irq_frz[11:0]);
    assign irqp[13] = irq_frz[13] && !(|irq_frz[12:0]);
    assign irqp[14] = irq_frz[14] && !(|irq_frz[13:0]);
    assign irqp[15] = irq_frz[15] && !(|irq_frz[14:0]);
    
    assign irqn[0]=   irqp[1] | irqp[3] | irqp[ 5] | irqp[ 7] | irqp[ 9] | irqp[11] | irqp[13] | irqp[15];
    assign irqn[1]=   irqp[2] | irqp[3] | irqp[ 6] | irqp[ 7] | irqp[10] | irqp[11] | irqp[14] | irqp[15];
    assign irqn[2]=   irqp[4] | irqp[5] | irqp[ 6] | irqp[ 7] | irqp[12] | irqp[13] | irqp[14] | irqp[15];
    assign irqn[3]=   irqp[8] | irqp[9] | irqp[10] | irqp[11] | irqp[12] | irqp[13] | irqp[14] | irqp[15];
    
    assign rst_rq_inta[ 0]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h0); 
    assign rst_rq_inta[ 1]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h1); 
    assign rst_rq_inta[ 2]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h2); 
    assign rst_rq_inta[ 3]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h3); 
    assign rst_rq_inta[ 4]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h4); 
    assign rst_rq_inta[ 5]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h5); 
    assign rst_rq_inta[ 6]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h6); 
    assign rst_rq_inta[ 7]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h7); 
    assign rst_rq_inta[ 8]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h8); 
    assign rst_rq_inta[ 9]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'h9); 
    assign rst_rq_inta[10]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'ha); 
    assign rst_rq_inta[11]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'hb); 
    assign rst_rq_inta[12]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'hc); 
    assign rst_rq_inta[13]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'hd); 
    assign rst_rq_inta[14]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'he); 
    assign rst_rq_inta[15]= inta_s[1] && !inta_s[2] && (irqn_r[3:0] == 4'hf); 

    FDE_1 i_en_all_rq(.C(sclk), .CE(pre_wen), .D(1'b1), .Q(en_all_rq));  // initially disabled - till first write
    
    FDC  i_insa_0  (.C(irq_in[ 0]), .CLR(irq_insc[ 0]), .D(1'b1), .Q(irq_insa[ 0]));
    FDC  i_insa_1  (.C(irq_in[ 1]), .CLR(irq_insc[ 1]), .D(1'b1), .Q(irq_insa[ 1]));
    FDC  i_insa_2  (.C(irq_in[ 2]), .CLR(irq_insc[ 2]), .D(1'b1), .Q(irq_insa[ 2]));
    FDC  i_insa_3  (.C(irq_in[ 3]), .CLR(irq_insc[ 3]), .D(1'b1), .Q(irq_insa[ 3]));
    FDC  i_insa_4  (.C(irq_in[ 4]), .CLR(irq_insc[ 4]), .D(1'b1), .Q(irq_insa[ 4]));
    FDC  i_insa_5  (.C(irq_in[ 5]), .CLR(irq_insc[ 5]), .D(1'b1), .Q(irq_insa[ 5]));
    FDC  i_insa_6  (.C(irq_in[ 6]), .CLR(irq_insc[ 6]), .D(1'b1), .Q(irq_insa[ 6]));
    FDC  i_insa_7  (.C(irq_in[ 7]), .CLR(irq_insc[ 7]), .D(1'b1), .Q(irq_insa[ 7]));
    FDC  i_insa_8  (.C(irq_in[ 8]), .CLR(irq_insc[ 8]), .D(1'b1), .Q(irq_insa[ 8]));
    FDC  i_insa_9  (.C(irq_in[ 9]), .CLR(irq_insc[ 9]), .D(1'b1), .Q(irq_insa[ 9]));
    FDC  i_insa_10 (.C(irq_in[10]), .CLR(irq_insc[10]), .D(1'b1), .Q(irq_insa[10]));
    FDC  i_insa_11 (.C(irq_in[11]), .CLR(irq_insc[11]), .D(1'b1), .Q(irq_insa[11]));
    FDC  i_insa_12 (.C(irq_in[12]), .CLR(irq_insc[12]), .D(1'b1), .Q(irq_insa[12]));
    FDC  i_insa_13 (.C(irq_in[13]), .CLR(irq_insc[13]), .D(1'b1), .Q(irq_insa[13]));
    FDC  i_insa_14 (.C(irq_in[14]), .CLR(irq_insc[14]), .D(1'b1), .Q(irq_insa[14]));
    FDC  i_insa_15 (.C(irq_in[15]), .CLR(irq_insc[15]), .D(1'b1), .Q(irq_insa[15]));
    
    FD_1   i_irqm_0  (.C(sclk), .D(en_rq[ 0] || (!dis_rq[ 0] && irqm[ 0])), .Q(irqm[ 0]));
    FD_1   i_irqm_1  (.C(sclk), .D(en_rq[ 1] || (!dis_rq[ 1] && irqm[ 1])), .Q(irqm[ 1]));
    FD_1   i_irqm_2  (.C(sclk), .D(en_rq[ 2] || (!dis_rq[ 2] && irqm[ 2])), .Q(irqm[ 2]));
    FD_1   i_irqm_3  (.C(sclk), .D(en_rq[ 3] || (!dis_rq[ 3] && irqm[ 3])), .Q(irqm[ 3]));
    FD_1   i_irqm_4  (.C(sclk), .D(en_rq[ 4] || (!dis_rq[ 4] && irqm[ 4])), .Q(irqm[ 4]));
    FD_1   i_irqm_5  (.C(sclk), .D(en_rq[ 5] || (!dis_rq[ 5] && irqm[ 5])), .Q(irqm[ 5]));
    FD_1   i_irqm_6  (.C(sclk), .D(en_rq[ 6] || (!dis_rq[ 6] && irqm[ 6])), .Q(irqm[ 6]));
    FD_1   i_irqm_7  (.C(sclk), .D(en_rq[ 7] || (!dis_rq[ 7] && irqm[ 7])), .Q(irqm[ 7]));
    FD_1   i_irqm_8  (.C(sclk), .D(en_rq[ 8] || (!dis_rq[ 8] && irqm[ 8])), .Q(irqm[ 8]));
    FD_1   i_irqm_9  (.C(sclk), .D(en_rq[ 9] || (!dis_rq[ 9] && irqm[ 9])), .Q(irqm[ 9]));
    FD_1   i_irqm_10 (.C(sclk), .D(en_rq[10] || (!dis_rq[10] && irqm[10])), .Q(irqm[10]));
    FD_1   i_irqm_11 (.C(sclk), .D(en_rq[11] || (!dis_rq[11] && irqm[11])), .Q(irqm[11]));
    FD_1   i_irqm_12 (.C(sclk), .D(en_rq[12] || (!dis_rq[12] && irqm[12])), .Q(irqm[12]));
    FD_1   i_irqm_13 (.C(sclk), .D(en_rq[13] || (!dis_rq[13] && irqm[13])), .Q(irqm[13]));
    FD_1   i_irqm_14 (.C(sclk), .D(en_rq[14] || (!dis_rq[14] && irqm[14])), .Q(irqm[14]));
    FD_1   i_irqm_15 (.C(sclk), .D(en_rq[15] || (!dis_rq[15] && irqm[15])), .Q(irqm[15]));

    FD_1   i_en_rq_0   (.C(sclk), .D(en_rqs & did[ 0]), .Q(en_rq[ 0]));
    FD_1   i_en_rq_1   (.C(sclk), .D(en_rqs & did[ 1]), .Q(en_rq[ 1]));
    FD_1   i_en_rq_2   (.C(sclk), .D(en_rqs & did[ 2]), .Q(en_rq[ 2]));
    FD_1   i_en_rq_3   (.C(sclk), .D(en_rqs & did[ 3]), .Q(en_rq[ 3]));
    FD_1   i_en_rq_4   (.C(sclk), .D(en_rqs & did[ 4]), .Q(en_rq[ 4]));
    FD_1   i_en_rq_5   (.C(sclk), .D(en_rqs & did[ 5]), .Q(en_rq[ 5]));
    FD_1   i_en_rq_6   (.C(sclk), .D(en_rqs & did[ 6]), .Q(en_rq[ 6]));
    FD_1   i_en_rq_7   (.C(sclk), .D(en_rqs & did[ 7]), .Q(en_rq[ 7]));
    FD_1   i_en_rq_8   (.C(sclk), .D(en_rqs & did[ 8]), .Q(en_rq[ 8]));
    FD_1   i_en_rq_9   (.C(sclk), .D(en_rqs & did[ 9]), .Q(en_rq[ 9]));
    FD_1   i_en_rq_10  (.C(sclk), .D(en_rqs & did[10]), .Q(en_rq[10]));
    FD_1   i_en_rq_11  (.C(sclk), .D(en_rqs & did[11]), .Q(en_rq[11]));
    FD_1   i_en_rq_12  (.C(sclk), .D(en_rqs & did[12]), .Q(en_rq[12]));
    FD_1   i_en_rq_13  (.C(sclk), .D(en_rqs & did[13]), .Q(en_rq[13]));
    FD_1   i_en_rq_14  (.C(sclk), .D(en_rqs & did[14]), .Q(en_rq[14]));
    FD_1   i_en_rq_15  (.C(sclk), .D(en_rqs & did[15]), .Q(en_rq[15]));
    
    FD_1   i_irq_um_0    (.C(sclk), .D(~(rst_rq[ 0]) & (irq_insc[ 0] | irq_um[ 0])), .Q(irq_um[ 0]));
    FD_1   i_irq_um_1    (.C(sclk), .D(~(rst_rq[ 1]) & (irq_insc[ 1] | irq_um[ 1])), .Q(irq_um[ 1]));
    FD_1   i_irq_um_2    (.C(sclk), .D(~(rst_rq[ 2]) & (irq_insc[ 2] | irq_um[ 2])), .Q(irq_um[ 2]));
    FD_1   i_irq_um_3    (.C(sclk), .D(~(rst_rq[ 3]) & (irq_insc[ 3] | irq_um[ 3])), .Q(irq_um[ 3]));
    FD_1   i_irq_um_4    (.C(sclk), .D(~(rst_rq[ 4]) & (irq_insc[ 4] | irq_um[ 4])), .Q(irq_um[ 4]));
    FD_1   i_irq_um_5    (.C(sclk), .D(~(rst_rq[ 5]) & (irq_insc[ 5] | irq_um[ 5])), .Q(irq_um[ 5]));
    FD_1   i_irq_um_6    (.C(sclk), .D(~(rst_rq[ 6]) & (irq_insc[ 6] | irq_um[ 6])), .Q(irq_um[ 6]));
    FD_1   i_irq_um_7    (.C(sclk), .D(~(rst_rq[ 7]) & (irq_insc[ 7] | irq_um[ 7])), .Q(irq_um[ 7]));
    FD_1   i_irq_um_8    (.C(sclk), .D(~(rst_rq[ 8]) & (irq_insc[ 8] | irq_um[ 8])), .Q(irq_um[ 8]));
    FD_1   i_irq_um_9    (.C(sclk), .D(~(rst_rq[ 9]) & (irq_insc[ 9] | irq_um[ 9])), .Q(irq_um[ 9]));
    FD_1   i_irq_um_10   (.C(sclk), .D(~(rst_rq[10]) & (irq_insc[10] | irq_um[10])), .Q(irq_um[10]));
    FD_1   i_irq_um_11   (.C(sclk), .D(~(rst_rq[11]) & (irq_insc[11] | irq_um[11])), .Q(irq_um[11]));
    FD_1   i_irq_um_12   (.C(sclk), .D(~(rst_rq[12]) & (irq_insc[12] | irq_um[12])), .Q(irq_um[12]));
    FD_1   i_irq_um_13   (.C(sclk), .D(~(rst_rq[13]) & (irq_insc[13] | irq_um[13])), .Q(irq_um[13]));
    FD_1   i_irq_um_14   (.C(sclk), .D(~(rst_rq[14]) & (irq_insc[14] | irq_um[14])), .Q(irq_um[14]));
    FD_1   i_irq_um_15   (.C(sclk), .D(~(rst_rq[15]) & (irq_insc[15] | irq_um[15])), .Q(irq_um[15]));
    
    
    myRAM16X8D_1   i_vecttab (.D(irqv_d[7:0]), .WE(set_irqv), .clk(sclk), .AW(irqv_a[3:0]), .AR(irqn_r[3:0]), .QR(irqv[7:0]));
    
    always @ (negedge sclk) begin
	   if (pre_wen) did[15:0] <= di[15:0];
      if (en_all_rq) begin
        irq_insb[15:0] <= irq_insa[15:0];
        irq_insc[15:0] <= irq_insb[15:0] & (~irq_insc[15:0]);
      end else begin
        irq_insb[15:0] <= 16'h0;
        irq_insc[15:0] <= 16'h0;
      end  
      rst_rqs  <= pre_wen && (pre_wa[1:0] == 2'h0);
      dis_rqs   <= pre_wen && (pre_wa[1:0] == 2'h1);
      en_rqs  <= pre_wen && (pre_wa[1:0] == 2'h2);
      pre_set_irqv <= pre_wen && (pre_wa[1:0] == 2'h3);
      set_irqv <= pre_set_irqv;
      if (pre_set_irqv) irqv_a[3:0] <= did[11:8];      
      if (pre_set_irqv) irqv_d[7:0] <= did[ 7:0];
      rst_rq[15:0] <= ({15{rst_rqs}} & did[15:0]) | rst_rq_inta[15:0];
      dis_rq[15:0] <= ({15{dis_rqs}} & did[15:0]);
//      en_rq [15:0] <= ({15{ en_rqs}} & did[15:0]);
      
      inta_s[5:0] <= {inta_s[4:0], inta};
      if (inta_s[1]) irq_frz[15:0] <= irq_um[15:0] & irqm[15:0];
      
      irqn_r[3:0] <= irqn[3:0]; // will be invalid for a cycle - long combinatorial logic delays
      pre_irq <= |irq_frz[15:0] & inta_s[5];
      
  end    
    
endmodule
module myRAM16X8D_1(D,WE,clk,AW,AR,QW,QR);
   input  [7:0]  D;
   input          WE,clk;
   input  [3:0]  AW;
   input  [3:0]  AR;
   output [7:0]  QW;
   output [7:0]  QR;
   reg    [7:0]  ram [0:15];
   always @ (negedge clk) if (WE) ram[AW] <= D; 
   assign   QW= ram[AW];
   assign   QR= ram[AR];
endmodule
