/*
** -----------------------------------------------------------------------------**
** twelve_ios.v
**
** GPIO control
**
** Copyright (C) 2005-2007 Elphel, Inc
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
// update to eliminate need for a shadow register
// each pair of data bits at write cycle control the data and enable in the following way:
// bit 1 bit 0  dibit  enable data
//   0     0      0    - no change -
//   0     1      1      1      0
//   1     0      2      1      1
//   1     1      3      0      0


//Unified IO control for the 6 pins that are connected from the FPGA to the inter-board 16-pin connector
// those pins were controlled (in models 303, 313, 323 and earlier 333) by the control register, status was
// read through the status register.

// Now each pin will be controlled by 2 bits (data+enable), total 12 bits that will come from one of 4 sources 
// selected by bits [13:12] of the new control word:
// 0 - use bits [11:0] of the control word
// 1 - use channel A (USB)?
// 2 - use channel B (tbd)
// 3 - use channel C (tbd)
// Updating logic
// global enable signals (disabled channel will not compete for per-biot access)
// next 4 enable signals are controlled by bit pairs (0X - don't change, 10 - disable, 11 - enable)
// bit [25:24] - enable software bits (contolled by bits [23:0] (on at powerup)
// bit [27:26] - enable chn. A
// bit [29:28] - enable chn. B
// bit [31:30] - enable chn. C
// Enabled bits will be priority encoded (C - highest, software - lowest)
module twelve_ios      (sclk, // @negedge
                         pre_wen, // 1 cycle ahead of write data
                         di,      // [15:0] data in
                         io_do,   // [5:0] data to I/O pins
                         io_t,    // [5:0] tristate I/O pins
                         da,      // [5:0] data from port A (USB?)
                         da_en,   // [5:0] data enable from port A (USB?)
                         db,      // [5:0] data from port B
                         db_en,   // [5:0] data enable from port B
                         dc,      // [5:0] data from port C
                         dc_en);   // [5:0] data enable from port C
    input         sclk;
    input         pre_wen;
    input  [15:0] di;
    output [11:0] io_do;
    output [11:0] io_t;
    input  [11:0] da;
    input  [11:0] da_en;
    input  [11:0] db;
    input  [11:0] db_en;
    input  [11:0] dc;
    input  [11:0] dc_en;
    
//    wire   [23:0] cr;    // control register - reset at powerup
    wire   [11:0] ds;    // "software" data (programmed by lower 24 bits)
    wire   [11:0] ds_en;    // "software" data enable (programmed by lower 24 bits)
	 wire   [ 3:0] ch_en; // channel enable
    reg           pre_wen_d;
    reg           cr_wen;
    reg    [31:0] did;   // registered (dealyed by 1 clock) version of di[25:0]
    
    wire   [11:0] ds_en_m;
    wire   [11:0] da_en_m;
    wire   [11:0] db_en_m;
    wire   [11:0] dc_en_m;
	 
assign dc_en_m[11:0]= dc_en[11:0] & {12{ch_en[3]}};
assign db_en_m[11:0]= db_en[11:0] & {12{ch_en[2]}} & ~dc_en_m[11:0];
assign da_en_m[11:0]= da_en[11:0] & {12{ch_en[1]}} & ~dc_en_m[11:0] & ~db_en_m[11:0];
assign ds_en_m[11:0]= ds_en[11:0] & {12{ch_en[0]}} & ~dc_en_m[11:0] & ~db_en_m[11:0] & ~da_en_m[11:0];
assign io_do[11:0]=(dc_en_m[11:0] & dc[11:0]) |
                   (db_en_m[11:0] & db[11:0]) |
                   (da_en_m[11:0] & da[11:0]) |
                   (ds_en_m[11:0] & ds[11:0]);
assign io_t[11:0]=~(dc_en_m[11:0] | db_en_m[11:0] | da_en_m[11:0] | ds_en_m[11:0]);
//   0     0      0    - no change -
//   0     1      1      1      0
//   1     0      2      1      1
//   1     1      3      0      0

    FDE_1 i_ds_0     (.C(sclk), .CE(cr_wen & (did[ 0] | did[ 1])), .D(            ~did[ 0] ), .Q(ds[ 0]));
    FDE_1 i_ds_1     (.C(sclk), .CE(cr_wen & (did[ 2] | did[ 3])), .D(            ~did[ 2] ), .Q(ds[ 1]));
    FDE_1 i_ds_2     (.C(sclk), .CE(cr_wen & (did[ 4] | did[ 5])), .D(            ~did[ 4] ), .Q(ds[ 2]));
    FDE_1 i_ds_3     (.C(sclk), .CE(cr_wen & (did[ 6] | did[ 7])), .D(            ~did[ 6] ), .Q(ds[ 3]));
    FDE_1 i_ds_4     (.C(sclk), .CE(cr_wen & (did[ 8] | did[ 9])), .D(            ~did[ 8] ), .Q(ds[ 4]));
    FDE_1 i_ds_5     (.C(sclk), .CE(cr_wen & (did[10] | did[11])), .D(            ~did[10] ), .Q(ds[ 5]));
    FDE_1 i_ds_6     (.C(sclk), .CE(cr_wen & (did[12] | did[13])), .D(            ~did[12] ), .Q(ds[ 6]));
    FDE_1 i_ds_7     (.C(sclk), .CE(cr_wen & (did[14] | did[15])), .D(            ~did[14] ), .Q(ds[ 7]));
    FDE_1 i_ds_8     (.C(sclk), .CE(cr_wen & (did[16] | did[17])), .D(            ~did[16] ), .Q(ds[ 8]));
    FDE_1 i_ds_9     (.C(sclk), .CE(cr_wen & (did[18] | did[19])), .D(            ~did[18] ), .Q(ds[ 9]));
    FDE_1 i_ds_10    (.C(sclk), .CE(cr_wen & (did[20] | did[21])), .D(            ~did[20] ), .Q(ds[10]));
    FDE_1 i_ds_11    (.C(sclk), .CE(cr_wen & (did[22] | did[23])), .D(            ~did[22] ), .Q(ds[11]));
	 
    FDE_1 i_ds_en_0  (.C(sclk), .CE(cr_wen & (did[ 0] | did[ 1])), .D(~(did[ 1] &  did[ 0])), .Q(ds_en[ 0]));
    FDE_1 i_ds_en_1  (.C(sclk), .CE(cr_wen & (did[ 2] | did[ 3])), .D(~(did[ 3] &  did[ 2])), .Q(ds_en[ 1]));
    FDE_1 i_ds_en_2  (.C(sclk), .CE(cr_wen & (did[ 4] | did[ 5])), .D(~(did[ 5] &  did[ 4])), .Q(ds_en[ 2]));
    FDE_1 i_ds_en_3  (.C(sclk), .CE(cr_wen & (did[ 6] | did[ 7])), .D(~(did[ 7] &  did[ 6])), .Q(ds_en[ 3]));
    FDE_1 i_ds_en_4  (.C(sclk), .CE(cr_wen & (did[ 8] | did[ 9])), .D(~(did[ 9] &  did[ 8])), .Q(ds_en[ 4]));
    FDE_1 i_ds_en_5  (.C(sclk), .CE(cr_wen & (did[10] | did[11])), .D(~(did[11] &  did[10])), .Q(ds_en[ 5]));
    FDE_1 i_ds_en_6  (.C(sclk), .CE(cr_wen & (did[12] | did[13])), .D(~(did[13] &  did[12])), .Q(ds_en[ 6]));
    FDE_1 i_ds_en_7  (.C(sclk), .CE(cr_wen & (did[14] | did[15])), .D(~(did[15] &  did[14])), .Q(ds_en[ 7]));
    FDE_1 i_ds_en_8  (.C(sclk), .CE(cr_wen & (did[16] | did[17])), .D(~(did[17] &  did[16])), .Q(ds_en[ 8]));
    FDE_1 i_ds_en_9  (.C(sclk), .CE(cr_wen & (did[18] | did[19])), .D(~(did[19] &  did[18])), .Q(ds_en[ 9]));
    FDE_1 i_ds_en_10 (.C(sclk), .CE(cr_wen & (did[20] | did[21])), .D(~(did[21] &  did[20])), .Q(ds_en[10]));
    FDE_1 i_ds_en_11 (.C(sclk), .CE(cr_wen & (did[22] | did[23])), .D(~(did[23] &  did[22])), .Q(ds_en[11]));

    FDE_1 #(.INIT(1'b1)) i_ch_en_0  (.C(sclk), .CE(cr_wen & did[25]), .D(did[24]), .Q(ch_en[ 0]));
    FDE_1 #(.INIT(1'b0)) i_ch_en_1  (.C(sclk), .CE(cr_wen & did[27]), .D(did[26]), .Q(ch_en[ 1]));
    FDE_1 #(.INIT(1'b0)) i_ch_en_2  (.C(sclk), .CE(cr_wen & did[29]), .D(did[28]), .Q(ch_en[ 2]));
    FDE_1 #(.INIT(1'b0)) i_ch_en_3  (.C(sclk), .CE(cr_wen & did[31]), .D(did[30]), .Q(ch_en[ 3]));
 
    always @ (negedge sclk) begin
      pre_wen_d  <= pre_wen;
      cr_wen <=pre_wen_d;
		if (pre_wen)   did[15: 0] <= di[15:0];
		if (pre_wen_d) did[31:16] <= di[15:0];
    end
endmodule
