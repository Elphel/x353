/*
** -----------------------------------------------------------------------------**
** clkios353.v
**
** I/O pads related circuitry
**
** Copyright (C) 2002-2006 Elphel, Inc
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
// Some placement constraints are in this file
module dcm333(
      sclk,   // input global clock, 120MHz, phase=0
      SDCLK,   // positive clock to SDRAM
      SDNCLK,  // negative clock to SDRAM
      sdcl_fb,
      xclk,    // 60MHz for compressor
      phsel,   // additional phase control for SDRAM CLK 
      dcm_rst, // reset DCM phase
      dcm_incdec,  // variable phase control to adjust SDCLK so read DQS is aligned with sclk90/sclk270
      dcm_en,
      dcm_clk,
      dcm_done,
      locked,  // dcm locked
      status   // dcm status (bit 1 - dcm clkin stopped)
      );
  input sclk;
  output xclk;
  output SDCLK, SDNCLK;
  input [1:0] phsel;
  input       sdcl_fb;
  input dcm_rst, dcm_incdec, dcm_en, dcm_clk;
  output dcm_done;
  
  output [7:0] status; // dcm status (bit 1 - dcm clkin stopped)
  output       locked; // dcm locked

  wire isdclk0, isdclk90, isdclk180, isdclk270;
  wire   ixclk;
  wire  gsdclk; //used only for the feedback
  wire  isdclk;
  reg   dcm_done;
  wire  dcm_done_dcm; // single-cycle
  assign isdclk=phsel[1]? (phsel[0]?isdclk270:isdclk180):(phsel[0]?isdclk90:isdclk0);

 FD   i_ixclk  (.C(sclk), .D(!ixclk), .Q(ixclk));
 BUFG i_xclk   (.I(ixclk), .O(xclk));

// second - adjustable DCM. Will be adjusted so read DQS (dependent on SDCLK) will be aligned with sclk90/270
// maybe will need some delay as there is DLL in SDRAM and responce may be slow. 
DCM i_dcm2(
    .CLKIN (sclk),
    .CLKFB (isdclk90),
    .RST (dcm_rst),
    .PSEN (dcm_en),
    .PSINCDEC (dcm_incdec),
    .PSCLK (dcm_clk),.DSSEN (1'b0),
//    .CLK0 (isdclk0),
//    .CLK90 (isdclk90),
//    .CLK180 (isdclk180),
//    .CLK270 (isdclk270),
    .CLK0 (isdclk90),
    .CLK90 (isdclk180),
    .CLK180 (isdclk270),
    .CLK270 (isdclk0),
    .CLKDV (),
    .CLK2X (),
    .CLK2X180 (),
    .CLKFX (),
    .CLKFX180 (),
    .STATUS (status[7:0]),
    .LOCKED (locked),
    .PSDONE (dcm_done_dcm));
// s-ynthesis attribute loc of i_dcm2 is "DCM_X1Y1"
// synthesis attribute CLK_FEEDBACK of i_dcm2 is "1X"
// synthesis attribute CLKIN_DIVIDE_BY_2 of i_dcm2 is "FALSE"
// synthesis attribute CLKIN_PERIOD of i_dcm2 is 8.33333
// synthesis attribute CLKOUT_PHASE_SHIFT of i_dcm2 is "VARIABLE"
// synthesis attribute DESKEW_ADJUST of i_dcm2 is "SYSTEM_SYNCHRONOUS"
// synthesis attribute DLL_FREQUENCY_MODE of i_dcm2 is "LOW"
// synthesis attribute DUTY_CYCLE_CORRECTION of i_dcm2 is "TRUE"

// put here default phase shift ....

// synthesis attribute PHASE_SHIFT of i_dcm2 is 0
// synthesis translate_off
 defparam i_dcm2.CLK_FEEDBACK="1X";
 defparam i_dcm2.CLKIN_DIVIDE_BY_2="FALSE";
 defparam i_dcm2.CLKIN_PERIOD=8.33333;
 defparam i_dcm2.CLKOUT_PHASE_SHIFT="VARIABLE";
 defparam i_dcm2.DESKEW_ADJUST="SYSTEM_SYNCHRONOUS";
 defparam i_dcm2.DLL_FREQUENCY_MODE="LOW";
 defparam i_dcm2.DUTY_CYCLE_CORRECTION="TRUE";
 defparam i_dcm2.PHASE_SHIFT=0;
// synthesis translate_on
// BUFG    i_gsdclk (.I(isdclk90), .O(gsdclk));

 OBUFDS  i_SDCLK  (.O(SDCLK),.OB(SDNCLK),.I(isdclk));
// OBUFDS  i_SDCLK  (.O(SDNCLK),.OB(SDCLK),.I(!isdclk));
// make dcm_done behave as dcm_ready
   always @ (posedge dcm_clk  or posedge dcm_rst)
     if (dcm_rst) dcm_done  <= 1'b1;
     else if (dcm_en) dcm_done <=1'b0;
     else if (dcm_done_dcm) dcm_done <=1'b1;


endmodule






module clockios353(
      CLK0,    // input clock pad - 120MHz
      sclk0,   // global clock, 120MHz, phase=0  (addresses, commands should be strobed at neg edge)
      /*sclk90,*/  // global clock, 120MHz, phase=90 (strobe data write to sdram)
      sclk180, // global clock, 120MHz, phase=180 (just to generate DQS :-( )
      sclk270, // global clock, 120MHz, phase=270 (strobe data write to sdram)
      iclk0,  //before BUFG
      dcmrst,  //reset dcm
      locked,  // dcm locked
      status   // dcm status (bit 1 - dcm clkin stopped)
      );
  input CLK0;
  output sclk0,/*sclk90,*/sclk270,sclk180;
  output iclk0;

  input        dcmrst; //reset dcm
  output [7:0] status; // dcm status (bit 1 - dcm clkin stopped)
  output       locked; // dcm locked
  
  wire  iclk0;
  wire  isclk0, /*isclk90,*/ isclk270, isclk180;

  IBUFG i_iclk0 (.I(CLK0), .O(iclk0));
// DCM - just 4 phases out
DCM i_dcm1(
    .CLKIN (iclk0),
    .CLKFB (sclk0),
    .RST (dcmrst), .PSEN (1'b0),.PSINCDEC (1'b0), .PSCLK (1'b0),.DSSEN (1'b0),
    .CLK0 (isclk0),
    .CLK90 (/*isclk90*/),
    .CLK180 (isclk180),
    .CLK270 (isclk270),
    .CLKDV (),
    .CLK2X (),
    .CLK2X180 (),
    .CLKFX (),
    .CLKFX180 (),
    .STATUS (status[7:0]),
    .LOCKED (locked),
    .PSDONE ());
// s- ynthesis attribute loc of i_dcm1 is "DCM_X0Y0"
// synthesis attribute CLK_FEEDBACK of i_dcm1 is "1X"
// synthesis attribute CLKIN_DIVIDE_BY_2 of i_dcm1 is "FALSE"
// synthesis attribute CLKIN_PERIOD of i_dcm1 is 8.33333
// synthesis attribute DESKEW_ADJUST of i_dcm1 is "SYSTEM_SYNCHRONOUS"
// synthesis attribute DLL_FREQUENCY_MODE of i_dcm1 is "LOW"
// synthesis attribute DUTY_CYCLE_CORRECTION of i_dcm1 is "TRUE"
// synthesis translate_off
 defparam i_dcm1.CLK_FEEDBACK="1X";
 defparam i_dcm1.CLKIN_DIVIDE_BY_2="FALSE";
 defparam i_dcm1.CLKIN_PERIOD=8.33333;
 defparam i_dcm1.DESKEW_ADJUST="SYSTEM_SYNCHRONOUS";
 defparam i_dcm1.DLL_FREQUENCY_MODE="LOW";
 defparam i_dcm1.DUTY_CYCLE_CORRECTION="TRUE";
// synthesis translate_on
 BUFG i_sclk0  (.I(isclk0),.  O(sclk0));
// s-ynthesis attribute loc of i_sclk0 is   "BUFGMUX0"
/* BUFG i_sclk90 (.I(isclk90), .O(sclk90)); */
// s-ynthesis attribute loc of i_sclk90 is  "BUFGMUX1"
 BUFG i_sclk180(.I(isclk180),.O(sclk180));
// s-ynthesis attribute loc of i_sclk180 is  "BUFGMUX2"
 BUFG i_sclk270(.I(isclk270),.O(sclk270));
// s-ynthesis attribute loc of i_sclk270 is "BUFGMUX3"
endmodule

/*
module clock_pclk( clk0,			// global clock (phase =0)
                   CLK1,         // external input clock
						 pclk,			// global clock, sensor pixel rate
						 clk_en,			// enable clock output to sensor
						 pclk_src,// [3:0] - source fror the pclk
						 dclk);			// clock output to sensor (sync to pclk)
  input CLK1,clk0,clk_en;
  input [3:0] pclk_src;
  output pclk,dclk;

  wire	ipclk,dclk,pclk;
 
   reg clk_en_r;

  IBUF i_iclk1 (.I(CLK1), .O(iclk1));

// and global clock (divided by 1, 1.5,2,...,8)
pclk_cntrl i_pclk_cntrl(.ext_clk(iclk1),  // external clock, may be stopped (will wait for up to 16 clk periods?)
                        .clk(clk0),      // always running global clock
				            .div(pclk_src[3:0]), // 0:      q=ext_clk
	                                          // 1:      q=clk - temporary made clk/1.5 - not to fry MI1300
							                        // 2:      q=clk/1.5
                                             //...0'hf: q=clk/8
				            .q(ipclk));          // output clock

  always @ (posedge pclk) clk_en_r<=clk_en;
  assign dclk=!clk_en_r || !ipclk;
  BUFG	i_pclk (.I(ipclk), .O(pclk));
endmodule












// switches between external clock (no divisor)
// and global clock (divided by 1, 1.5,2,...,8)
module pclk_cntrl(ext_clk,  // external clock, may be stopped (will wait for up to 16 clk periods?)
                  clk,      // always running global clock
                  div,      // 0:      q=ext_clk
                            // 1:      q=clk - temporary made clk/1.5 - not to fry MI1300
                            // 2:      q=clk/1.5
                            //...0'hf: q=clk/8
			         q);       // output clock
  input ext_clk;
  input clk;
  input [3:0] div;
  output q;

  reg       ena;
  reg       enb;
//  reg       clksel;

  wire  [3:0] div_p;
  wire       blank;
  wire       copy_div,blank_off;
  wire       a_on, a_off0,  b_on, b_off;
  reg	[3:0] da0; // from a_start to a_on
  reg	[3:0] da1; // from a_on to a_off
  reg	[3:0] db0; // from a_start to a_on
  reg	[3:0] db1; // from a_on to a_off
  reg        a_start;
  reg        b_start;
  reg        a_off;
  reg       a_singleon; // a stays on for 1 cycle;
  reg        use_b;
  wire a,b,ab;
   FDE	i_div_p0 (.C(clk),.CE(copy_div),.D(div[0]),.Q(div_p[0]));
   FDE	i_div_p1 (.C(clk),.CE(copy_div),.D(div[1]),.Q(div_p[1]));
   FDE	i_div_p2 (.C(clk),.CE(copy_div),.D(div[2]),.Q(div_p[2]));
   FDE	i_div_p3 (.C(clk),.CE(copy_div),.D(div[3]),.Q(div_p[3]));

  always @ (posedge enb or posedge ext_clk) begin
    if (enb) ena <=1'b0;
    else ena <= (div[3:0]==4'b0);
  end

  always @ (posedge clk) begin
    enb <= (div_p[3:0]!=4'b0);
    a_off <= a_singleon?a_on:a_off0;
    a_start <= blank_off || (!blank && (a_singleon?a_on:a_off0));
  end
  always @ (negedge clk) begin 
    b_start <= a_start;
    use_b <= (div_p[3:0]!=4'h0) && (div_p[1:0]!=2'h3);
  end
   FD		i_blank (.C(clk),.D((div[3:0] != div_p[3:0]) || (blank && !blank_off)),.Q(blank)); // add always blank if (div==0)? no!
	SRL16 i_copy_div (.Q(copy_div),.D((div[3:0] != div_p[3:0]) && !blank), .CLK(clk),   .A0(1'b1),  .A1(1'b1), .A2(1'b1), .A3(1'b1));	// dly=max (16)
	SRL16 i_blank_off(.Q(blank_off),.D(copy_div), .CLK(clk),   .A0(1'b1),  .A1(1'b0), .A2(1'b0), .A3(1'b0));	// 2

	SRL16   i_a_on  (.Q(a_on),  .D(a_start), .CLK(clk),   .A0(da0[0]),  .A1(da0[1]), .A2(da0[2]), .A3(da0[3]));
	SRL16   i_a_off0(.Q(a_off0),.D(a_on),   .CLK(clk),   .A0(da1[0]),  .A1(da1[1]), .A2(da1[2]), .A3(da1[3]));
	SRL16_1 i_b_on  (.Q(b_on),  .D(b_start), .CLK(clk),   .A0(db0[0]),  .A1(db0[1]), .A2(db0[2]), .A3(db0[3]));
	SRL16_1 i_b_off (.Q(b_off), .D(b_on),   .CLK(clk),   .A0(db1[0]),  .A1(db1[1]), .A2(db1[2]), .A3(db1[3]));

   FD		i_a (.C(clk),.D(enb && (a_on || (a && !a_off))),.Q(a));
   FD_1	i_b (.C(clk),.D(use_b && (b_on || (b && !b_off))),.Q(b));
	assign ab= ~(a ^ b);
   assign q= (!ena || ext_clk) && (!enb || ab);

  always @ (posedge clk) case (div_p[3:0])
    4'h0: begin  da0<=4'h0;  da1<=4'h0;  a_singleon<=1'b0; end // any, not used
//    4'h1: begin  da0<=4'h0;  da1<=4'h0;  a_singleon<=1'b1; end
    4'h1: begin  da0<=4'h0;  da1<=4'h0;  a_singleon<=1'b0; end // for safety - not higher than clk0/1.5 (50MHz for 75 MHz)
    4'h2: begin  da0<=4'h0;  da1<=4'h0;  a_singleon<=1'b0; end
    4'h3: begin  da0<=4'h0;  da1<=4'h0;  a_singleon<=1'b1; end
    4'h4: begin  da0<=4'h3;  da1<=4'h0;  a_singleon<=1'b1; end
    4'h5: begin  da0<=4'h2;  da1<=4'h1;  a_singleon<=1'b0; end
    4'h6: begin  da0<=4'h1;  da1<=4'h3;  a_singleon<=1'b0; end
    4'h7: begin  da0<=4'h1;  da1<=4'h0;  a_singleon<=1'b0; end
    4'h8: begin  da0<=4'h6;  da1<=4'h0;  a_singleon<=1'b0; end
    4'h9: begin  da0<=4'h4;  da1<=4'h3;  a_singleon<=1'b0; end
    4'ha: begin  da0<=4'h2;  da1<=4'h6;  a_singleon<=1'b0; end
    4'hb: begin  da0<=4'h2;  da1<=4'h1;  a_singleon<=1'b0; end
    4'hc: begin  da0<=4'h9;  da1<=4'h1;  a_singleon<=1'b0; end
    4'hd: begin  da0<=4'h6;  da1<=4'h5;  a_singleon<=1'b0; end
    4'he: begin  da0<=4'h3;  da1<=4'h9;  a_singleon<=1'b0; end
    4'hf: begin  da0<=4'h3;  da1<=4'h2;  a_singleon<=1'b0; end
  endcase

  always @ (negedge clk) case (div_p[3:0])
    4'h0: begin  db0<=4'h0;  db1<=4'h0;  end
//    4'h1: begin  db0<=4'h1;  db1<=4'h0;  end
    4'h1: begin  db0<=4'h0;  db1<=4'h0;  end // for safety - not higher than clk0/1.5 (50MHz for 75 MHz)
    4'h2: begin  db0<=4'h0;  db1<=4'h0;  end
    4'h3: begin  db0<=4'h0;  db1<=4'h0;  end
    4'h4: begin  db0<=4'h0;  db1<=4'h0;  end
    4'h5: begin  db0<=4'h0;  db1<=4'h2;  end
    4'h6: begin  db0<=4'h2;  db1<=4'h1;  end
    4'h7: begin  db0<=4'h0;  db1<=4'h0;  end
    4'h8: begin  db0<=4'h1;  db1<=4'h1;  end
    4'h9: begin  db0<=4'h1;  db1<=4'h4;  end
    4'ha: begin  db0<=4'h4;  db1<=4'h2;  end
    4'hb: begin  db0<=4'h0;  db1<=4'h0;  end
    4'hc: begin  db0<=4'h2;  db1<=4'h2;  end
    4'hd: begin  db0<=4'h2;  db1<=4'h6;  end
    4'he: begin  db0<=4'h6;  db1<=4'h3;  end
    4'hf: begin  db0<=4'h0;  db1<=4'h0;  end
  endcase
endmodule

*/
