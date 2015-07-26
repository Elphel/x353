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
//  wire  gsdclk; //used only for the feedback
  wire  isdclk;
  reg   dcm_done;
  wire  dcm_done_dcm; // single-cycle
  assign isdclk=phsel[1]? (phsel[0]?isdclk270:isdclk180):(phsel[0]?isdclk90:isdclk0);

 FD   i_ixclk  (.C(sclk), .D(!ixclk), .Q(ixclk));
 BUFG i_xclk   (.I(ixclk), .O(xclk));

// second - adjustable DCM. Will be adjusted so read DQS (dependent on SDCLK) will be aligned with sclk90/270
// maybe will need some delay as there is DLL in SDRAM and responce may be slow. 
DCM #(
     .CLKIN_DIVIDE_BY_2     ("FALSE"),
     .CLKIN_PERIOD          (8.33333),
     .CLK_FEEDBACK          ("1X"),
     .DESKEW_ADJUST         ("SYSTEM_SYNCHRONOUS"),
     .DLL_FREQUENCY_MODE    ("LOW"),
     .DUTY_CYCLE_CORRECTION ("TRUE"),
     .PHASE_SHIFT           (0),
     .CLKOUT_PHASE_SHIFT    ("VARIABLE")
 ) i_dcm2(
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
DCM #(
     .CLKIN_DIVIDE_BY_2("FALSE"),
     .CLKIN_PERIOD(8.33333),
     .CLK_FEEDBACK("1X"),
     .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"),
     .DLL_FREQUENCY_MODE("LOW"),
     .DUTY_CYCLE_CORRECTION("TRUE")
 ) i_dcm1(
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
 BUFG i_sclk0  (.I(isclk0),.  O(sclk0));
// s-ynthesis attribute loc of i_sclk0 is   "BUFGMUX0"
/* BUFG i_sclk90 (.I(isclk90), .O(sclk90)); */
// s-ynthesis attribute loc of i_sclk90 is  "BUFGMUX1"
 BUFG i_sclk180(.I(isclk180),.O(sclk180));
// s-ynthesis attribute loc of i_sclk180 is  "BUFGMUX2"
 BUFG i_sclk270(.I(isclk270),.O(sclk270));
// s-ynthesis attribute loc of i_sclk270 is "BUFGMUX3"
endmodule



