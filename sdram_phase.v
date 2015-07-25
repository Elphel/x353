/*
** -----------------------------------------------------------------------------**
** sdram_phase.v
**
** I/O pads related circuitry
**
** Copyright (C) 2002 Elphel, Inc
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

// assumed CL=2.5
module sdram_phase (// wclk,       // global CPU WE pulse
                     pre_wcmd,   // decoded address - enables wclk
                     wd,         // CPU write data [1:0]
                                 //       0 - nop, just reset status data
                                 //       1 - increase phase shift
                                 //       2 - decrease phase shift
                                 //       3 - reset phase shift to default (preprogrammed in FPGA configuration)
                                 //       c - reset phase90
                                 //       4 - incr pahse90
                                 //       8 - decrease phase90
                     ph_err,     // [1:0] 0 - no data (SDRAM reads) since last change (wclk*wcmd)
                                 //       1 - clock is too late
                                 //       2 - clock is too early
                                 //       3 - OK (some measurements show too late, some - too early)
                     sclk0,      // global clock, phase 0
/*                     sclk90,     // global clock, phase 0 */
                     sclk270,    // global clock, phase 0
                     enrd180,    // read enable, latency 2 from the command, sync with sclk falling edge
                     udqsr90,    // data from SDRAM interface pin UDQS strobed at rising sclk90
                     ldqsr90,    // data from SDRAM interface pin LDQS strobed at rising sclk90
                     udqsr270,   // data from SDRAM interface pin UDQS strobed at rising sclk270
                     ldqsr270,   // data from SDRAM interface pin UDQS strobed at rising sclk270
                     dcm_rst,    // set DCM phase to default
                     dcm_clk,    // clock for changing DCM phase (now == sclk0)
                     dcm_en,     // enable inc/dec of the DCM phase
                     dcm_incdec, // 0 - increment, 1 - decrement DCM phase
                     phase90sel  // add phase 0 - 0, 1 - 90, 2 - 180, 3 - 270

                     );
   input         pre_wcmd;
   input  [ 3:0] wd;
   output [ 1:0] ph_err;
   input         sclk0, /*sclk90, */sclk270;
   input         enrd180;
   input         udqsr90, udqsr270, ldqsr90, ldqsr270;
   output        dcm_rst, dcm_clk, dcm_en, dcm_incdec;
   output [ 1:0] phase90sel;
//	reg           wcmd;
	wire          wcmd = pre_wcmd;
   reg    [ 1:0] phase90sel;
   reg dcm_rst;
   reg [1:0] dcm_drst;
   reg       dcm_en;
   reg       dcm_incdec; 


   reg           enrd0, enrd180_d,enrd90,enrd270;
   reg           waslate90, waslate270, wasearly90, wasearly270;
// generate control pulses from CPU command

  always @ (negedge sclk0) begin
//    wcmd   <=pre_wcmd;
    dcm_drst[1:0] <= {dcm_drst[0], (wcmd && (wd[1:0] == 2'b11))};
    dcm_rst    <= (wcmd && (wd[1:0] == 2'b11)) || dcm_drst[0]  || dcm_drst[1] ;
    dcm_en     <= wcmd && (wd[1]!=wd[0]);
    dcm_incdec <= wcmd && wd[0]; 
    if      (wcmd && wd[2] && wd[3]) phase90sel[1:0] <= 2'h0;
    else if (wcmd && wd[2]) phase90sel[1:0] <= phase90sel[1:0] +1;
    else if (wcmd && wd[3]) phase90sel[1:0] <= phase90sel[1:0] -1;
  end



// DCM control outputs (use automatic adjustment later?) 
//   assign  dcm_clk=sclk0;
   assign  dcm_clk=!sclk0; // DCM is triggered by posedge

// generate phase error signals

   always @ (posedge sclk0)   enrd0     <= enrd180;
   always @ (negedge sclk0)   enrd180_d <= enrd180;
//   always @ (posedge sclk90)  enrd90    <= enrd180_d;
   always @ (negedge sclk270)  enrd90    <= enrd180_d;
   always @ (posedge sclk270) enrd270   <= enrd0;
//   always @ (posedge sclk90 or posedge wcmd)
   always @ (negedge sclk270 or posedge wcmd)
     if (wcmd) begin
       waslate90  <= 1'b0;
       wasearly90 <= 1'b0;
     end else begin
       waslate90  <= waslate90  || (enrd90 && ( udqsr90 ||  ldqsr90));
       wasearly90 <= wasearly90 || (enrd90 && (!udqsr90 || !ldqsr90));
     end
   always @ (posedge sclk270 or posedge wcmd)
     if (wcmd) begin
       waslate270  <= 1'b0;
       wasearly270 <= 1'b0;
     end else begin
       waslate270  <= waslate270  || (enrd270 && (!udqsr270 || !ldqsr270));
       wasearly270 <= wasearly270 || (enrd270 && ( udqsr270 ||  ldqsr270));
     end

     assign ph_err[1:0]= {(wasearly90 || wasearly270), (waslate90 || waslate270)};

endmodule
