/*
** -----------------------------------------------------------------------------**
** sensor_phase353.v
**
** Phase conpensating for the sesnor data (separate for data and hact/vact)
**
** Copyright (C) 2010 Elphel, Inc
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

module sensor_phase353 #(
    parameter IOSTANDARD_SENSOR =     "LVCMOS33",
    parameter IFD_DELAY_SENSOR_VHACT =  "0",
    parameter IBUF_DELAY_SENSOR_VHACT = "0"
)(
                        cclk,       // command clock (posedge, invert on input if needed)
                        wcmd,       // write command
                        cmd,        // CPU write data [5:0]
                                    //       0 - nop, just reset status data
                                    //       1 - increase phase shift
                                    //       2 - decrease phase shift
                                    //       3 - reset phase shift to default (preprogrammed in FPGA configuration)
                                    //       c - reset phase90
                                    //       4 - incr pahse90
                                    //       8 - decrease phase90
                                    //       10 - increase hact/vact phase
                                    //       20 - decrease hact/vact phase
                                    //       30 - reset hact/vact phase
                        HACT,       //   sensor HACT I/O pin (input), used to reset FIFO
                        VACT,       //   sensor VACT I/O pin (input)
                        DI,         //   sensor D[DATA_WIDTH-1:0] i/o pins (input), strobed @posedge gclk_idata and en_idata
                        debug,      // 2-bit debug mode input
							   hact_length,// [13:0] WOI width-1 (to overwrite sensor HACT duration)
                        hact_regen, // 0 - use hact from sensor, 1 - regenerate using hact_lengh
                        mode_12bits,// input, 1 -  enable 12/14 bit mode, 0 - 10 bit mode
                        mode_14bits,// input, 1 -  enable 14 bit mode, 0 - 12/10 bit mode
                        mode_alt,   //   enable alternative vact/hact input (sync to data)
                        sync_alt,   //   alternative HACT/VACT input pad (10347) (VACT - 1 clock, HACT >1)
                        iclk,       //   DCM input clock (GCLK)
                        sclk,       //   global FIFO output clock (posedge)
                        shact,      //   hact - sync to sclk
                        en_svact,   // disable processing second vact after the trigger in triggered mode
                        svact,      //   vact - sync to sclk (single cycle)
                        sdo,        //   data output[DATA_WIDTH-1:0], sync to sclk
                        dcm_done,   //   DCM command done
                        status,     // dcm status (bit 1 - dcm clkin stopped)
                        locked);    //   DCM locked
/*in UCF                        
NET "i_sensorpads/i_sensor_phase/gclk_idata" TNM_NET = "TNM_GCLK_IDATA";
NET "i_sensorpads/i_sensor_phase/en_idata" TNM_NET = "TNM_EN_IDATA";
TIMESPEC "TS_PCLK_GCLK_IDATA" = FROM "TNM_PCLK" TO "TNM_GCLK_IDATA" TIG;
TIMESPEC "TS_GCLK_IDATA_PCLK" = FROM "TNM_GCLK_IDATA" TO "TNM_PCLK" TIG;
TIMESPEC "TS_DOUBLECYC_IDATA" = FROM "TNM_EN_IDATA" TO "TNM_EN_IDATA" "TS_CLK1";
NET "i_sensorpads/i_sensor_phase/phase_hact_sel_sync*" TIG;
NET "i_sensorpads/i_sensor_phase/mode_alt_sync" TIG;
NET "i_sensorpads/i_sensor_phase/mode_12bits_sync" TIG;
NET "i_sensorpads/i_sensor_phase/mode_14bits_sync" TIG;
NET "hact_length*" TIG;
*/

  parameter MIN_VACT_PERIOD=130; // 3-130, to increase maximal value (130) - chnge counter width
`ifdef SIMULATION
  parameter IS_SIMUL=1;
`else
  parameter IS_SIMUL=0;
`endif  


  input         cclk;       // command clock (posedge, invert on input if needed)
  input         wcmd;       // write command
  input   [5:0] cmd;        // CPU write data [5:0]
                            //       0 - nop, just reset status data
                            //       1 - increase phase shift
                            //       2 - decrease phase shift
                            //       3 - reset phase shift to default (preprogrammed in FPGA configuration)
                            //       c - reset phase90
                            //       4 - incr pahse90
                            //       8 - decrease phase90
                            //       10 - increase hact/vact phase
                            //       20 - decrease hact/vact phase
                            //       30 - reset hact/vact phase
  input         HACT;       //   sensor HACT I/O pin (input), used to reset FIFO
  input         VACT;       //   sensor VACT I/O pin (input)
  input  [11:0] DI;         //   sensor D11:0], after IBUF (IOBUF)
  input [1:0]    debug; // 2-bit debug mode input, connect to 2'b0 if not needed
  input  [13:0] hact_length;// [13:0] WOI width (to overwrite sensor HACT duration)
  input         hact_regen; // 0 - use hact from sensor, 1 - regenerate using hact_lengh
  input         mode_12bits;// input, 1 -  enable 12/14 bit mode, 0 - 10 bit mode, use 1'b1 in 10359
  input         mode_14bits;// input, 1 -  enable 14 bit mode, 0 - 12/10 bit mode, use 1'b0 in 10359
  
  input         mode_alt;   //   enable alternative vact/hact input (sync to data), use 1'b0 in 10359
  input         sync_alt;   //   alternative HACT/VACT input pad (10347) (VACT - 1 clock, HACT >1)

  input         iclk;       //   global sensor input clock (posedge) - the clock that goes to all 3 sensors
  input         sclk;       //   global FIFO output clock (posedge)
  output        shact;      //   hact - sync to sclk
  input         en_svact;   // disable processing second vact after the trigger in triggered mode
  output        svact;      //   vact - sync to sclk
  output [13:0] sdo;        //   data output[DATA_WIDTH-1:0], sync to sclk (Use {pxdo[11:0],unused[1:0]} in 10359
  output        dcm_done;   //   DCM command done
  output  [7:0] status;     //   DCM command done
  output        locked;     //   DCM locked



   wire       dcm_rst_cmd;
   reg  [2:0] dcm_drst;
   reg        dcm_en;
   reg        dcm_incdec; 
   reg        dcm_done;
   wire       dcm_done_dcm; // single-cycle
   reg        dcm_rst;
   reg  [2:0] dcm_reset_done;
   reg  [1:0] phase90sel;
   reg  [2:0] phase_hact_sel;
   wire       locked;
   wire       dcm_out0,dcm2x,dcm2x180;
   wire       gclk_idata;
   reg        en_idata;
   wire       pre_pre_en_idata, pre_pre_en_idata90;
   reg        pre_en_idata;
   reg        inv_gclk_idata;
   reg        inv_en_idata;
   wire [7:0] status;
//   reg        reset_fifo_in_cntr; // only for simulation;
   wire       reset_fifo_in_cntr; // only for simulation;

  FD i_dcm_rst_cmd(.Q(dcm_rst_cmd), .D((wcmd && (cmd[1:0] == 2'b11)) || (dcm_rst_cmd && !dcm_drst[2])), .C(cclk)) ;

  always @ (posedge cclk) begin
    dcm_reset_done[2:0] <= {dcm_reset_done[1] & ~dcm_reset_done[0], dcm_reset_done[0], dcm_rst}; // sync to cclkl end of dcm reset
    dcm_en     <= wcmd && (cmd[1]!=cmd[0]);
    dcm_incdec <= wcmd && cmd[0]; 
    if (wcmd) begin
      if      (cmd[2] && cmd[3]) phase90sel[1:0] <= 2'h0;
      else if (cmd[2])          phase90sel[1:0] <= phase90sel[1:0] +1;
      else if (cmd[3])          phase90sel[1:0] <= phase90sel[1:0] -1;
    end
    if (wcmd) begin
      if      (cmd[4] && cmd[5]) phase_hact_sel[2:0] <= 3'h0;
      else if (cmd[4])          phase_hact_sel[2:0] <= phase_hact_sel[2:0] +1;
      else if (cmd[5])          phase_hact_sel[2:0] <= phase_hact_sel[2:0] -1;
    end
//    if (wcmd) reset_fifo_in_cntr <= IS_SIMUL && (cmd[5:0]==6'h3f);
  end

  always @ (posedge iclk) begin
    dcm_drst[2:0] <= dcm_drst[2]? 3'b0:{dcm_drst[1], dcm_drst[0], dcm_rst_cmd};
    dcm_rst    <= dcm_drst[0]  || dcm_drst[1]   || dcm_drst[2] ;
  end
// make dcm_done behave as dcm_ready
  always @ (posedge cclk)
     if (wcmd && |cmd[2:0]) dcm_done <=1'b0;
     else if (dcm_done_dcm || dcm_reset_done[2]) dcm_done <=1'b1;

/// DCM to compensate sensor delays. Adjustment for data phase - both fine and 90-degrees, hact/vact - 90-degree steps relative to data

always @ (posedge gclk_idata) begin
  inv_gclk_idata <= phase90sel[0]; /// TODO: check polarities. Make them TIG?
  inv_en_idata <= phase90sel[1];   /// TODO: check polarities. Make them TIG?
  pre_en_idata <=inv_gclk_idata?pre_pre_en_idata90:pre_pre_en_idata; /// adjust tap
  en_idata <= pre_en_idata ^ inv_en_idata;
end   

BUFGMUX i_pclk  (.O(gclk_idata),  .I0(dcm2x), .I1(dcm2x180), .S(inv_gclk_idata));
  DCM_SP #(
      .CLKIN_DIVIDE_BY_2("FALSE"),     // TRUE/FALSE to enable CLKIN divide by two feature
      .CLKIN_PERIOD(10.0),            //96Hz
      .CLKOUT_PHASE_SHIFT("VARIABLE"),// Specify phase shift of NONE, FIXED or VARIABLE
      .CLK_FEEDBACK("1X"),            // Specify clock feedback of NONE, 1X or 2X
      .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"), // SOURCE_SYNCHRONOUS, SYSTEM_SYNCHRONOUS or
                                            //   an integer from 0 to 15
      .DLL_FREQUENCY_MODE("LOW"),     // HIGH or LOW frequency mode for DLL
      .DUTY_CYCLE_CORRECTION("TRUE"), // Duty cycle correction, TRUE or FALSE
      .PHASE_SHIFT(0),                // Amount of fixed phase shift from -255 to 255
      .STARTUP_WAIT("FALSE")          // Delay configuration DONE until DCM LOCK, TRUE/FALSE
   ) i_dcm_sensor(
    .CLKIN    (iclk),
    .CLKFB    (dcm_out0),
    .RST      (dcm_rst),
    .PSEN     (dcm_en),
    .PSINCDEC (dcm_incdec),
    .PSCLK    (cclk),
    .DSSEN    (1'b0),
    .CLK0     (dcm_out0),
    .CLK90    (pre_pre_en_idata),   // adjust tap
    .CLK180   (pre_pre_en_idata90), // adjust tap
    .CLK270   (),
    .CLKDV    (),
    .CLK2X    (dcm2x),
    .CLK2X180 (dcm2x180),
    .CLKFX    (),
    .CLKFX180 (),
    .STATUS   (status[7:0]),
    .LOCKED   (locked),
    .PSDONE   (dcm_done_dcm));


// Preventing skipping VACT/HACT after DCM reset (disabling update of the fifo output address:
 reg [1:0] dcm_in_locked;  // syncing to sclk
 reg [3:0] dcm_locked_cntr; // counter that extends !locked to the fifo size
 reg       dcm_fifo_locked; // safe to reset fifo output address
 always @ (posedge sclk)  begin
   dcm_in_locked[1:0] <= {dcm_in_locked[0],locked};
   if      (!dcm_in_locked[1])          dcm_fifo_locked      <= 1'b0;
   else if (dcm_locked_cntr[3:0]==4'h0) dcm_fifo_locked      <= 1'b1;
   
   if      (!dcm_in_locked[1])          dcm_locked_cntr[3:0] <= 4'hf;
   else if (!dcm_fifo_locked)           dcm_locked_cntr[3:0] <= dcm_locked_cntr[3:0]-1;
 end
 
// reg [11:0] idi;
 wire [11:0] idi;
 reg  [13:0] idi14;
 wire        hact_q0, vact_q0;
// wire       hact_q1,vact_q1;
 reg        hact_q1,vact_q1;
 wire       sync_alt_d0;
 reg  [2:1] sync_alt_d;
 reg  [3:0] hact_q1_d;
 reg  [3:0] hact_q0_d;
 reg  [3:0] vact_q1_d;
 reg  [3:0] vact_q0_d;
 reg        hact_vd; // variable delay, each clock cycle
 reg        vact_vd; // variable delay, each clock cycle
 reg        hact_selected; // selected between normal and alt hact
 reg        vact_selected; // selected between normal and alt vact
 reg  [1:0] hact_selected_d;
 reg        hact_selected_2_cycles; 
 reg  [1:0] vact_selected_d;
 reg        vact_selected_2_cycles;
 wire       alt_hact;
 wire       alt_vact;
 
 reg  [2:0] phase_hact_sel_sync;
 reg        mode_alt_sync;
 reg        mode_12bits_sync;
 reg        mode_14bits_sync;
 reg        svact,shact;
 reg [13:0] sdo;
 wire[13:0] pre_sdo;
 wire       pre_shact;
 wire       pre_svact;   ///nominally 2-cycle long output from FIFO (can be 1 or 3 long)
 reg        pre_svact_d; 
 wire       ihact,ivact;
 reg [13:0] hact_count;
 reg        pre_shact_d;
 reg [13:0] hact_length_sync;
 reg        hact_regen_sync; // sync to output
 reg        hact_regen_isync; // sync to gclk_idata
 reg        hact_count_zero;

 
 assign alt_hact=sync_alt_d[1] && (sync_alt_d0 ||  sync_alt_d[2]); /// already controlled by mode_alt
 assign alt_vact=sync_alt_d[1] && !sync_alt_d0 && !sync_alt_d[2]; /// already controlled by mode_alt
//synthesis translate_off
 wire vact_or_hact= VACT || HACT;
 reg  sim_rst;
 always @ (negedge glbl.GSR or posedge vact_or_hact) begin
   if  (vact_or_hact) sim_rst <= 1'b0;
   else               sim_rst <= 1'b1;
 end
 assign     reset_fifo_in_cntr=glbl.GSR || sim_rst;
//synthesis translate_on

 wire ihact00,ivact00;
///  some are double cycle
  IBUF   #(
        .IOSTANDARD          (IOSTANDARD_SENSOR),
        .IBUF_DELAY_VALUE    (IBUF_DELAY_SENSOR_VHACT),
        .IFD_DELAY_VALUE     (IFD_DELAY_SENSOR_VHACT)
   ) i_hact	(.I(HACT), .O(ihact));
  IBUF   #(
        .IOSTANDARD          (IOSTANDARD_SENSOR),
        .IBUF_DELAY_VALUE    (IBUF_DELAY_SENSOR_VHACT),
        .IFD_DELAY_VALUE     (IFD_DELAY_SENSOR_VHACT)
   ) i_vact	(.I(VACT), .O(ivact));

// synthesis attribute IOB     of i_hact       is "TRUE"
// synthesis attribute IOB     of i_vact       is "TRUE"
   
  always @ (posedge gclk_idata)  begin
    hact_q1 <= ihact00;
    vact_q1 <= ivact00;
    hact_q0_d[3:0]<={hact_q0_d[2:0],hact_q0};
    hact_q1_d[3:0]<={hact_q1_d[2:0],hact_q1};
    vact_q0_d[3:0]<={vact_q0_d[2:0],vact_q0};
    vact_q1_d[3:0]<={vact_q1_d[2:0],vact_q1};
  end
  
  always @ (posedge gclk_idata) if (en_idata) begin
// HACT delay -3..+4 90-degree steps, positive hact arrives later than data
    {vact_vd,hact_vd} <= phase_hact_sel_sync[2]?
                          (phase_hact_sel_sync[1]?
                            (phase_hact_sel_sync[0]?{vact_q1_d[2],hact_q1_d[2]}:{vact_q0_d[3],hact_q0_d[3]}):
                            (phase_hact_sel_sync[0]?{vact_q1_d[3],hact_q1_d[3]}:{vact_q0_d[0],hact_q0_d[0]})):
                          (phase_hact_sel_sync[1]?
                            (phase_hact_sel_sync[0]?{vact_q1_d[0],hact_q1_d[0]}:{vact_q0_d[1],hact_q0_d[1]}):
                            (phase_hact_sel_sync[0]?{vact_q1_d[1],hact_q1_d[1]}:{vact_q0_d[2],hact_q0_d[2]}));


  end
    IDDR2 i_ihact   (.Q0(hact_q0),.Q1(ihact00),.C0(gclk_idata),.C1(!gclk_idata),.CE(1'b1), .D(ihact), .R(1'b0), .S(1'b0) );	 
    IDDR2 i_ivact   (.Q0(vact_q0),.Q1(ivact00),.C0(gclk_idata),.C1(!gclk_idata),.CE(1'b1), .D(ivact), .R(1'b0), .S(1'b0) );
//    FD    i_hact_q1 (.C(gclk_idata), .D(ihact00), .Q(hact_q1));
//    FD    i_vact_q1 (.C(gclk_idata), .D(ivact00), .Q(vact_q1));

    FDCE i_sync_alt_d0 (.Q(sync_alt_d0),  .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(sync_alt));
    
    FDCE i_idi_0       (.Q(idi[ 0]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 0]));
    FDCE i_idi_1       (.Q(idi[ 1]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 1]));
    FDCE i_idi_2       (.Q(idi[ 2]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 2]));
    FDCE i_idi_3       (.Q(idi[ 3]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 3]));
    FDCE i_idi_4       (.Q(idi[ 4]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 4]));
    FDCE i_idi_5       (.Q(idi[ 5]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 5]));
    FDCE i_idi_6       (.Q(idi[ 6]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 6]));
    FDCE i_idi_7       (.Q(idi[ 7]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 7]));
    FDCE i_idi_8       (.Q(idi[ 8]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 8]));
    FDCE i_idi_9       (.Q(idi[ 9]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[ 9]));
    FDCE i_idi_10      (.Q(idi[10]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[10]));
    FDCE i_idi_11      (.Q(idi[11]),      .C(gclk_idata),.CE(en_idata),.CLR(1'b0),.D(DI[11]));
// are they still needed - seems yes    
// synthesis attribute IOB     of i_sync_alt_d0 is "TRUE"
// synthesis attribute IOB     of i_idi_0       is "TRUE"
// synthesis attribute IOB     of i_idi_1       is "TRUE"
// synthesis attribute IOB     of i_idi_2       is "TRUE"
// synthesis attribute IOB     of i_idi_3       is "TRUE"
// synthesis attribute IOB     of i_idi_4       is "TRUE"
// synthesis attribute IOB     of i_idi_5       is "TRUE"
// synthesis attribute IOB     of i_idi_6       is "TRUE"
// synthesis attribute IOB     of i_idi_7       is "TRUE"
// synthesis attribute IOB     of i_idi_8       is "TRUE"
// synthesis attribute IOB     of i_idi_9       is "TRUE"
// synthesis attribute IOB     of i_idi_10      is "TRUE"
// synthesis attribute IOB     of i_idi_11      is "TRUE"

reg  [1:0] shact_zero; // shact was zero (inactive), sync to gclk_data
  always @ (posedge gclk_idata) if (en_idata) begin
    idi14[13:4] <= idi[11:2];
    idi14[ 3:2] <= (mode_12bits_sync || mode_14bits_sync)? idi[1:0]:2'h0;
    idi14[ 1:0] <=  mode_14bits_sync? {vact_q0_d[0],hact_q0_d[0]}:2'h0;
    sync_alt_d[2:1]<={sync_alt_d[1], sync_alt_d0 & mode_alt_sync}; ///sync_alt_d[0] - IOB
    phase_hact_sel_sync[2:0] <= phase_hact_sel[2:0];
    mode_14bits_sync         <= mode_14bits;
    mode_12bits_sync         <= mode_12bits;
//    hact_regen_isync         <= hact_regen;
// only update hact_regen_isync when HACT (both input and output from fifo) are inactive (to prevent short lines)
    if ((!hact_q0_d[0]) && shact_zero[1]) hact_regen_isync  <= hact_regen;
//shact_zero[1]
 //hact_q0_d[3:0]
    mode_alt_sync <= mode_alt;
    hact_selected <= mode_alt_sync? alt_hact : hact_vd;
    vact_selected <= mode_alt_sync? alt_vact : vact_vd;
    
 ///if (hact_regen_isync==0) will copy hact to hact_selected_2_cycles    
    hact_selected_d[1:0]<={hact_selected_d[0], hact_selected && hact_regen_isync };
    hact_selected_2_cycles <= (hact_selected      && !hact_selected_d[0]) ||
                              (hact_selected_d[0] && !hact_selected_d[1]);
    vact_selected_d[1:0]<={vact_selected_d[0],vact_selected};
/*
    vact_selected_2_cycles <= (vact_selected      && !vact_selected_d[0]) ||
                              (vact_selected_d[0] && !vact_selected_d[1]);
*/                              
                              
    vact_selected_2_cycles <= (debug[0] || !hact_selected && !hact_selected_d[0] && !hact_selected_d[1]) && ((vact_selected      && !vact_selected_d[0]) ||
                              (vact_selected_d[0] && !vact_selected_d[1]));

                              
  end
//phase_hact_sel==3'h0 - delay by 2 cycles from data
reg  [3:0] fifo_data_in_addr;
reg  [3:0] fifo_hact_in_addr;
reg  [3:0] fifo_data_in_addr_saved;

reg        ihact_rst_in;
wire       reset_rq;
reg        rq;
reg        reset_out_fifo;
reg  [2:0] pre_reset_out_fifo;

assign     reset_rq= reset_out_fifo || (IS_SIMUL && reset_fifo_in_cntr) ;
//glbl.GSR
always @ (posedge gclk_idata) if (en_idata) begin
/// input FIFO counter is only reset for the simulation
  if (IS_SIMUL && reset_fifo_in_cntr) fifo_data_in_addr[3:0] <= 4'h0;
  else                                fifo_data_in_addr[3:0] <= fifo_data_in_addr[3:0] + 1;
//  if (IS_SIMUL && reset_fifo_in_cntr) fifo_hact_in_addr[3:0] <= 4'he;
//  else                                fifo_hact_in_addr[3:0] <= fifo_data_in_addr[3:0] - 1; // 1 behind data
  if (IS_SIMUL && reset_fifo_in_cntr) fifo_hact_in_addr[3:0] <= 4'hc;
  else                                fifo_hact_in_addr[3:0] <= fifo_data_in_addr[3:0] - 3; // 3 behind data
//  if (ihact_rst_in) fifo_data_in_addr_saved[3:0] <= fifo_data_in_addr[3:0] - 4'h5; // save FIFO in address for transferring to FIFO out address later
  if (ihact_rst_in) fifo_data_in_addr_saved[3:0] <= fifo_data_in_addr[3:0] - 4'h6; // save FIFO in address for transferring to FIFO out address later
end

always @ (posedge  gclk_idata or posedge  reset_rq) begin
  if (reset_rq)                       rq <= 1'b0;
  else if (ihact_rst_in && en_idata)  rq <= 1'b1;
end


reg  [3:0] fifo_out_addr;
always @ (posedge sclk) begin
  pre_reset_out_fifo[2:0]<={pre_reset_out_fifo[1:0], rq};
  reset_out_fifo<=pre_reset_out_fifo[1] && ! pre_reset_out_fifo[2];
  if (reset_out_fifo && dcm_fifo_locked) fifo_out_addr[3:0] <= fifo_data_in_addr_saved[3:0];
  else                                   fifo_out_addr[3:0] <= fifo_out_addr[3:0]+1;
end



reg        rq_back;
reg        reset_out_fifo_back;
reg  [2:0] pre_reset_out_fifo_back;
wire       reset_rq_back;


assign     reset_rq_back= reset_out_fifo_back || (IS_SIMUL && reset_fifo_in_cntr) ;

always @ (posedge  sclk or posedge  reset_rq_back) begin
  if      (reset_rq_back)      rq_back <= 1'b0;
  else if ( reset_out_fifo)    rq_back <= 1'b1;
end

reg        wait_reset_back;
always @ (posedge gclk_idata)  if (en_idata) begin
  shact_zero[1:0]<={shact_zero[0], ~shact}; // dual sync
  pre_reset_out_fifo_back[2:0]<={pre_reset_out_fifo_back[1:0], rq_back};
  reset_out_fifo_back<=pre_reset_out_fifo_back[1] && ! pre_reset_out_fifo_back[2];
  wait_reset_back <= !(IS_SIMUL && reset_fifo_in_cntr) && !hact_selected && shact_zero[1] && (ihact_rst_in || (wait_reset_back && !reset_out_fifo_back));
  ihact_rst_in <= (IS_SIMUL && reset_fifo_in_cntr) || (!hact_selected && shact_zero[1] && !wait_reset_back && !ihact_rst_in);
end


//FIFO  16 deep, 2 wide (vact, hact)
wire pre_svact_outfifo;
myRAM_WxD_D #( .DATA_WIDTH(2),.DATA_DEPTH(4))
            i_fifo_hact_vact(.D({vact_selected_2_cycles,hact_selected_2_cycles}),
                             .WE(en_idata),
                             .clk(gclk_idata),
                             .AW(fifo_hact_in_addr[3:0]),
                             .AR(fifo_out_addr[3:0]),
                             .QW(),
                             .QR({pre_svact_outfifo,pre_shact}));

//FIFO  16 deep, 14 wide (data)
myRAM_WxD_D #( .DATA_WIDTH(14),.DATA_DEPTH(4))
            i_fifo_data_vact(.D(idi14[13:0]),
                             .WE(en_idata),
                             .clk(gclk_idata),
                             .AW(fifo_data_in_addr[3:0]),
                             .AR(fifo_out_addr[3:0]),
                             .QW(),
                             .QR(pre_sdo[13:0]));
reg vact_bypass;
//assign pre_svact=debug[1]?vact_bypass:pre_svact_outfifo;
// for simulation
assign pre_svact= (debug[1]?vact_bypass:pre_svact_outfifo) && (!(IS_SIMUL && reset_fifo_in_cntr));
reg  [7:0] svact_filter_cntr=8'h80;
reg        en_svact_sync;
//  parameter MIN_VACT_PERIOD=130; // 3-130, to increase maximal value (130) - chnge counter width


always @ (posedge sclk)  begin
  vact_bypass<=vact_selected_2_cycles;
    hact_length_sync[13:0] <= hact_length[13:0];
  hact_regen_sync        <= hact_regen_isync;
  pre_shact_d            <= pre_shact;
  
  if (IS_SIMUL && reset_fifo_in_cntr)                      shact <= 1'b0;
  else if (pre_shact && !pre_shact_d)                      shact <= 1'b1;
  else if (hact_regen_sync?hact_count_zero:(!pre_shact))   shact <= 1'b0;

  if (!shact) hact_count[13:0] <= hact_length_sync[13:0];
  else        hact_count[13:0] <= hact_count[13:0]-1;
  hact_count_zero <= (hact_count[13:0]== 14'h2); // now starts from total number, not number-1
  sdo[13:0]     <= pre_sdo[13:0];
  
  en_svact_sync <= en_svact;
  pre_svact_d   <= pre_svact;
//  if      (pre_svact_d)           svact_filter_cntr <= 8'h82-MIN_VACT_PERIOD;
/// for simulation
  if      (pre_svact_d)                    svact_filter_cntr <= 8'h82-MIN_VACT_PERIOD;
  else if (IS_SIMUL && reset_fifo_in_cntr) svact_filter_cntr <= 8'h78;
  else if (!svact_filter_cntr[7])          svact_filter_cntr <= svact_filter_cntr+1;
  svact         <= pre_svact && ! pre_svact_d && en_svact_sync && svact_filter_cntr[7];
end
                             
endmodule

