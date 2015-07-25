/*
** -----------------------------------------------------------------------------**
** control_regs.v
**
** various control bits (what was a single control register before)
**
** Copyright (C) 2008 Elphel, Inc
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

module control_regs     (sclk,    // @negedge
                         wen,     // sync to address and d[0:15]
                         wa,      // [1:0] register select
                         di,      // [15:0] data in
/// outputs
                         bayer_phase, //[1:0]
                         hact_regen,
                         reset_mcontr,
                         break_frames, /// Enable ending frame if no more data is available
                         zoran,
                         use_sensor_clk,
                         xt_pol,
                         arst,
                         aro,
                         encnvclk,
                         sensor_trigger,
                         mrst,
                         external_timestamp, // use external timestamp if available
                         output_timestamp,   // output timestamp, not just pulse
                         dclkmode,
                         pxd14,
                         latehact, //[1:0]=dcr[22:21];//  register hact, vact N/4 Tpclk later than data (needed for MT9P001 @ 96MHz)
                         pclksrc,  //[1:0]=dcr[25:24]; // pclk source
                         hfc_sel,   //[2:0]=dcr[30:28];
                         blockvsync, // block vsync from sensor to sequencers
                         compressed_frames // 8-bit selection of which frames (modulo 8) to compress. deafaults to 'hff
                      );   // [2:0] current frame modulo 8
    input         sclk;
    input         wen;
    input  [ 1:0] wa;
    input  [15:0] di;
    output [ 1:0] bayer_phase; //[1:0]
    output        hact_regen;
    output        reset_mcontr;
    output        break_frames; /// Enable ending frame if no more data is available
    output        zoran;
    output        use_sensor_clk;
    output        xt_pol;
    output        arst;
    output        aro;
    output        encnvclk;
    output        sensor_trigger;
    output        mrst;
    output        external_timestamp;
    output        output_timestamp;
    output  [7:0] compressed_frames;
    output        dclkmode;
    output        pxd14;
    output  [1:0] latehact; //[1:0]=dcr[22:21];//  register hact, vact N/4 Tpclk later than data (needed for MT9P001 @ 96MHz)
    output  [1:0] pclksrc;  //[1:0]=dcr[25:24]; // pclk source
    output  [2:0] hfc_sel;  //[2:0]=dcr[30:28];
    output        blockvsync; // block vsync from sensor to sequencers
    reg     [1:0] wend; // we delayed
    reg     [1:0] wad;  // address delayed
    reg     [2:0] reg_wr; // write data to registers (group 0, group1)
    reg    [15:0] d1;
    reg    [31:0] d2;
    reg     [7:0] compressed_frames=8'hff; // TODO change other FDE_1 to registers

    always @ (negedge sclk) begin
      wend[1:0] <= {wend[0], wen};
      if (wen) wad[1:0] <= wa[1:0];
      reg_wr[2:0] <= {wend[1] & (wad[1:0]==2'h1), wend[1] & (wad[1:0]==2'h3), wend[1] & (wad[1:0]==2'h2)};
      if (wen || wend[0]) d1[15:0] <= di[15:0];
      if (wend[0]) d2[15: 0] <= d1[15:0];
      if (wend[1]) d2[31:16] <= d1[15:0];

    end
    FDE_1 i_bayer_phase_0     (.C(sclk),.CE(reg_wr[0] & d2[ 2]),.D(d2[ 0]), .Q(bayer_phase[0]));
    FDE_1 i_bayer_phase_1     (.C(sclk),.CE(reg_wr[0] & d2[ 2]),.D(d2[ 1]), .Q(bayer_phase[1]));
    FDE_1 i_hact_regen        (.C(sclk),.CE(reg_wr[0] & d2[ 4]),.D(d2[ 3]), .Q(hact_regen));
    FDE_1 i_reset_mcontr      (.C(sclk),.CE(reg_wr[0] & d2[ 6]),.D(d2[ 5]), .Q(reset_mcontr));
    FDE_1 i_zoran             (.C(sclk),.CE(reg_wr[0] & d2[ 8]),.D(d2[ 7]), .Q(zoran));
    FDE_1 i_use_sensor_clk    (.C(sclk),.CE(reg_wr[0] & d2[10]),.D(d2[ 9]), .Q(use_sensor_clk));
    FDE_1 i_xt_pol            (.C(sclk),.CE(reg_wr[0] & d2[12]),.D(d2[11]), .Q(xt_pol));
    FDE_1 i_arst              (.C(sclk),.CE(reg_wr[0] & d2[14]),.D(d2[13]), .Q(arst));
    FDE_1 i_aro               (.C(sclk),.CE(reg_wr[0] & d2[16]),.D(d2[15]), .Q(aro));
    FDE_1 i_encnvclk          (.C(sclk),.CE(reg_wr[0] & d2[18]),.D(d2[17]), .Q(encnvclk));
    FDE_1 i_sensor_trigger    (.C(sclk),.CE(reg_wr[0] & d2[20]),.D(d2[19]), .Q(sensor_trigger));
    FDE_1 i_break_frames      (.C(sclk),.CE(reg_wr[0] & d2[22]),.D(d2[21]), .Q(break_frames));
     
    FDE_1 i_mrst              (.C(sclk),.CE(reg_wr[1] & d2[ 1]),.D(d2[ 0]), .Q(mrst));
    FDE_1 i_external_timestamp(.C(sclk),.CE(reg_wr[1] & d2[ 3]),.D(d2[ 2]), .Q(external_timestamp));
    FDE_1 i_dclkmode          (.C(sclk),.CE(reg_wr[1] & d2[ 5]),.D(d2[ 4]), .Q(dclkmode));
    FDE_1 i_pxd14             (.C(sclk),.CE(reg_wr[1] & d2[ 7]),.D(d2[ 6]), .Q(pxd14));
    FDE_1 i_latehact_0        (.C(sclk),.CE(reg_wr[1] & d2[10]),.D(d2[ 8]), .Q(latehact[0]));
    FDE_1 i_latehact_1        (.C(sclk),.CE(reg_wr[1] & d2[10]),.D(d2[ 9]), .Q(latehact[1]));
    FDE_1 i_pclksrc_0         (.C(sclk),.CE(reg_wr[1] & d2[13]),.D(d2[11]), .Q(pclksrc[0]));
    FDE_1 i_pclksrc_1         (.C(sclk),.CE(reg_wr[1] & d2[13]),.D(d2[12]), .Q(pclksrc[1]));
    FDE_1 i_hfc_sel_0         (.C(sclk),.CE(reg_wr[1] & d2[17]),.D(d2[14]), .Q(hfc_sel[0]));
    FDE_1 i_hfc_sel_1         (.C(sclk),.CE(reg_wr[1] & d2[17]),.D(d2[15]), .Q(hfc_sel[1]));
    FDE_1 i_hfc_sel_2         (.C(sclk),.CE(reg_wr[1] & d2[17]),.D(d2[16]), .Q(hfc_sel[2]));
    FDE_1 i_blockvsync        (.C(sclk),.CE(reg_wr[1] & d2[19]),.D(d2[18]), .Q(blockvsync));
    FDE_1 i_output_timestamp  (.C(sclk),.CE(reg_wr[1] & d2[21]),.D(d2[20]), .Q(output_timestamp));

    always @ (negedge sclk) begin
      if (reg_wr[2] & d2[8]) compressed_frames[7:0] <= d2[7:0];
    end
    
endmodule
