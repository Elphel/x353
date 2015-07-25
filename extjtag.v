/*
** -----------------------------------------------------------------------------**
** extjtag.v
**
** GPIO control
**
** Copyright (C) 2007 Elphel, Inc
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

/*
 Control programming of external FPGA on the sensor/sensor multiplexor board
 Mulptiplex status signals into a single line
 bits:
 31:20 - not used
 19:16 - 0xb..0xf - no changes
       - 0xa - select xfpgadone
       - 0x9 - select xfpgatdo
       - 0x8 - select senspgmin (default)
       - 0x0..0x7 - no changes
 15:10 - not used
  9: 8 - 3 - set xpgmen,
       - 2 - reset xpgmen,  
       - 0, 1 - no changes to xpgmen
  7: 6 - 3 - set xfpgaprog,
       - 2 - reset xfpgaprog,  
       - 0, 1 - no changes to xfpgaprog
  5: 4 - 3 - set xfpgatck,
       - 2 - reset xfpgatck,  
       - 0, 1 - no changes to xfpgatck
  3: 2 - 3 - set xfpgatms,
       - 2 - reset xfpgatms,
       - 0, 1 - no changes to xfpgatms
  1: 0 - 3 - set xfpgatdi,
       - 2 - reset xfpgatdi,
       - 0, 1 - no changes to xfpgatdi
*/

module extjtag          (sclk, // @negedge
                         pre_wen, // 1 cycle ahead of write data
                         di,      // [31:0] data in (only some bits are used)
                         xpgmen,     // enable programming mode for an external FPGA
                         xfpgaprog,  // PROG_B to be sent to an external FPGA
                         xfpgatck,   // TCK to be sent to an external FPGA
                         xfpgatms,   // TMS to be sent to an external FPGA
                         xfpgatdi,   // TDI to be sent to an external FPGA
                         
                         senspgmin,  // state of the SENSPGM I/O pin (read)
                         xfpgadone,  // state of the MRST pin ("DONE" pin on an external FPGA)
                         xfpgatdo,    // TDO read from an external FPGA

                         state);       // multiplexed state (one of 3 inputs)
    input         sclk;
    input         pre_wen;
    input  [31:0] di;
    output        xpgmen;       // enable programming mode for an external FPGA
    output        xfpgaprog;    // PROG_B to be sent to an external FPGA
    output        xfpgatck;     // TCK to be sent to an external FPGA
    output        xfpgatms;     // TMS to be sent to an external FPGA
    output        xfpgatdi;
                         
    input         senspgmin;
    input         xfpgadone;
    input         xfpgatdo;     // TDO read from an external FPGA
    output        state;
    
//    reg           wen;
    wire           wen= pre_wen;
    wire   [1:0]  mux;    // select source for state, initilaized to
    wire          xpgmen;
    wire          xfpgaprog;
    wire          xfpgatck;
    wire          xfpgatms;
    wire          xfpgatdi;
    wire          state=mux[1]?(mux[0]?1'b0:xfpgadone):(mux[0]?xfpgatdo:senspgmin);
/*
    always @ (negedge sclk) begin
      wen  <= pre_wen;
    end
*/    
    
    FDE_1 i_mux_0  (.C(sclk), .CE(wen & di[19]), .D(di[16]), .Q(mux[0]));
    FDE_1 i_mux_1  (.C(sclk), .CE(wen & di[19]), .D(di[17]), .Q(mux[1]));

    FDE_1 i_xpgmen    (.C(sclk), .CE(wen & di[9]), .D(di[8]), .Q(xpgmen));
    FDE_1 i_xfpgaprog (.C(sclk), .CE(wen & di[7]), .D(di[6]), .Q(xfpgaprog));
    FDE_1 i_xfpgatck  (.C(sclk), .CE(wen & di[5]), .D(di[4]), .Q(xfpgatck));
    FDE_1 i_xfpgatms  (.C(sclk), .CE(wen & di[3]), .D(di[2]), .Q(xfpgatms));
    FDE_1 i_xfpgatdi  (.C(sclk), .CE(wen & di[1]), .D(di[0]), .Q(xfpgatdi));
    
endmodule
