/*
** -----------------------------------------------------------------------------**
** sensdcclk.v
**
** Frame-synchronous DC-DC converter clock
**
** Copyright (C) 2002 Elphel, Inc
**
** -----------------------------------------------------------------------------**
**  This file is part of X313
**  X313 is free software - hardware description language (HDL) code.
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

module sensdcclk(	clk,		// sensor clock (may be variable)
						frame,	// frame sync (1 cycle, sync to sensclk)
						d,			// [6:0]	divider. Nominal 5'h05 for 20MHz pixel rate
									// data=(Fpix[MHz]/1.2)-1, if Fpix=20MHz, data=15.7->16=5'h10
						sclk,		// 
						pre_we,  // write enable to sensdcclk (sync to cwr)
						cnvclk,  // 625KHz clock to drive DC-DC converter
						cnvext); // 0 - DCDC use internal clock, 1 - external

    input			clk;
    input			frame;
    input [6:0]	d;
    input			sclk;
    input			pre_we;
    output			cnvclk;
    output			cnvext;

	 wire	 [6:0]	dvs;
	 reg	 [4:0]	cntr;
	 reg				cnvext;
	 wire				cntr_en= |dvs[4:0];	// clock off if set to 0;
	 reg				cnvclk;
	 reg				fullper;	// full cnvclk period since last frame sync
	 wire				sync=fullper && frame;
	 reg	 [3:0]	div1;
	 reg	 [3:0]	div2;
	 reg	 [3:0]	div3;
	 wire				cry1=(div1[3:0]==4'hf);
	 wire				cry2=(div2[3:0]==4'hf);
	 wire				cry3=(div3[3:0]==4'hf);
//	 wire				cry=cry1&& cry2 && cry3;
	 wire				dec=(dvs[6:5]==2'b0) || (cry1 &&  (!dvs[6] || (cry2 && (!dvs[5] || cry3) )));
	 wire				half= (cntr[4:0]== 5'b0) && dec;
//    reg           we;
    wire       we= pre_we;

//    always @(negedge sclk) we <= pre_we;

	 always @ (posedge clk) begin
	   if (!cntr_en)				div1 <= 4'b0;
		else							div1 <= div1+1;
	   if (!cntr_en)				div2 <= 4'b0;
		else if (cry1)				div2 <= div2+1;
	   if (!cntr_en)				div3 <= 4'b0;
		else if (cry1 && cry2)	div3 <= div3+1;
	 end


	 FDE_1 i_dvs_0	(.C(sclk), .CE(we), .D(d[0]), .Q(dvs[0]));
	 FDE_1 i_dvs_1	(.C(sclk), .CE(we), .D(d[1]), .Q(dvs[1]));
	 FDE_1 i_dvs_2	(.C(sclk), .CE(we), .D(d[2]), .Q(dvs[2]));
	 FDE_1 i_dvs_3	(.C(sclk), .CE(we), .D(d[3]), .Q(dvs[3]));
	 FDE_1 i_dvs_4	(.C(sclk), .CE(we), .D(d[4]), .Q(dvs[4]));
	 FDE_1 i_dvs_5	(.C(sclk), .CE(we), .D(d[5]), .Q(dvs[5]));
	 FDE_1 i_dvs_6	(.C(sclk), .CE(we), .D(d[6]), .Q(dvs[6]));

	 always @ (posedge clk) cnvext <= cntr_en;

	 always @ (posedge clk)
	   if (sync || !cntr_en || half) cntr[4:0] <= dvs[4:0];
		else	if (dec)						cntr[4:0] <= cntr[4:0]-1;
	 always @ (posedge clk)
	   if (sync || !cntr_en) cnvclk <= 1'b0;
		else if (half)			 cnvclk <= ~cnvclk;
	 always @ (posedge clk)
	   if (sync || !cntr_en) fullper <= 1'b0;
		else if (half)			 fullper <= fullper | cnvclk;
endmodule

