/*******************************************************************************
 * Include file: imu_sim2_include.vh
 * Date:2015-07-26  
 * Author: Andrey Filippov     
 * Description: Moved here all simulation for IMU logger 
 *
 * Copyright (c) 2015 Elphel, Inc .
 * imu_sim2_include.vh is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  imu_sim_include2.vh is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/> .
 *******************************************************************************/
 `ifdef TEST_IMU
  
      cpu_wr(X313_WA_IOPINS,      X313_WA_IOPINS_EN_IMU_OUT); // 'hc0000000;
/*
  reg           we_config_imu; // bits 1:0, 2 - enable slot[1:0]
  reg           we_config_gps; // bits 6:3, 7 - enable - {ext,inver, slot[1:0]} slot==0 - disable
  reg           we_config_msg; // bits 12:8,13 - enable - {invert,extinp[3:0]} extinp[3:0]=='hf' - disable
  reg           we_config_syn; // bit  14,  15 - enable  - enable logging external timestamps
  reg           we_config_rst; // bit  16,  17 - enable - reset modules 
*/

      cpu_wr(X313_WA_IMU_CTRL,            3); // select config register
//      cpu_wr(X313_WA_IMU_DATA,      'h3e695); // configure channels and reset // gps timestamp from 1sec input, positive
//      cpu_wr(X313_WA_IMU_DATA,      'h3e6b5); // configure channels and reset // gps timestamp from 1sec input, negative
//      cpu_wr(X313_WA_IMU_DATA,      'h3e6d5); // configure channels and reset // gps timestamp after pause
//      cpu_wr(X313_WA_IMU_DATA,      'h3e6f5); // configure channels and reset  // gps timestamp at "$" start
      cpu_wr(X313_WA_IMU_DATA,     'h43e6f5); // configure channels and reset  // gps timestamp at "$" start, reset configure_debug



      cpu_wr(X313_WA_IMU_CTRL,            2); // select register number 2 (serial half-bit duration)
      cpu_wr(X313_WA_IMU_DATA,       'h8007); // reset rs232 by 1 in MSB

      cpu_wr(X313_WA_IMU_CTRL,            2); // select register number 2 (serial half-bit duration)
      cpu_wr(X313_WA_IMU_DATA,       'h0007); // serial speed 8 cycles (period = 32 CLK0 cycles)
      
      cpu_wr(X313_WA_IMU_CTRL,            3); // select config register
      cpu_wr(X313_WA_IMU_DATA,      'h20000); // remove reset


// encode 4 sentences 
/*
$GPRMC,042931.0,A,4043.39929,N,11155.92706,W,000.00,283.8,250411,013.2,E*45
$GPGGA,042931.0,4043.39929,N,11155.92706,W,1,09,0.8,1280.5,M,-13.8,M,,*5B
$GPGSA,A,3,04,07,08,11,15,17,24,26,27,,,,1.7,0.8,1.5*36
$GPVTG,283.8,T,270.5,M,000.00,N,0000.00,K*7F
*/
      cpu_wr(X313_WA_IMU_CTRL,         'h20); // format write
// just ($GP)RMC, GGA, GSA and VTG
      cpu_wr(X313_WA_IMU_DATA,          'h6); //
      cpu_wr(X313_WA_IMU_DATA,          'hf); //
      cpu_wr(X313_WA_IMU_DATA,          'he); //
      cpu_wr(X313_WA_IMU_DATA,          'h0); //
      cpu_wr(X313_WA_IMU_DATA,          'h9); //

      cpu_wr(X313_WA_IMU_DATA,          'h7); //
      cpu_wr(X313_WA_IMU_DATA,          'h6); //
      cpu_wr(X313_WA_IMU_DATA,          'hb); //
      cpu_wr(X313_WA_IMU_DATA,          'h1); //
      cpu_wr(X313_WA_IMU_DATA,          'hc); //

      cpu_wr(X313_WA_IMU_DATA,          'hf); //
      cpu_wr(X313_WA_IMU_DATA,          'h9); //
      cpu_wr(X313_WA_IMU_DATA,          'h8); //
      cpu_wr(X313_WA_IMU_DATA,          'h0); //
      cpu_wr(X313_WA_IMU_DATA,          'h0); //
      cpu_wr(X313_WA_IMU_DATA,          'h0); //
      
///      cpu_wr(X313_WA_IMU_CTRL,          'h30); // first format
//$GPRMC,042931.0,A,4043.39929,N,11155.92706,W,000.00,283.8,250411,013.2,E*45
//0101010 000 : 'hb 'h2a 'h04 'h0
//      cpu_wr(X313_WA_IMU_DATA,          'h0b); //number of fields including dummy comma
      cpu_wr(X313_WA_IMU_DATA,          'h0a); // testing - made 1 shorter than actual
      cpu_wr(X313_WA_IMU_DATA,          'h2a); //
      cpu_wr(X313_WA_IMU_DATA,          'h04); //
      cpu_wr(X313_WA_IMU_DATA,          'h00); //

///      cpu_wr(X313_WA_IMU_CTRL,          'h34); // second format
//$GPGGA,042931.0,4043.39929,N,11155.92706,W,1,09,0.8,1280.5,M,-13.8,M,,*5B
//0010 1000 0101 0 : 'h0e 'h14 'h0a 'h0
      cpu_wr(X313_WA_IMU_DATA,          'h0e); //number of fields including dummy comma
      cpu_wr(X313_WA_IMU_DATA,          'h14); //
      cpu_wr(X313_WA_IMU_DATA,          'h0a); //
      cpu_wr(X313_WA_IMU_DATA,          'h00); //

///      cpu_wr(X313_WA_IMU_CTRL,          'h38); // third format
//$GPGSA,A,3,04,07,08,11,15,17,24,26,27,,,,1.7,0.8,1.5*36
//01000000 00000000 00 : 'h11 'h01 'h00 'h0
      cpu_wr(X313_WA_IMU_DATA,          'h11); //number of fields including dummy comma
      cpu_wr(X313_WA_IMU_DATA,          'h01); //
      cpu_wr(X313_WA_IMU_DATA,          'h00); //
      cpu_wr(X313_WA_IMU_DATA,          'h00); //

///      cpu_wr(X313_WA_IMU_CTRL,          'h3c); // fourth format
//$GPVTG,283.8,T,270.5,M,000.00,N,0000.00,K*7F
//00101010 1 : 'h08 'haa 'h00 'h0
      cpu_wr(X313_WA_IMU_DATA,          'h08); //number of fields including dummy comma
      cpu_wr(X313_WA_IMU_DATA,          'haa); //
      cpu_wr(X313_WA_IMU_DATA,          'h00); //
      cpu_wr(X313_WA_IMU_DATA,          'h00); //


      
      cpu_wr(X313_WA_IMU_CTRL,            4); // select register number 4
      cpu_wr(X313_WA_IMU_DATA,         'h10); // x gyro low
      cpu_wr(X313_WA_IMU_DATA,         'h12); // x gyro high
      cpu_wr(X313_WA_IMU_DATA,         'h14); //
      cpu_wr(X313_WA_IMU_DATA,         'h16); //
      cpu_wr(X313_WA_IMU_DATA,         'h18); //
      cpu_wr(X313_WA_IMU_DATA,         'h1a); //
      cpu_wr(X313_WA_IMU_DATA,         'h1c); // x accel low
      cpu_wr(X313_WA_IMU_DATA,         'h1e); //
      cpu_wr(X313_WA_IMU_DATA,         'h20); //
      cpu_wr(X313_WA_IMU_DATA,         'h22); //
      cpu_wr(X313_WA_IMU_DATA,         'h24); //
      cpu_wr(X313_WA_IMU_DATA,         'h26); // z accel high
      
      cpu_wr(X313_WA_IMU_DATA,         'h40); // x delta ang low
      cpu_wr(X313_WA_IMU_DATA,         'h42); // x delta ang high
      cpu_wr(X313_WA_IMU_DATA,         'h44); //
      cpu_wr(X313_WA_IMU_DATA,         'h46); //
      cpu_wr(X313_WA_IMU_DATA,         'h48); //
      cpu_wr(X313_WA_IMU_DATA,         'h4a); //
      cpu_wr(X313_WA_IMU_DATA,         'h4c); // x delta vel low
      cpu_wr(X313_WA_IMU_DATA,         'h4e); //
      cpu_wr(X313_WA_IMU_DATA,         'h50); //
      cpu_wr(X313_WA_IMU_DATA,         'h52); //
      cpu_wr(X313_WA_IMU_DATA,         'h54); //
      cpu_wr(X313_WA_IMU_DATA,         'h56); // z delta vel high
      
      cpu_wr(X313_WA_IMU_DATA,         'h0e); // temperature
      cpu_wr(X313_WA_IMU_DATA,         'h70); // time m/s
      cpu_wr(X313_WA_IMU_DATA,         'h72); // time d/h
      cpu_wr(X313_WA_IMU_DATA,         'h74); // time y/m

      cpu_wr(X313_WA_IMU_CTRL,            0); // select period register
      cpu_wr(X313_WA_IMU_DATA,            0); // reset IMU
      cpu_wr(X313_WA_IMU_DATA,            0); // reset bit counter
      
      #1000;
      cpu_wr(X313_WA_IMU_CTRL,            1); // select period register
      cpu_wr(X313_WA_IMU_DATA, IMU_BIT_DURATION); // set bit counter (clock frequency divider)

      cpu_wr(X313_WA_IMU_CTRL,            0); // select period register
      cpu_wr(X313_WA_IMU_DATA,   IMU_PERIOD); // set period
      
// set "odometer" message
      cpu_wr(X313_WA_IMU_CTRL,         'h40); // select start of message
      cpu_wr(X313_WA_IMU_DATA,         'h01234567); // Message first 4 bytes
      cpu_wr(X313_WA_IMU_DATA,         'h12345678); //next
      cpu_wr(X313_WA_IMU_DATA,         'h23456789); //next
      cpu_wr(X313_WA_IMU_DATA,         'h3456789a); //next
      cpu_wr(X313_WA_IMU_DATA,         'h456789ab); //next
      cpu_wr(X313_WA_IMU_DATA,         'h56789abc); //next
      cpu_wr(X313_WA_IMU_DATA,         'h6789abcd); //next
      cpu_wr(X313_WA_IMU_DATA,         'h789abcde); //next
      cpu_wr(X313_WA_IMU_DATA,         'h89abcdef); //next
      cpu_wr(X313_WA_IMU_DATA,         'h9abcdef0); //next
      cpu_wr(X313_WA_IMU_DATA,         'habcdef01); //next
      cpu_wr(X313_WA_IMU_DATA,         'hbcdef012); //next
      cpu_wr(X313_WA_IMU_DATA,         'hcdef0123); //next
      cpu_wr(X313_WA_IMU_DATA,         'hdef01234); //next
// extra 8 bytes - will not be logged
      cpu_wr(X313_WA_IMU_DATA,         'hef012345); //next
      cpu_wr(X313_WA_IMU_DATA,         'hf0123456); //next
      
//   cpu_wr(1,32'h00000);   // disable and reset dma
//   cpu_wr(1,32'h20000);   // enable DMA channel 1
   cpu_wr(1,32'h00024);   // disable and reset dma (both channels)
   cpu_wr(1,32'h00028);   // enable DMA channel 1
//      cpu_wr(X313_WA_IMU_DATA,            1); // set period
      
/*
 parameter X313_WA_IMU_DATA= 'h7e;
 parameter X313_WA_IMU_CTRL= 'h7f;

 parameter X313_RA_IMU_DATA= 'h7e; // read fifo word, advance pointer (32 reads w/o ready check)
 parameter X313_RA_IMU_STATUS= 'h7f; // LSB==ready

*/
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
       cpu_rd_ce1(1);
`endif
 