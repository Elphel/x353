/*******************************************************************************
 * Include file: imu_sim_include.vh
 * Date:2015-07-26  
 * Author: Andrey Filippov     
 * Description: Moved here all simulation for IMU logger 
 *
 * Copyright (c) 2015 Elphel, Inc .
 * imu_sim_include.vh is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  imu_sim_include.vh is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/> .
 *******************************************************************************/
 #250000;
     dma_en(0,1);
`ifdef TEST_IMU
      cpu_wr(X313_WA_IMU_CTRL,            3); // select config register
      cpu_wr(X313_WA_IMU_DATA,     'h4c0000); // set debug_config to 4'h3

      cpu_wr(X313_WA_IMU_CTRL,            1); // select period register
      cpu_wr(X313_WA_IMU_DATA, IMU_BIT_DURATION | 16'h1000); // set bit counter and stall of 16 sclk half-periods


      wait (IMU_CS); // wait IMU inactive
      IMU_103695REVA  = 1'b1; // switch to revision "A"
      cpu_wr(X313_WA_IMU_CTRL,            3); // select config register
      cpu_wr(X313_WA_IMU_DATA,     'h5c0000); // set debug_config to 4'h7
      
      cpu_wr(X313_WA_IMU_CTRL,            0); // select period register
      cpu_wr(X313_WA_IMU_DATA,   IMU_AUTO_PERIOD); // set period defined by IMU
`endif

     
#480000;
//#480000;
$finish;
`ifdef TEST_IMU
  
      cpu_rd(X313_RA_IMU_STATUS);     $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd(X313_RA_IMU_STATUS);     $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      
      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);


 `endif  
   
   
   $finish;            

//#250000;
//     dma_en(0,1);
//#480000;
`ifdef TEST_IMU
     dma_en(0,1);
     
      cpu_wr(X313_WA_IMU_CTRL,            3); // select config register
      cpu_wr(X313_WA_IMU_DATA,     'h4c0000); // set debug_config to 4'h3

      cpu_wr(X313_WA_IMU_CTRL,            1); // select period register
      cpu_wr(X313_WA_IMU_DATA, IMU_BIT_DURATION | 16'h1000); // set bit counter and stall of 16 sclk half-periods


      wait (IMU_CS); // wait IMU inactive
      IMU_103695REVA     = 1'b1; // switch to revision "A"
      cpu_wr(X313_WA_IMU_CTRL,            3); // select config register
      cpu_wr(X313_WA_IMU_DATA,     'h5c0000); // set debug_config to 4'h7
      
      cpu_wr(X313_WA_IMU_CTRL,            0); // select period register
      cpu_wr(X313_WA_IMU_DATA,   IMU_AUTO_PERIOD); // set period defined by IMU
`endif
     
#480000;
$finish;
`ifdef TEST_IMU
  
      cpu_rd(X313_RA_IMU_STATUS);     $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd(X313_RA_IMU_STATUS);     $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      
      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);

      cpu_rd_ce1(X313_RA_IMU_STATUS); $display ("IMU_STATUS =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);
      cpu_rd_ce1(X313_RA_IMU_DATA);   $display ("IMU_DATA =%x",CPU_DI[31:0]);


 `endif  
   
   
   $finish;            

//#250000;
//     dma_en(0,1);
//#480000;
#200000;

$finish;            
 