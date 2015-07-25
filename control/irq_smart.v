/*
** -----------------------------------------------------------------------------**
** irq_smart.v
**
** making a simgle interrupt that combines frame sync and compressor done
** waiting for the latest of the 2 
**
** Copyright (C) 2008-2010 Elphel, Inc
**
** -----------------------------------------------------------------------------**
**  This file is part of X353
**  X353 is free software - hardware description language (HDL) code.wpage0_inc
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
//     control bits:
//     [1:0] : 3 - enable waiting for frame sync if compressor done comes earlier 
//             2 - disable waiting F.S. (should be disabled for single-frame acquisitions where no frame sync will follow the first frame)
//             1, 0  - don't change F.S. waiting 
//     [3:2] : 3 - wait for DMA FIFO to be transferred to the system before generating interrupt
//             2 - don't wait for the DMA FIFO to be emptied
//             1, 0  - don't change DMA FIFO waiting
//      [15] ; reset requests (mostly fro simulation
// NOTE: now if wait_fs is off, IRQ will be on either FS or done (may be twice)
module irq_smart        (sclk,               // @negedge
                         wen,                // sync to address and d[0:15]
                         di,                 // [15:0] data in, only [3:0] are currently used
                         frame_sync,         // frame sync, single pulse @ negedge sclk
                         is_compressing,      // @posedge clk, needs re-sync
                         compressor_done,    // single pulse @ negedge sclk - compressor finished (some data is still in DMA FIFO)
                         fifo_empty,         // DMA FIFO empty (no gaps between compressor_done and !fifo_empty)
                         irq);                // single cycle $ negedge sclk output to be used as IRQ source

    input         sclk;
    input         wen;
    input  [15:0] di;
    input         frame_sync;         // frame sync, single pulse @ negedge sclk
    input         is_compressing;     // @posedge clk, needs re-sync
    input         compressor_done;    // single pulse @ negedge sclk - compressor finished (some data is still in DMA FIFO)
    input         fifo_empty;         // DMA FIFO empty (no gaps between compressor_done and !fifo_empty)
    output        irq;                // single cycle $ negedge sclk output to be used as IRQ source
	 
    reg    [2:0]  is_compressing_s;
    reg           is_finishing=0; /// no gap with is_compressing_s[2]
    reg           was_finishing;
                  // together they provide number of frames currently being processed (0/1/2)
    reg           wait_frame_sync;
    reg           wait_fifo;
    reg           compressor_fifo_done; // single cycle - compressor and and fifo done (next after done if !wait_fifo)
    reg           done_request  = 0;
    reg           irq;
    reg           rst;
    wire          will_postpone_fs; 
    wire          end_postpone_fs;
    wire          finished;
    reg           fs_postponed;
    wire          will_delay_done_irq;
    reg           delaying_done_irq;
    assign   will_postpone_fs=wait_frame_sync && (is_compressing_s[2] || is_finishing) ;
    assign   finished=was_finishing && ! is_finishing;
    assign   end_postpone_fs=finished || frame_sync;
    assign   will_delay_done_irq=wait_frame_sync && (finished && !fs_postponed);
    
    always @ (negedge sclk) begin
//control interface    
     if (wen & di[1]) wait_frame_sync <= di[0];
     if (wen & di[3]) wait_fifo       <= di[2];
     rst <=wen & di[15];
 // process frame sync postponed - wait for the compression to finish if it was started during previous frame
     fs_postponed <= !rst && ((will_postpone_fs && frame_sync) || (fs_postponed && !end_postpone_fs));
     delaying_done_irq <= !rst &&  (will_delay_done_irq || (delaying_done_irq && !frame_sync));
     
     is_compressing_s[2:0]<={is_compressing_s[1:0],is_compressing} ; // re-sync from posedge xclk to negedge clk
     done_request  <= !rst && (compressor_done || (done_request &&  !compressor_fifo_done));
     compressor_fifo_done <= done_request && (!wait_fifo || fifo_empty) && !compressor_fifo_done;
     is_finishing <=  !rst && ((is_compressing_s[2] && !is_compressing_s[1]) || 
                               (is_finishing && !compressor_fifo_done));
     was_finishing <= is_finishing;

     irq <= !rst && ((frame_sync &&  (!will_postpone_fs || delaying_done_irq)) ||
                     (fs_postponed && end_postpone_fs) || // will include frame_sync if compression did not finish
                     (!will_delay_done_irq && finished));
    end
endmodule
