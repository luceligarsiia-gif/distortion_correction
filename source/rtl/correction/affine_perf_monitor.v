// NEW counts changed corrected display selections, not output VSYNCs.
// SKP counts source-sequence gaps between launches within one enabled epoch.
module affine_perf_monitor #(parameter integer CLK_HZ=125000000)(
 input clk,rst_n,enabled,input [31:0] completed_frames,dropped_frames,
 input engine_start,engine_busy,engine_done,input [15:0] engine_sequence,
 input [31:0] clipped_tiles,input active_corrected,frame_valid,
 input [1:0] display_slot,output reg [63:0] status);
localparam integer TICKS_PER_MS=CLK_HZ/1000;
reg [31:0] second_ticks,completed_prev,dropped_prev,proc_ticks;
reg [15:0] new_count,cor_count,skip_count,error_count,previous_sequence;
reg [15:0] proc_ms,last_proc_ms;
reg [7:0] last_clipped;
reg [1:0] previous_slot;
reg displayed_valid,have_sequence;
wire display_event=active_corrected && frame_valid &&
 (!displayed_valid || display_slot!=previous_slot);
wire [15:0] sequence_distance=engine_sequence-previous_sequence;
wire [15:0] skips=(enabled && engine_start && have_sequence &&
 sequence_distance>0 && sequence_distance<32768)?sequence_distance-1'b1:16'd0;
wire [31:0] cor_delta=completed_frames-completed_prev;
wire [31:0] error_delta=dropped_frames-dropped_prev;
function [7:0] cap;
 input [31:0] n;
 begin cap=n>255?8'd255:n[7:0];end
endfunction
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  second_ticks<=0;new_count<=0;cor_count<=0;skip_count<=0;error_count<=0;
  completed_prev<=0;dropped_prev<=0;previous_slot<=0;displayed_valid<=0;
  have_sequence<=0;previous_sequence<=0;proc_ticks<=0;proc_ms<=0;
  last_proc_ms<=0;last_clipped<=0;status<=0;
 end else begin
  completed_prev<=completed_frames;dropped_prev<=dropped_frames;
  previous_slot<=display_slot;displayed_valid<=active_corrected && frame_valid;
  if(!enabled)have_sequence<=0;
  else if(engine_start)begin have_sequence<=1;previous_sequence<=engine_sequence;end
  if(engine_start)begin proc_ticks<=0;proc_ms<=0;end
  else if(engine_busy)begin
   if(proc_ticks==TICKS_PER_MS-1)begin
    proc_ticks<=0;if(proc_ms<999)proc_ms<=proc_ms+1'b1;
   end else proc_ticks<=proc_ticks+1'b1;
  end
  if(engine_done)begin
   last_proc_ms<=proc_ms+((proc_ticks!=0 && proc_ms<999)?16'd1:16'd0);
   last_clipped<=cap(clipped_tiles);
  end
  if(second_ticks==CLK_HZ-1)begin
   second_ticks<=0;
   status<={8'd0,last_clipped,last_proc_ms,
     cap({16'd0,error_count}+error_delta),cap({16'd0,skip_count}+skips),
     cap({16'd0,cor_count}+cor_delta),cap({16'd0,new_count}+display_event)};
   new_count<=0;cor_count<=0;skip_count<=0;error_count<=0;
  end else begin
   second_ticks<=second_ticks+1'b1;
   new_count<=cap({16'd0,new_count}+display_event);
   cor_count<=cap({16'd0,cor_count}+cor_delta);
   skip_count<=cap({16'd0,skip_count}+skips);
   error_count<=cap({16'd0,error_count}+error_delta);
  end
 end
endmodule
