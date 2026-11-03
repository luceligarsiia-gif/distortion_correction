// Source slot completion comes from actual row write completions, not VSYNC.
// Three corrected slots: never write either the displayed or newest ready slot.
module correction_scheduler #(parameter STREAM_CAPTURE=0)(input clk,rst_n,enable,
 input source_valid,input [15:0] source_sequence,input [1:0] source_slot,write_slot,display_slot,
 input source_complete,source_bad,
 input engine_busy,engine_done,engine_fault,
 output reg engine_start,output reg [27:0] source_base,dest_base,
 output source_hold_valid,output [1:0] source_hold_slot,
 output reg corrected_valid,output reg [1:0] corrected_slot,
 output reg [31:0] completed_frames,dropped_frames,output reg [15:0] engine_sequence);
reg [15:0] last_sequence;
reg have_sequence,inflight,overwritten,done_pending;
reg [1:0] locked_source,locked_dest;
reg [1:0] free_slot;
// Include acquisition combinationally, so a coincident capture VSYNC cannot
// select this source slot on the very edge that launches the correction.
wire acquire_source=enable && !engine_busy && !inflight && !engine_start && source_valid &&
    (!have_sequence || source_sequence!=last_sequence) && (STREAM_CAPTURE || source_slot!=write_slot);
assign source_hold_valid=inflight || acquire_source;
assign source_hold_slot=inflight ? locked_source : source_slot;
always @* begin
 if(display_slot!=0 && (!corrected_valid || corrected_slot!=0)) free_slot=0;
 else if(display_slot!=1 && (!corrected_valid || corrected_slot!=1)) free_slot=1;
 else free_slot=2;
end
always @(posedge clk or negedge rst_n)
if(!rst_n) begin
 engine_sequence<=0;engine_start<=0;source_base<=0;dest_base<=0;corrected_valid<=0;corrected_slot<=0;
 completed_frames<=0;dropped_frames<=0;last_sequence<=0;have_sequence<=0;inflight<=0;overwritten<=0;locked_source<=0;locked_dest<=0;done_pending<=0;
end else begin
 engine_start<=0;
 // Finish an already issued frame; launch nothing new while raw is requested.
 // Invalidate old corrected imagery so a later switch waits for a fresh frame.
 if(!enable) corrected_valid<=0;
 if(inflight && !engine_start && (STREAM_CAPTURE ? source_bad : write_slot==locked_source)) overwritten<=1;
 if(engine_done && inflight)done_pending<=1;
 if((engine_done || done_pending) && inflight && (!STREAM_CAPTURE || source_complete || source_bad)) begin
  done_pending<=0;
  inflight<=0;
  if(!engine_fault && !overwritten && (STREAM_CAPTURE ? !source_bad : write_slot!=locked_source)) begin
   corrected_slot<=locked_dest;corrected_valid<=enable;completed_frames<=completed_frames+1'b1;
  end else dropped_frames<=dropped_frames+1'b1;
 end
 if(acquire_source) begin
  done_pending<=0;engine_sequence<=source_sequence;engine_start<=1;inflight<=1;overwritten<=0;have_sequence<=1;last_sequence<=source_sequence;
  locked_source<=source_slot;locked_dest<=free_slot;
  source_base<={4'd0,source_slot,22'd0};
  dest_base<=28'h1000000+{4'd0,free_slot,22'd0};
 end
end
endmodule
