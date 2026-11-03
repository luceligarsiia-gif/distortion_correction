// All ports synchronous to DDR user clock. Addresses are 32-bit WORD addresses.
// Release raw reservation only after the reader switches at an idle boundary.
module display_frame_select(input clk,rst_n,frame_boundary,reader_idle,
 input requested_corrected,corrected_valid,input [1:0] corrected_slot,
 input source_valid,input [1:0] source_slot,write_slot,
 input source_hold_valid,input [1:0] source_hold_slot,
 output reg active_corrected,frame_valid,
 output reg [27:0] frame_base,output reg [1:0] display_slot,
 output raw_hold_valid,output [1:0] raw_hold_slot,
 output boundary_ready);
reg raw_reserved;
reg [1:0] raw_slot;
// source_valid is qualified against capture slot reuse by the publisher.
// Lock the latest completed source on the selection edge itself, including
// coincident capture VSYNC, instead of rejecting it because it is next in ring.
wire raw_ready=source_valid;
wire raw_acquire=frame_boundary && reader_idle && !requested_corrected && raw_ready;
assign raw_hold_valid=raw_reserved || raw_acquire;
assign raw_hold_slot=raw_acquire ? source_slot : raw_slot;
reg have_frame;
assign boundary_ready=reader_idle && (have_frame ||
 (requested_corrected ? corrected_valid : raw_ready));
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  active_corrected<=1;frame_valid<=0;have_frame<=0;frame_base<=28'h1000000;
  display_slot<=0;raw_reserved<=0;raw_slot<=0;
 end else begin
  
  if(frame_boundary) begin
  frame_valid<=boundary_ready;
  if(reader_idle) begin
   if(requested_corrected && corrected_valid) begin
    active_corrected<=1;have_frame<=1;
    raw_reserved<=0;
    frame_base<=28'h1000000+{4'd0,corrected_slot,22'd0};display_slot<=corrected_slot;
   end else if(!requested_corrected && raw_ready) begin
    active_corrected<=0;have_frame<=1;
    frame_base<={4'd0,source_slot,22'd0};raw_reserved<=1;raw_slot<=source_slot;
   end
   // If target unavailable, repeat the last complete, reserved frame.
  end
 end
 end
endmodule
