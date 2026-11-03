// Read-only diagnostic mailbox. Common reset assertion is required at both
// ends. A busy source drops a diagnostic sample and counts it, never stalls
// video/IMU. Payload is held unchanged until the synchronized acknowledge.
module diagnostic_cdc #(parameter WIDTH=32)(
 input src_clk,rst_n,src_valid,input [WIDTH-1:0] src_data,
 output busy,output reg [15:0] lost,
 input dst_clk,output reg dst_valid,output reg [WIDTH-1:0] dst_data);
reg [WIDTH-1:0] holding;
reg request,ack;
(* async_reg="true" *) reg [1:0] req_sync,ack_sync;
assign busy=request!=ack_sync[1];
always @(posedge src_clk or negedge rst_n)
 if(!rst_n)begin holding<=0;request<=0;ack_sync<=0;lost<=0;end
 else begin
  ack_sync<={ack_sync[0],ack};
  if(src_valid)begin
   if(!busy)begin holding<=src_data;request<=~request;end
   else if(lost!=65535)lost<=lost+1'b1;
  end
 end
always @(posedge dst_clk or negedge rst_n)
 if(!rst_n)begin req_sync<=0;ack<=0;dst_valid<=0;dst_data<=0;end
 else begin
  req_sync<={req_sync[0],request};dst_valid<=0;
  if(req_sync[1]!=ack)begin dst_data<=holding;dst_valid<=1;ack<=req_sync[1];end
 end
endmodule

// Track the SAME matrix latched by the renderer, indexed by corrected DDR slot.
// Convert on display selection, not on IMU arrival or engine completion.
module diagnostic_display_angle(
 input clk,rst_n,engine_start,input [1:0] dest_slot,
 input [287:0] matrix,input crop,
 input active_corrected,frame_valid,input [1:0] display_slot,
 output reg out_valid,output reg [31:0] angle,output reg angle_valid);
reg signed [31:0] cosines[0:2],sines[0:2];
reg [2:0] crops,initialized;
reg seen,selected_active,selected_valid;
reg [1:0] selected_slot;
reg start,waiting,job_valid;
reg signed [31:0] x,y;
wire done;wire signed [31:0] result;
wire change= !seen || selected_slot!=display_slot ||
 selected_active!=active_corrected || selected_valid!=frame_valid;
integer i;
affine_atan2 convert(clk,rst_n,start,x,y,done,result);
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  crops<=0;initialized<=0;seen<=0;selected_active<=0;selected_valid<=0;selected_slot<=0;
  start<=0;waiting<=0;job_valid<=0;x<=16384;y<=0;out_valid<=0;angle<=0;angle_valid<=0;
  for(i=0;i<3;i=i+1)begin cosines[i]<=16384;sines[i]<=0;end
 end else begin
  start<=0;out_valid<=0;
  if(engine_start && dest_slot<3)begin
   cosines[dest_slot]<=$signed(matrix[0+:32]);sines[dest_slot]<=$signed(matrix[96+:32]);
   crops[dest_slot]<=crop;initialized[dest_slot]<=1;
  end
  if(!waiting && change)begin
   seen<=1;selected_slot<=display_slot;selected_active<=active_corrected;selected_valid<=frame_valid;
   if(active_corrected && frame_valid && display_slot<3 && initialized[display_slot] && crops[display_slot])begin
    x<=cosines[display_slot];y<=sines[display_slot];start<=1;waiting<=1;job_valid<=1;
   end else begin angle_valid<=0;out_valid<=1;end
  end
  if(done)begin angle<=result;angle_valid<=job_valid;out_valid<=1;waiting<=0;end
 end
endmodule
