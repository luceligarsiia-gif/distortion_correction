// Bundled-data request/ack transfer. Payload remains stable until destination ACK.
module pose_cdc(input src_clk,src_rst_n,src_valid,input [287:0] src_matrix,
 input src_affine_crop,src_history_active,input dst_clk,dst_rst_n,output reg [287:0] dst_matrix,
 output reg dst_affine_crop,dst_history_active,output reg dst_valid);
reg [287:0] holding,latest;
reg holding_crop,latest_crop;
reg holding_active,latest_active;
reg pending;
reg request,acknowledge;
(* async_reg="true" *) reg [1:0] ack_sync,req_sync;
always @(posedge src_clk or negedge src_rst_n)
 if(!src_rst_n) begin holding<=0;latest<=0;holding_crop<=0;latest_crop<=0;holding_active<=0;latest_active<=0;pending<=0;request<=0;ack_sync<=0;end
 else begin
  ack_sync<={ack_sync[0],acknowledge};
  if(src_valid) begin latest<=src_matrix;latest_crop<=src_affine_crop;latest_active<=src_history_active;pending<=1;end
  if(pending && request==ack_sync[1]) begin
   holding<=latest;holding_crop<=latest_crop;holding_active<=latest_active;request<=~request;pending<=src_valid;
  end
 end
always @(posedge dst_clk or negedge dst_rst_n)
 if(!dst_rst_n) begin
  req_sync<=0;acknowledge<=0;dst_valid<=0;dst_matrix<=0;dst_affine_crop<=0;dst_history_active<=0;
  dst_matrix[0*32+:32]<=16384;dst_matrix[4*32+:32]<=16384;dst_matrix[8*32+:32]<=16384;
 end else begin
  req_sync<={req_sync[0],request};dst_valid<=0;
  if(req_sync[1]!=acknowledge) begin dst_matrix<=holding;dst_affine_crop<=holding_crop;dst_history_active<=holding_active;dst_valid<=1;acknowledge<=req_sync[1];end
 end
endmodule
