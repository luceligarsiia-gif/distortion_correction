// Completed DDR rows, indexed by the same four raw buffer owners as capture.
// A retained source slot is never reallocated until correction releases it.
module capture_progress #(parameter STREAM_CAPTURE=1,HEIGHT=1080)(
 input clk,rst_n,input [1:0] write_slot,input row_done,
 input source_valid,input [1:0] source_slot,read_slot,
 output offer_valid,output [1:0] offer_slot,output [15:0] offer_sequence,
 output [10:0] read_rows,output read_complete,read_bad);
reg [1:0] previous;
reg [10:0] rows[0:3];
reg [15:0] sequence_number,ids[0:3];
reg [3:0] bad,allocated;
wire allocation=write_slot!=previous;
wire current_ready=STREAM_CAPTURE && allocated[write_slot] && rows[write_slot]!=0 && !allocation;
assign offer_slot=current_ready?write_slot:source_slot;
assign offer_valid=current_ready || (source_valid && allocated[source_slot] && !bad[source_slot] &&
 !(allocation && source_slot==write_slot));
assign offer_sequence=ids[offer_slot];
assign read_bad=bad[read_slot] || (allocation && previous==read_slot && rows[read_slot]!=HEIGHT) ||
 (allocation && write_slot==read_slot);
assign read_rows=rows[read_slot];
assign read_complete=allocated[read_slot] && rows[read_slot]==HEIGHT && !read_bad;
integer i;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  previous<=0;sequence_number<=0;bad<=0;allocated<=0;
  for(i=0;i<4;i=i+1)begin rows[i]<=0;ids[i]<=0;end
 end else begin
  previous<=write_slot;
  if(allocation)begin
   if(allocated[previous] && rows[previous]!=HEIGHT)bad[previous]<=1;
   rows[write_slot]<=0;bad[write_slot]<=0;allocated[write_slot]<=1;
   sequence_number<=sequence_number+1'b1;ids[write_slot]<=sequence_number+1'b1;
  end else if(row_done && allocated[write_slot] && rows[write_slot]<HEIGHT)
   rows[write_slot]<=rows[write_slot]+1'b1;
 end
endmodule
