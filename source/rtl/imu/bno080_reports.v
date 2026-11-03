`timescale 1ns/1ps
// SH-2 input channel parser, only Rotation Vector (0x05) is enabled by host.
// Supports repeated 0xFB base timestamps and multiple 14-byte RV reports.
// Unknown/truncated records are discarded, not searched for magic byte values.
module bno080_reports(input clk,rst_n,input start,enable,byte_valid,
 input [7:0] data,output reg valid,
 output reg signed [15:0] qi,qj,qk,qr,output reg [1:0] accuracy,
 output reg debug_bad_record);
reg active;
reg [4:0] pos;
reg [7:0] kind;
reg [15:0] ti,tj,tk,tr;
reg [1:0] ta;
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  active<=0;pos<=0;kind<=0;valid<=0;qi<=0;qj<=0;qk<=0;qr<=0;
  ti<=0;tj<=0;tk<=0;tr<=0;ta<=0;accuracy<=0;debug_bad_record<=0;
 end else begin
  valid<=0;debug_bad_record<=0;
  if(start) begin
   debug_bad_record<=active && pos!=0;
   active<=enable;pos<=0;
  end
  else if(byte_valid && active) begin
   if(pos==0) begin
    kind<=data;pos<=1;
    if(data!=8'hfb && data!=8'h05)begin active<=0;debug_bad_record<=1;end
   end else if(kind==8'hfb) begin
    if(pos==4) pos<=0;else pos<=pos+1'b1;
   end else begin
    pos<=pos+1'b1;
    case(pos)
     2: ta<=data[1:0];
     4: ti[7:0]<=data; 5: ti[15:8]<=data;
     6: tj[7:0]<=data; 7: tj[15:8]<=data;
     8: tk[7:0]<=data; 9: tk[15:8]<=data;
     10: tr[7:0]<=data; 11: tr[15:8]<=data;
     13: begin qi<=ti;qj<=tj;qk<=tk;qr<=tr;accuracy<=ta;valid<=1;pos<=0;end
     default: ;
    endcase
   end
  end
 end
endmodule
