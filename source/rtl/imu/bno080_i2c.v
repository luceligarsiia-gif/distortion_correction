`timescale 1ns/1ps
// BNO080 I2C/SHTP host. AD0 high => 7-bit 0x4B. PS1=PS0=0.
// Request Rotation Vector at 100 Hz; signed Q14 quaternion I,J,K,Real.
// No MCU or camera SCCB bus is needed. Supports SCL clock stretching.
module bno080_i2c #(
 parameter CLK_HZ=27000000, BUS_HZ=100000,
 parameter [6:0] ADDRESS=7'h4b,
 parameter RESET_CYCLES=CLK_HZ/100, BOOT_CYCLES=CLK_HZ/2,
 parameter STALE_CYCLES=CLK_HZ/2, RETRY_CYCLES=CLK_HZ*2,
 parameter I2C_TIMEOUT=CLK_HZ/20
)(input clk,rst_n,input int_n,inout scl,sda,sensor_rst_n,
 output signed [15:0] qi,qj,qk,qr,output [1:0] accuracy,
 output valid,output online,output reg configured,
 output reg [15:0] reports,output reg [3:0] last_error,
 output [4:0] debug_events);
localparam RESET=0,BOOT=1,POLL=2,WAIT_BYTE=3,
 READ_START=4,READ_ADDR=5,READ_ISSUE=6,READ_GOT=7,READ_STOP=8,READ_END=9,
 WRITE_START=10,WRITE_ADDR=11,WRITE_ISSUE=12,WRITE_NEXT=13,
 WRITE_STOP=14,WRITE_END=15,FAIL_STOP=16,FAIL_RESET=17;
reg [4:0] state,return_state;
reg reset_low;
reg [31:0] timer,age;
(* ASYNC_REG="TRUE" *) reg [1:0] int_sync;
reg go;
reg [1:0] op;
reg [7:0] tx;
reg nack;
wire busy,done,bus_error,ack;
reg [3:0] event_pulse;
wire parser_bad;
// Pulse per occurrence, including repeated identical errors.
// [0] NACK [1] bus timeout/error [2] header [3] report retry [4] parser.
assign debug_events={parser_bad,event_pulse};
wire [7:0] rx;
// Read the header and cargo in ONE START...STOP transaction. The FPGA can
// extend read_length after byte 1, before choosing ACK/NACK for byte 3.
// A separate header peek can change continuation/sequence fields in SHTP.
reg bad_header;
reg [14:0] read_length,index;
reg [7:0] header [0:3];
reg [7:0] tx_seq;
reg parse_start,parse_enable,parse_byte;
reg [7:0] parse_data;
assign sensor_rst_n=reset_low?1'b0:1'bz;
assign online=configured && age<STALE_CYCLES;
imu_i2c_byte #(.CLK_HZ(CLK_HZ),.BUS_HZ(BUS_HZ),.TIMEOUT_CYCLES(I2C_TIMEOUT)) bus(
 .clk(clk),.rst_n(rst_n && !reset_low),.go(go),.op(op),.tx(tx),.nack(nack),
 .busy(busy),.done(done),.error(bus_error),.ack(ack),.rx(rx),.scl(scl),.sda(sda));
bno080_reports parser(.clk(clk),.rst_n(rst_n),.start(parse_start),.enable(parse_enable),
 .byte_valid(parse_byte),.data(parse_data),.valid(valid),.qi(qi),.qj(qj),.qk(qk),.qr(qr),.accuracy(accuracy),.debug_bad_record(parser_bad));
// Payload matches the supplied SparkFun setFeatureCommand: 4+17 bytes.
function [7:0] feature_byte;
 input [14:0] n;
 begin case(n)
  0:feature_byte=21; 2:feature_byte=2; 3:feature_byte=tx_seq;
  4:feature_byte=8'hfd; 5:feature_byte=8'h05;
  9:feature_byte=8'h10; 10:feature_byte=8'h27; // 10000 us, LE
  default:feature_byte=0;
 endcase end
endfunction
task issue;
 input [1:0] command;
 input [7:0] value;
 input final_byte;
 input [4:0] next_state;
 begin op<=command;tx<=value;nack<=final_byte;go<=1;
  return_state<=next_state;state<=WAIT_BYTE;end
endtask
always @(posedge clk or negedge rst_n)
 if(!rst_n) int_sync<=3;
 else int_sync<={int_sync[0],int_n};
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  state<=RESET;return_state<=RESET;reset_low<=1;timer<=0;age<=RETRY_CYCLES;
  go<=0;op<=0;tx<=0;nack<=0;configured<=0;reports<=0;last_error<=0;event_pulse<=0;
  bad_header<=0;read_length<=4;index<=0;tx_seq<=0;
  header[0]<=0;header[1]<=0;header[2]<=0;header[3]<=0;
  parse_start<=0;parse_enable<=0;parse_byte<=0;parse_data<=0;
 end else begin
  go<=0;parse_start<=0;parse_byte<=0;event_pulse<=0;
  if(age<RETRY_CYCLES) age<=age+1'b1;
  if(valid) begin age<=0;reports<=reports+1'b1;last_error<=0;end
  case(state)
   RESET: begin
    reset_low<=1;configured<=0;tx_seq<=0;
    parse_start<=1;parse_enable<=0;
    if(timer==RESET_CYCLES-1) begin timer<=0;reset_low<=0;state<=BOOT;end
    else timer<=timer+1'b1;
   end
   BOOT: if(timer==BOOT_CYCLES-1) begin timer<=0;age<=STALE_CYCLES;state<=POLL;end
    else timer<=timer+1'b1;
   POLL: begin
    if(age>=RETRY_CYCLES) begin last_error<=5;timer<=0;state<=RESET;event_pulse[3]<=1;end
    else if(!int_sync[1]) begin
     bad_header<=0;index<=0;read_length<=4;state<=READ_START;
     parse_start<=1;parse_enable<=0;
    end else if(!configured) begin index<=0;state<=WRITE_START;end
   end
   WAIT_BYTE: if(done) begin
    if(bus_error) begin last_error<=2;timer<=0;state<=RESET;event_pulse[1]<=1;end
    else if(op==1 && !ack) begin last_error<=1;state<=FAIL_STOP;event_pulse[0]<=1;end
    else state<=return_state;
   end
   READ_START: issue(0,0,0,READ_ADDR);
   READ_ADDR: issue(1,{ADDRESS,1'b1},0,READ_ISSUE);
   READ_ISSUE: issue(2,0,index==read_length-1'b1,READ_GOT);
   READ_GOT: begin
    if(index<4) begin
     header[index[1:0]]<=rx;
     if(index==1) begin
      if({rx[6:0],header[0]}>=4 && {rx[6:0],header[0]}<=1024)
       read_length<={rx[6:0],header[0]};
      else if({rx[6:0],header[0]}!=0) begin bad_header<=1;last_error<=3;event_pulse[2]<=1;end
     end
     if(index==3) begin
      // Empty header is a normal polling race. Other channels are drained.
      if(read_length>4 && header[2]>5) begin bad_header<=1;last_error<=4;event_pulse[2]<=1;end
      parse_start<=1;
      parse_enable<=configured && !bad_header && read_length>4 &&
                    !header[1][7] && (header[2]==3 || header[2]==4);
     end
    end else if(!bad_header) begin parse_byte<=1;parse_data<=rx;end
    if(index==read_length-1'b1) state<=READ_STOP;
    else begin index<=index+1'b1;state<=READ_ISSUE;end
   end
   READ_STOP: issue(3,0,0,READ_END);
   READ_END: begin
    if(bad_header) begin timer<=0;state<=RESET;end
    else state<=POLL;
   end
   WRITE_START: issue(0,0,0,WRITE_ADDR);
   WRITE_ADDR: issue(1,{ADDRESS,1'b0},0,WRITE_ISSUE);
   WRITE_ISSUE: issue(1,feature_byte(index),0,WRITE_NEXT);
   WRITE_NEXT: if(index==20) state<=WRITE_STOP;
    else begin index<=index+1'b1;state<=WRITE_ISSUE;end
   WRITE_STOP: issue(3,0,0,WRITE_END);
   WRITE_END: begin configured<=1;tx_seq<=tx_seq+1'b1;state<=POLL;end
   FAIL_STOP: issue(3,0,0,FAIL_RESET);
   FAIL_RESET: begin timer<=0;state<=RESET;end
   default: begin timer<=0;state<=RESET;end
  endcase
 end
endmodule
