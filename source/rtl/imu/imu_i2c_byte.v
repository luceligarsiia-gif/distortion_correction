`timescale 1ns/1ps
// Open-drain, single-master I2C byte engine. clk is NOT an I2C clock.
// op: 0 START, 1 WRITE (ack=1 when slave ACKs), 2 READ, 3 STOP.
// READ uses nack=1 for the final byte. A new command is accepted when !busy.
// Both input pins are synchronized; EVERY SCL-high phase waits for stretching.
module imu_i2c_byte #(
 parameter CLK_HZ=27000000, parameter BUS_HZ=100000,
 parameter TIMEOUT_CYCLES=CLK_HZ/20
)(input clk,rst_n,input go,input [1:0] op,input [7:0] tx,input nack,
 output reg busy,done,error,ack,output reg [7:0] rx,inout scl,sda);
localparam HALF=(CLK_HZ/(2*BUS_HZ)<4)?4:CLK_HZ/(2*BUS_HZ);
// Keep SDA unchanged briefly AFTER each falling SCL edge, so FPGA output
// skew cannot create a false START/STOP at a byte/ACK boundary.
localparam HOLD=(CLK_HZ/2000000<2)?2:CLK_HZ/2000000;
localparam IDLE=0,START_WAIT=1,START_HOLD=2,START_LOW=3,
 BIT_LOW=4,BIT_WAIT=5,BIT_HIGH=6,ACK_LOW=7,ACK_WAIT=8,ACK_HIGH=9,
 STOP_LOW=10,STOP_WAIT=11,STOP_HIGH=12,STOP_FREE=13,ACK_RELEASE=14;
reg [3:0] state;
reg scl_low,sda_low,reading,last_nack;
reg [7:0] shift;
reg [2:0] bit_no;
reg [31:0] phase,watchdog;
(* ASYNC_REG="TRUE" *) reg [1:0] scl_sync,sda_sync;
assign scl=scl_low?1'b0:1'bz;
assign sda=sda_low?1'b0:1'bz;
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin scl_sync<=3;sda_sync<=3;end
 else begin scl_sync<={scl_sync[0],scl};sda_sync<={sda_sync[0],sda};end
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  state<=IDLE;scl_low<=0;sda_low<=0;busy<=0;done<=0;error<=0;ack<=0;rx<=0;
  phase<=0;watchdog<=0;shift<=0;bit_no<=0;reading<=0;last_nack<=0;
 end else begin
  done<=0;error<=0;
  if(busy) watchdog<=watchdog+1'b1; else watchdog<=0;
  if(busy && watchdog>=TIMEOUT_CYCLES-1) begin
   state<=IDLE;scl_low<=0;sda_low<=0;busy<=0;done<=1;error<=1;phase<=0;
  end else case(state)
   IDLE: if(go) begin
    busy<=1;phase<=0;watchdog<=0;ack<=0;
    case(op)
     0: begin scl_low<=0;sda_low<=0;state<=START_WAIT;end
     1,2: begin
      reading<=op==2;last_nack<=nack;shift<=tx;bit_no<=7;
      scl_low<=1;sda_low<=(op==1 && !tx[7]);state<=BIT_LOW;
     end
     3: begin scl_low<=1;sda_low<=1;state<=STOP_LOW;end
    endcase
   end
   START_WAIT: if(scl_sync[1] && sda_sync[1]) begin
    if(phase==HALF-1) begin sda_low<=1;phase<=0;state<=START_HOLD;end
    else phase<=phase+1'b1;
   end else phase<=0;
   START_HOLD: if(phase==HALF-1) begin
    scl_low<=1;phase<=0;state<=START_LOW;
   end else phase<=phase+1'b1;
   START_LOW: if(phase==HALF-1) begin busy<=0;done<=1;state<=IDLE;end
    else phase<=phase+1'b1;
   BIT_LOW: begin
    if(phase==HOLD-1) sda_low<=!reading && !shift[7];
    if(phase==HALF-1) begin scl_low<=0;phase<=0;state<=BIT_WAIT;end
    else phase<=phase+1'b1;
   end
   BIT_WAIT: if(scl_sync[1]) begin phase<=0;state<=BIT_HIGH;end
   BIT_HIGH: if(phase==HALF-1) begin
    if(reading) shift<={shift[6:0],sda_sync[1]};
    scl_low<=1;phase<=0;
    if(bit_no==0) begin
     if(reading) rx<={shift[6:0],sda_sync[1]};
     state<=ACK_LOW;
    end else begin
     bit_no<=bit_no-1'b1;
     if(!reading) shift<={shift[6:0],1'b0};
     state<=BIT_LOW;
    end
   end else phase<=phase+1'b1;
   ACK_LOW: begin
    if(phase==HOLD-1) sda_low<=reading && !last_nack;
    if(phase==HALF-1) begin scl_low<=0;phase<=0;state<=ACK_WAIT;end
    else phase<=phase+1'b1;
   end
   ACK_WAIT: if(scl_sync[1]) begin phase<=0;state<=ACK_HIGH;end
   ACK_HIGH: if(phase==HALF-1) begin
    ack<=!sda_sync[1];scl_low<=1;phase<=0;state<=ACK_RELEASE;
   end else phase<=phase+1'b1;
   ACK_RELEASE: if(phase==HOLD-1) begin
    sda_low<=0;busy<=0;done<=1;state<=IDLE;
   end else phase<=phase+1'b1;
   STOP_LOW: if(phase==HALF-1) begin scl_low<=0;phase<=0;state<=STOP_WAIT;end
    else phase<=phase+1'b1;
   STOP_WAIT: if(scl_sync[1]) begin phase<=0;state<=STOP_HIGH;end
   STOP_HIGH: if(phase==HALF-1) begin sda_low<=0;phase<=0;state<=STOP_FREE;end
    else phase<=phase+1'b1;
   STOP_FREE: if(phase==HALF-1) begin busy<=0;done<=1;state<=IDLE;end
    else phase<=phase+1'b1;
   default: state<=IDLE;
  endcase
 end
endmodule
