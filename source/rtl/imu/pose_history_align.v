// Timestamp composed poses in the DDR clock domain. Query once per input frame,
// NOT when rendering/displaying it. Positive delay selects an EARLIER pose.
// 32 reports (normally 320 ms) cover the configurable 0..150 ms offset.
// Interpolate coefficients between bracketing reports using one 17x17 product.
// Signed modular timestamps handle the 32-bit clock-counter wrap.
module pose_history_align #(
 parameter integer TICKS_PER_MS=125000, EXTERNAL_FRAME_TIME=0
)(input clk,rst_n,sample_valid,input [287:0] sample_matrix,
 input sample_crop,sample_active,
 input frame_start,input [1:0] frame_slot,input [7:0] delay_ms,
 output reg frame_valid,output reg [1:0] out_slot,
 output reg [287:0] frame_matrix,output reg frame_crop,
 output reg clamped,output wire busy,
 input [31:0] frame_time,output wire [31:0] time_now);
localparam [287:0] IDENTITY={32'd16384,32'd0,32'd0,32'd0,32'd16384,32'd0,32'd0,32'd0,32'd16384};
localparam IDLE=0,READ_WAIT=1,CHECK=2,DIVIDE=3,MUL_ISSUE=4,MUL_SAVE=5;
reg [2:0] state;
reg [31:0] clock_ticks,target;
reg [319:0] history[0:31];
reg [319:0] read_data;
reg [4:0] write_ptr,read_ptr;
reg [5:0] count,remaining;
reg [287:0] latest,newer,older,assembled;
reg latest_crop,latest_active,have_newer;
reg [31:0] newer_time;
reg [46:0] numerator,quotient;
reg [32:0] remainder;
reg [31:0] denominator;
reg [5:0] div_bit;
reg [16:0] fraction;
reg [3:0] cell_id;
reg signed [33:0] product;
wire [31:0] record_time=read_data[319:288];
wire [287:0] record_matrix=read_data[287:0];
wire signed [31:0] target_distance=$signed(target-record_time);
wire [32:0] shifted_remainder={remainder[31:0],numerator[div_bit]};
wire take_bit=shifted_remainder>={1'b0,denominator};
wire [46:0] quotient_next=quotient|(take_bit?(47'd1<<div_bit):47'd0);
wire signed [31:0] difference=$signed(newer[cell_id*32+:32])-$signed(older[cell_id*32+:32]);
wire signed [16:0] multiplier_a=difference[16:0];
wire signed [16:0] multiplier_b=$signed(fraction);
wire signed [34:0] rounded_product={{1{product[33]}},product}+35'sd16384;
wire signed [31:0] interpolated=$signed(older[cell_id*32+:32])+(rounded_product>>>15);
assign busy=state!=IDLE;
assign time_now=clock_ticks;
// No asynchronous resets on the memory port: infer block RAM, not flip-flops.
always @(posedge clk)begin
 read_data<=history[read_ptr];
 if(sample_valid && sample_active)
    history[(latest_active && sample_crop==latest_crop)?write_ptr:5'd0]<={clock_ticks,sample_matrix};
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  state<=IDLE;clock_ticks<=0;target<=0;write_ptr<=0;read_ptr<=0;count<=0;remaining<=0;
  latest<=IDENTITY;newer<=IDENTITY;older<=IDENTITY;assembled<=IDENTITY;
  latest_crop<=0;latest_active<=0;have_newer<=0;newer_time<=0;
  numerator<=0;quotient<=0;remainder<=0;denominator<=1;div_bit<=0;fraction<=0;
  cell_id<=0;product<=0;frame_valid<=0;out_slot<=0;frame_matrix<=IDENTITY;frame_crop<=0;clamped<=0;
 end else begin
  clock_ticks<=clock_ticks+1'b1;frame_valid<=0;
  if(sample_valid)begin
   latest<=sample_matrix;latest_crop<=sample_crop;latest_active<=sample_active;
   if(!sample_active)begin count<=0;write_ptr<=0;end
   else if(!latest_active || sample_crop!=latest_crop)begin count<=1;write_ptr<=1;end
   else begin write_ptr<=write_ptr+1'b1;if(count<32)count<=count+1'b1;end
  end
  case(state)
   IDLE:if(frame_start)begin
    out_slot<=frame_slot;frame_crop<=sample_valid?sample_crop:latest_crop;clamped<=0;
    if((delay_ms==0 && !EXTERNAL_FRAME_TIME) || !latest_active || count<2 ||
       (sample_valid && (sample_active!=latest_active || sample_crop!=latest_crop)))begin
     frame_matrix<=sample_valid?sample_matrix:latest;frame_valid<=1;
     clamped<=delay_ms!=0 && (sample_valid?sample_active:latest_active) && count<2;
    end else begin
     target<=(EXTERNAL_FRAME_TIME?frame_time:clock_ticks)-delay_ms*TICKS_PER_MS;
     read_ptr<=write_ptr-1'b1;remaining<=count;have_newer<=0;state<=READ_WAIT;
    end
   end
   READ_WAIT:state<=CHECK;
   CHECK:begin
    if(target_distance>=0)begin
     if(!have_newer)begin
      frame_matrix<=record_matrix;frame_valid<=1;clamped<=1;state<=IDLE;
     end else begin
      older<=record_matrix;assembled<=record_matrix;
      numerator<={target_distance[31:0],15'd0};denominator<=newer_time-record_time;
      quotient<=0;remainder<=0;div_bit<=46;state<=DIVIDE;
     end
    end else if(remaining==1)begin
     frame_matrix<=record_matrix;frame_valid<=1;clamped<=1;state<=IDLE;
    end else begin
     newer<=record_matrix;newer_time<=record_time;have_newer<=1;
     read_ptr<=read_ptr-1'b1;remaining<=remaining-1'b1;state<=READ_WAIT;
    end
   end
   DIVIDE:begin
    remainder<=take_bit?(shifted_remainder-{1'b0,denominator}):shifted_remainder;
    quotient<=quotient_next;
    if(div_bit==0)begin fraction<=quotient_next[16:0];cell_id<=0;state<=MUL_ISSUE;end
    else div_bit<=div_bit-1'b1;
   end
   MUL_ISSUE:begin product<=multiplier_a*multiplier_b;state<=MUL_SAVE;end
   MUL_SAVE:begin
    assembled[cell_id*32+:32]<=interpolated;
    if(cell_id==8)begin
     frame_matrix<={interpolated,assembled[255:0]};frame_valid<=1;state<=IDLE;
    end else begin cell_id<=cell_id+1'b1;state<=MUL_ISSUE;end
   end
   default:state<=IDLE;
  endcase
  // Never blend matrices from different S5 references, or through disconnect.
  // The CDC payload includes active status, so epoch boundaries are atomic.
  if(state!=IDLE && sample_valid && (!sample_active || sample_active!=latest_active || sample_crop!=latest_crop))begin
   frame_matrix<=sample_matrix;frame_crop<=sample_crop;frame_valid<=1;clamped<=1;state<=IDLE;
  end
 end
endmodule

// Capture the target on input-slot allocation; resolve after frame completion.
// This permits bracketing interpolation at SYNC=0 without using render time.
// This is FPGA arrival-time alignment, NOT camera exposure/IMU hardware sync.
module frame_pose_match #(
 parameter integer TICKS_PER_MS=125000,EARLY_QUERY=0
)(input clk,rst_n,sample_valid,input [287:0] sample_matrix,
 input sample_crop,sample_active,
 input frame_start,input [1:0] frame_slot,input [7:0] delay_ms,
 input source_valid,input [1:0] source_slot,read_slot,
 output source_ready,output [287:0] read_matrix,output read_crop);
localparam [287:0] IDENTITY={32'd16384,32'd0,32'd0,32'd0,32'd16384,32'd0,32'd0,32'd0,32'd16384};
reg [287:0] matrices[0:3];
reg [31:0] stamps[0:3];
reg [7:0] delays[0:3],epochs[0:3],generations[0:3];
reg [3:0] crops,ready;
reg [7:0] epoch,query_epoch,query_generation;
reg active_d,crop_d;
wire epoch_change=sample_valid && (sample_active!=active_d || sample_crop!=crop_d);
wire [7:0] effective_epoch=epoch+{7'd0,epoch_change};
wire [31:0] time_now;
wire busy,result_valid;
wire [1:0] result_slot;
wire [287:0] result_matrix;
reg [31:0] latest_report_time;
reg have_report;
wire query;
wire [1:0] history_slot;
wire [7:0] history_delay,history_epoch,history_generation;
wire [31:0] history_stamp;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin latest_report_time<=0;have_report<=0;end
 else if(sample_valid)begin latest_report_time<=time_now;have_report<=sample_active;end
generate if(EARLY_QUERY)begin: early_request
 localparam QIDLE=0,QTARGET=1,QWAIT=2,QRUN=3;
 reg [1:0] state,slot_q;
 reg [7:0] delay_q,epoch_q,generation_q;
 reg [31:0] stamp_q,target_q,report_age;
 reg expired,pulse;
 wire canceled=generations[slot_q]!=generation_q || (frame_start && frame_slot==slot_q);
 wire bracket_ready=!sample_active ||
  (have_report && $signed(latest_report_time-target_q)>=0) || expired;
 assign query=pulse;
 assign history_slot=slot_q;assign history_delay=delay_q;assign history_stamp=stamp_q;
 assign history_epoch=epoch_q;assign history_generation=generation_q;
 // Separate slot lookup, target calculation, eligibility, and publication.
 // The history's wide matrix enables are driven by one local registered pulse.
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin report_age<=0;expired<=0;end
  else if(sample_valid)begin report_age<=0;expired<=0;end
  else begin report_age<=time_now-latest_report_time;expired<=report_age>=20*TICKS_PER_MS;end
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
   state<=QIDLE;slot_q<=0;delay_q<=0;epoch_q<=0;generation_q<=0;
   stamp_q<=0;target_q<=0;pulse<=0;
  end else begin
   pulse<=0;
   case(state)
    QIDLE:if(source_valid && !ready[source_slot] && !busy && !result_valid && !frame_start)begin
     slot_q<=source_slot;delay_q<=delays[source_slot];stamp_q<=stamps[source_slot];
     generation_q<=generations[source_slot];epoch_q<=effective_epoch;state<=QTARGET;
    end
    QTARGET:begin target_q<=stamp_q-delay_q*TICKS_PER_MS;state<=QWAIT;end
    QWAIT:if(bracket_ready)begin pulse<=1;state<=QRUN;end
    QRUN:if(result_valid)state<=QIDLE;
   endcase
   if(state!=QIDLE && canceled)begin state<=QIDLE;pulse<=0;end
  end
end else begin: complete_request
 assign query=source_valid && !ready[source_slot] && !busy && !result_valid && !frame_start;
 assign history_slot=source_slot;assign history_delay=delays[source_slot];assign history_stamp=stamps[source_slot];
 assign history_epoch=effective_epoch;assign history_generation=generations[source_slot];
end endgenerate
assign source_ready=ready[source_slot] && !(frame_start && frame_slot==source_slot);
assign read_matrix=matrices[read_slot];
assign read_crop=crops[read_slot];
pose_history_align #(.TICKS_PER_MS(TICKS_PER_MS),.EXTERNAL_FRAME_TIME(1)) history(
 .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),.sample_matrix(sample_matrix),
 .sample_crop(sample_crop),.sample_active(sample_active),
 .frame_start(query),.frame_slot(history_slot),.delay_ms(history_delay),
 .frame_valid(result_valid),.out_slot(result_slot),.frame_matrix(result_matrix),
 .frame_crop(),.clamped(),.busy(busy),
 .frame_time(history_stamp),.time_now(time_now));
integer i;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  crops<=0;ready<=0;epoch<=0;query_epoch<=0;query_generation<=0;active_d<=0;crop_d<=0;
  for(i=0;i<4;i=i+1)begin
   matrices[i]<=IDENTITY;stamps[i]<=0;delays[i]<=0;epochs[i]<=0;generations[i]<=0;
  end
 end else begin
  if(sample_valid)begin active_d<=sample_active;crop_d<=sample_crop;end
  if(epoch_change)epoch<=effective_epoch;
  if(query)begin query_epoch<=history_epoch;query_generation<=history_generation;end
  if(result_valid && generations[result_slot]==query_generation)begin
   // Never reinterpret an old image using a new reference/health epoch.
   // Across an epoch boundary keep that frame's captured pose as fallback.
   if(epochs[result_slot]==query_epoch && query_epoch==effective_epoch && sample_active)
    matrices[result_slot]<=result_matrix;
   ready[result_slot]<=1;
  end
  // Slot reuse takes priority over any old result on this clock edge.
  if(frame_start)begin
   stamps[frame_slot]<=time_now;delays[frame_slot]<=delay_ms;
   epochs[frame_slot]<=effective_epoch;generations[frame_slot]<=generations[frame_slot]+1'b1;
   matrices[frame_slot]<=sample_matrix;crops[frame_slot]<=sample_crop;
   // Match every active geometric mode; perspective uses the normal crop bit.
   ready[frame_slot]<=!sample_active;
  end
 end
endmodule
