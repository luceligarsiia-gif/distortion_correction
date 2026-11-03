// Streaming local illumination control. There is no pixel/frame store here.
// Default grid: 8 x 9, 256 x 128 pixels per tile; edge tiles are partial.
// Completed raw-frame statistics control the following frame. Map publication
// is atomic in vertical blanking. Per-line vertical interpolation is prepared
// in horizontal blanking; the pixel path only needs horizontal interpolation.
module local_gamma_tiles #(
 parameter integer WIDTH=1920, HEIGHT=1080,
 parameter integer COLS=8, ROWS=9, TILE_X_SHIFT=8, TILE_Y_SHIFT=7,
 parameter integer MIN_TILE_SAMPLES=16
)(input clk,rst_n,frame_boundary,in_de,
 input [7:0] luma,
 output wire [7:0] dark_left,dark_right,high_left,high_right,
 output wire [4:0] fraction_x,
 output reg map_valid,
 output reg [3:0] dark_peak,high_peak);
localparam integer N=COLS*ROWS;
reg [11:0] x,y;
reg de_d,active_started;
wire line_end=de_d && !in_de;
wire inside_image=x<WIDTH && y<HEIGHT;
wire [11:0] xoffset=x-(1<<(TILE_X_SHIFT-1));
wire [11:0] yoffset=y-(1<<(TILE_Y_SHIFT-1));
wire [3:0] left_col=x<(1<<(TILE_X_SHIFT-1))?0:
                        ((xoffset>>TILE_X_SHIFT)>=COLS-1?COLS-1:(xoffset>>TILE_X_SHIFT));
wire [3:0] right_col=left_col==COLS-1?left_col:left_col+1'b1;
assign fraction_x=(x<(1<<(TILE_X_SHIFT-1)) || left_col==COLS-1)?0:
                    ((xoffset & ((1<<TILE_X_SHIFT)-1))*32)>>TILE_X_SHIFT;
reg [7:0] line_dark[0:COLS-1],line_high[0:COLS-1];
assign dark_left=line_dark[left_col];
assign dark_right=line_dark[right_col];
assign high_left=line_high[left_col];
assign high_right=line_high[right_col];

// Explicit shift/add multiplier: do not consume another APM/DSP.
function [12:0] times5;
 input [7:0] a; input [4:0] b;
 reg [12:0] p0,p1,p2,p3,p4;
 begin
  p0=b[0]?{5'd0,a}:13'd0; p1=b[1]?{4'd0,a,1'b0}:13'd0;
  p2=b[2]?{3'd0,a,2'd0}:13'd0; p3=b[3]?{2'd0,a,3'd0}:13'd0;
  p4=b[4]?{1'd0,a,4'd0}:13'd0;
  times5=(p0+p1)+(p2+p3)+p4;
 end
endfunction
function [7:0] blend;
 input [7:0] a,b; input [4:0] fraction;
 reg [13:0] value;
 begin value=({1'b0,a,5'd0}-{1'b0,times5(a,fraction)})+
              {1'b0,times5(b,fraction)}+14'd16;
  blend=value[12:5];
 end
endfunction
function [7:0] approach;
 input [7:0] old_value,target;
 begin
  if(target>old_value) approach=(target-old_value>32)?old_value+8'd32:target;
  else approach=(old_value-target>16)?old_value-8'd16:target;
 end
endfunction

// Packed statistics RAM, not a video buffer. Single read/write ports are
// shared by sparse active-pixel updates and vertical-blank map generation.
reg [43:0] stats_mem[0:N-1] /* synthesis syn_ramstyle="block_ram" */;
reg [43:0] stats_word;
wire [19:0] sum_hold=stats_word[43:24];
wire [11:0] count_hold=stats_word[23:12],bright_hold=stats_word[11:0];
reg [7:0] dark0[0:N-1],dark1[0:N-1],high0[0:N-1],high1[0:N-1];
reg active_bank,ready,ever_captured,frame_bad;
reg [3:0] pending_dark_peak,pending_high_peak;
// The sample read/modify/write pipeline has three registers. Sample spacing
// is sixteen pixels, so no back-to-back access hazard exists for one tile.
reg sample1,sample2;
reg [7:0] sample_index1,sample_index2,sample_y1;
reg [7:0] sample_y2;
reg [2:0] scan_state;
reg [7:0] scan_index;
reg [7:0] target_dark_q,target_high_q;
reg enough_q,below8_q,below24_q,below48_q,below72_q,below96_q,below120_q,below144_q,below168_q;
reg bright4_q,bright8_q,bright32_q,bright128_q;
reg [7:0] target_dark,target_high,old_dark,old_high;
reg scan_good;
wire [7:0] smooth_dark=approach(old_dark,target_dark_q);
wire [7:0] smooth_high=approach(old_high,target_high_q);
wire [3:0] dark_units=smooth_dark[7:4];
wire [3:0] high_units=smooth_high[7:4];
// Extended before shifting: comparisons must not truncate a 12-bit count.
wire [19:0] count_ext={8'd0,count_hold};
wire [19:0] bright_ext={8'd0,bright_hold};
always @*begin
 target_dark=0;target_high=0;
 if(enough_q)begin
  // Night mode: strong gain for low-light regions, bounded at pixel level.
  // Near-zero pixels retain a gentle toe; normal regions get reduced gain.
  if(below8_q)target_dark=128;
  else if(below24_q)target_dark=128;
  else if(below48_q)target_dark=128;
  else if(below72_q)target_dark=128;
  else if(below96_q)target_dark=112;
  else if(below120_q)target_dark=80;
  else if(below144_q)target_dark=48;
  else if(below168_q)target_dark=24;
  if(bright4_q)target_high=128;
  else if(bright8_q)target_high=112;
  else if(bright32_q)target_high=96;
  else if(bright128_q)target_high=64;
 end
end

// Vertical interpolation is evaluated only during line blanking. Eight
// columns take 256 clocks plus setup, within the 280-clock 1080p blanking.
// Each smoothed coefficient = (4*center + left + right + up + down + 4)/8.
// Read the ten neighbours serially to reuse a single dark/high read port.
reg [3:0] line_state;
reg [1:0] read_phase;
reg [7:0] read_dark_q,read_high_q;
reg [3:0] line_index,row_top,row_bottom;
wire [3:0] col_before=line_index==0?0:line_index-1'b1;
wire [3:0] col_after=line_index==COLS-1?line_index:line_index+1'b1;
wire [3:0] top_before=row_top==0?0:row_top-1'b1;
wire [3:0] top_after=row_top==ROWS-1?row_top:row_top+1'b1;
wire [3:0] bottom_before=row_bottom==0?0:row_bottom-1'b1;
wire [3:0] bottom_after=row_bottom==ROWS-1?row_bottom:row_bottom+1'b1;
reg [4:0] fraction_y;
reg [7:0] va_dark,vb_dark,va_high,vb_high;
reg [10:0] top_dark_sum,top_high_sum,bottom_dark_sum,bottom_high_sum;
reg [7:0] read_index;
reg [7:0] read_index_q;
wire [7:0] read_dark=active_bank?dark1[read_index_q]:dark0[read_index_q];
wire [7:0] read_high=active_bank?high1[read_index_q]:high0[read_index_q];
always @*begin
 case(line_state)
  2,7:read_index=(line_state==2?row_top:row_bottom)*COLS+col_before;
  3,8:read_index=(line_state==3?row_top:row_bottom)*COLS+col_after;
  4:read_index=top_before*COLS+line_index;
  5:read_index=top_after*COLS+line_index;
  9:read_index=bottom_before*COLS+line_index;
  10:read_index=bottom_after*COLS+line_index;
  default:read_index=(line_state<=5?row_top:row_bottom)*COLS+line_index;
 endcase
end
reg prepare_line;
reg [11:0] prepare_y;
wire [11:0] prep_offset=prepare_y-(1<<(TILE_Y_SHIFT-1));
wire [3:0] prep_top=prepare_y<(1<<(TILE_Y_SHIFT-1))?0:
                       ((prep_offset>>TILE_Y_SHIFT)>=ROWS-1?ROWS-1:(prep_offset>>TILE_Y_SHIFT));
wire [3:0] prep_bottom=prep_top==ROWS-1?prep_top:prep_top+1'b1;
wire [4:0] prep_fraction=(prepare_y<(1<<(TILE_Y_SHIFT-1)) || prep_top==ROWS-1)?0:
                    ((prep_offset & ((1<<TILE_Y_SHIFT)-1))*32)>>TILE_Y_SHIFT;
// No asynchronous reset on memory data: initialize one word per clock.
// One synchronous read, one synchronous write infer block RAM on Logos2.
wire [19:0] sample_sum=sum_hold+{12'd0,sample_y2};
wire [11:0] sample_count=count_hold+12'd1;
wire [11:0] sample_bright=bright_hold+(sample_y2>=192?12'd1:12'd0);
always @(posedge clk)begin
 if(scan_state==5 || scan_state==4)stats_mem[scan_index]<=44'd0;
 else if(sample2)stats_mem[sample_index2]<={sample_sum,sample_count,sample_bright};
 if(scan_state==1)stats_word<=stats_mem[scan_index];
 else if(sample1)stats_word<=stats_mem[sample_index1];
end
integer i;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  x<=0;y<=0;de_d<=0;active_started<=0;active_bank<=0;map_valid<=0;
  ready<=0;ever_captured<=0;frame_bad<=0;dark_peak<=0;high_peak<=0;
  pending_dark_peak<=0;pending_high_peak<=0;sample1<=0;sample2<=0;
  sample_index1<=0;sample_index2<=0;sample_y1<=0;sample_y2<=0;
  scan_state<=5;scan_index<=0;
  target_dark_q<=0;target_high_q<=0;
  enough_q<=0;below8_q<=0;below24_q<=0;below48_q<=0;below72_q<=0;below96_q<=0;below120_q<=0;below144_q<=0;below168_q<=0;
  bright4_q<=0;bright8_q<=0;bright32_q<=0;bright128_q<=0;old_dark<=0;old_high<=0;scan_good<=0;
  line_state<=0;read_phase<=0;read_index_q<=0;read_dark_q<=0;read_high_q<=0;line_index<=0;row_top<=0;row_bottom<=0;fraction_y<=0;
  va_dark<=0;vb_dark<=0;va_high<=0;vb_high<=0;
  top_dark_sum<=0;top_high_sum<=0;bottom_dark_sum<=0;bottom_high_sum<=0;
  prepare_line<=0;prepare_y<=0;
  for(i=0;i<N;i=i+1)begin
   dark0[i]<=0;dark1[i]<=0;high0[i]<=0;high1[i]<=0;
  end
  for(i=0;i<COLS;i=i+1)begin line_dark[i]<=0;line_high[i]<=0;end
 end else begin
  de_d<=in_de;
  prepare_line<=0;
  if(frame_boundary)begin
   x<=0;y<=0;active_started<=0;frame_bad<=0;ever_captured<=1;
   // An unexpectedly early boundary invalidates the entire control map.
   if((scan_state!=0 && scan_state!=5) || sample1 || sample2)begin frame_bad<=1;map_valid<=0;end
   if(scan_state==0)begin
    scan_state<=1;scan_index<=0;scan_good<=ever_captured && !frame_bad;
    pending_dark_peak<=0;pending_high_peak<=0;
   end
   prepare_line<=1;prepare_y<=0;
  end else begin
   if(in_de)begin x<=x+1'b1;active_started<=1;end
   else x<=0;
   if(line_end)begin y<=y+1'b1;prepare_line<=1;prepare_y<=y+1'b1;end
  end
  if(in_de && (scan_state!=0 || !inside_image))frame_bad<=1;
  sample1<=in_de && inside_image && scan_state==0 && x[3:0]==4'd8;
  if(in_de && inside_image && scan_state==0 && x[3:0]==4'd8)begin
   sample_index1<=(y>>TILE_Y_SHIFT)*COLS+(x>>TILE_X_SHIFT);sample_y1<=luma;
  end
  sample2<=sample1;
  if(sample1)begin
   sample_index2<=sample_index1;
   sample_y2<=sample_y1;
  end
  case(scan_state)
   1:begin
    old_dark<=active_bank?dark1[scan_index]:dark0[scan_index];
    old_high<=active_bank?high1[scan_index]:high0[scan_index];scan_state<=2;
   end
   2:begin
    // Register wide comparisons separately from priority selection and slew.
    enough_q<=count_hold>=MIN_TILE_SAMPLES;
    below8_q<=sum_hold<=(count_ext<<3);
    below24_q<=sum_hold<(count_ext<<4)+(count_ext<<3);
    below48_q<=sum_hold<(count_ext<<5)+(count_ext<<4);
    below72_q<=sum_hold<(count_ext<<6)+(count_ext<<3);
    below96_q<=sum_hold<(count_ext<<6)+(count_ext<<5);
    below120_q<=sum_hold<(count_ext<<7)-(count_ext<<3);
    below144_q<=sum_hold<(count_ext<<7)+(count_ext<<4);
    below168_q<=sum_hold<(count_ext<<7)+(count_ext<<5)+(count_ext<<3);
    bright4_q<=(bright_ext<<2)>=count_ext;
    bright8_q<=(bright_ext<<3)>=count_ext;
    bright32_q<=(bright_ext<<5)>=count_ext;
    bright128_q<=(bright_ext<<7)>=count_ext;
    scan_state<=3;
   end
   3:begin
    target_dark_q<=target_dark;target_high_q<=target_high;scan_state<=4;
   end
   4:begin
    if(count_hold<MIN_TILE_SAMPLES)scan_good<=0;
    if(active_bank)begin dark0[scan_index]<=smooth_dark;high0[scan_index]<=smooth_high;end
    else begin dark1[scan_index]<=smooth_dark;high1[scan_index]<=smooth_high;end
    if(dark_units>pending_dark_peak)pending_dark_peak<=dark_units;
    if(high_units>pending_high_peak)pending_high_peak<=high_units;
    if(scan_index==N-1)begin scan_state<=0;ready<=scan_good && count_hold>=MIN_TILE_SAMPLES;end
    else begin scan_index<=scan_index+1'b1;scan_state<=1;end
   end
   5:begin
    if(scan_index==N-1)begin scan_index<=0;scan_state<=0;end
    else scan_index<=scan_index+1'b1;
   end
   default:;
  endcase
  // Do not publish after the first active pixel: one frame uses one map.
  if(ready && !active_started && !in_de && !frame_boundary)begin
   active_bank<=~active_bank;map_valid<=1;ready<=0;
   dark_peak<=pending_dark_peak;high_peak<=pending_high_peak;
   prepare_line<=1;prepare_y<=0;
  end
  if(in_de)begin line_state<=0;read_phase<=0;end
  else if(prepare_line)begin
   row_top<=prep_top;row_bottom<=prep_bottom;fraction_y<=prep_fraction;
   line_index<=0;line_state<=1;read_phase<=0;
  end else if(line_state>=1 && line_state<=10 && read_phase==0)begin
   read_index_q<=read_index;read_phase<=1;
  end else if(line_state>=1 && line_state<=10 && read_phase==1)begin
   read_dark_q<=read_dark;read_high_q<=read_high;read_phase<=2;
  end else begin
   read_phase<=0;
   case(line_state)
   1:begin
    top_dark_sum<={1'b0,read_dark_q,2'b0};
    top_high_sum<={1'b0,read_high_q,2'b0};
    line_state<=2;
   end
   2,3,4,5:begin
    top_dark_sum<=top_dark_sum+{3'd0,read_dark_q};
    top_high_sum<=top_high_sum+{3'd0,read_high_q};
    line_state<=line_state+1'b1;
   end
   6:begin
    bottom_dark_sum<={1'b0,read_dark_q,2'b0};
    bottom_high_sum<={1'b0,read_high_q,2'b0};
    line_state<=7;
   end
   7,8,9,10:begin
    bottom_dark_sum<=bottom_dark_sum+{3'd0,read_dark_q};
    bottom_high_sum<=bottom_high_sum+{3'd0,read_high_q};
    line_state<=line_state+1'b1;
   end
   11:begin
    va_dark<=(top_dark_sum+11'd4)>>3;
    va_high<=(top_high_sum+11'd4)>>3;
    vb_dark<=(bottom_dark_sum+11'd4)>>3;
    vb_high<=(bottom_high_sum+11'd4)>>3;
    line_state<=12;
   end
   12:begin
    line_dark[line_index]<=blend(va_dark,vb_dark,fraction_y);
    line_high[line_index]<=blend(va_high,vb_high,fraction_y);
    if(line_index==COLS-1)line_state<=0;
    else begin line_index<=line_index+1'b1;line_state<=1;end
   end
   default:;
  endcase
  end
 end
endmodule
