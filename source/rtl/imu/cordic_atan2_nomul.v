`timescale 1ns/1ps
// Iterative CORDIC vectoring atan2.
// x/y may use any common fixed-point scale.  Angle output is 0.1 degree.
// magnitude is the CORDIC-gain-scaled vector magnitude (K ~= 1.64676).
module cordic_atan2_nomul #(
    parameter W = 40
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire signed [W-1:0]     x_in,
    input  wire signed [W-1:0]     y_in,
    output reg                     busy,
    output reg                     done,
    output reg signed [15:0]       angle_deg10,
    output reg signed [W-1:0]      magnitude
);

    reg signed [W-1:0] x_reg, y_reg;
    reg signed [31:0] angle_cdeg; // centi-degree, keeps CORDIC rounding error small
    reg [4:0] iter;

    wire signed [W-1:0] x_shift = x_reg >>> iter;
    wire signed [W-1:0] y_shift = y_reg >>> iter;

    reg signed [W-1:0] x_next, y_next;
    reg signed [31:0] angle_next;

    // PDS does not map the Verilog signed divide operator used for /10.
    // Convert signed centi-degrees to signed deci-degrees without a divider.
    // For 0..18000, floor(n/10) == (n * 26215) >> 18 exactly.
    wire        angle_neg = angle_next[31];
    wire [31:0] angle_abs = angle_neg ? (~angle_next + 32'd1) : angle_next;
    // 26215 = (2^15-2^13)+(2^11-2^9)+(2^7-2^5)+(2^3-1).
    // Exact constant multiplication using only shifts/adds, no DSP multiplier.
    wire [46:0] angle_wide = {15'd0,angle_abs};
    wire [46:0] angle_div10_product =
        ((angle_wide << 15) - (angle_wide << 13)) +
        ((angle_wide << 11) - (angle_wide << 9)) +
        ((angle_wide << 7) - (angle_wide << 5)) +
        ((angle_wide << 3) - angle_wide);
    wire [31:0] angle_div10_abs = angle_div10_product >> 18;
    wire signed [31:0] angle_div10 = angle_neg ? -$signed(angle_div10_abs)
                                                :  $signed(angle_div10_abs);

    function signed [31:0] atan_cdeg;
        input [4:0] idx;
        begin
            case (idx)
                5'd0 : atan_cdeg = 32'sd4500;
                5'd1 : atan_cdeg = 32'sd2657;
                5'd2 : atan_cdeg = 32'sd1404;
                5'd3 : atan_cdeg = 32'sd713;
                5'd4 : atan_cdeg = 32'sd358;
                5'd5 : atan_cdeg = 32'sd179;
                5'd6 : atan_cdeg = 32'sd90;
                5'd7 : atan_cdeg = 32'sd45;
                5'd8 : atan_cdeg = 32'sd22;
                5'd9 : atan_cdeg = 32'sd11;
                5'd10: atan_cdeg = 32'sd6;
                5'd11: atan_cdeg = 32'sd3;
                5'd12: atan_cdeg = 32'sd1;
                5'd13: atan_cdeg = 32'sd1;
                default: atan_cdeg = 32'sd0;
            endcase
        end
    endfunction

    always @* begin
        x_next = x_reg;
        y_next = y_reg;
        angle_next = angle_cdeg;
        if (y_reg > 0) begin
            x_next = x_reg + y_shift;
            y_next = y_reg - x_shift;
            angle_next = angle_cdeg + atan_cdeg(iter);
        end else if (y_reg < 0) begin
            x_next = x_reg - y_shift;
            y_next = y_reg + x_shift;
            angle_next = angle_cdeg - atan_cdeg(iter);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy        <= 1'b0;
            done        <= 1'b0;
            angle_deg10 <= 16'sd0;
            magnitude   <= {W{1'b0}};
            x_reg       <= {W{1'b0}};
            y_reg       <= {W{1'b0}};
            angle_cdeg  <= 32'sd0;
            iter        <= 5'd0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                busy <= 1'b1;
                iter <= 5'd0;
                // CORDIC vectoring converges cleanly with x >= 0.  Pre-rotate
                // vectors in quadrants II/III by 180 degrees.
                if (x_in < 0) begin
                    x_reg <= -x_in;
                    y_reg <= -y_in;
                    angle_cdeg <= (y_in >= 0) ? 32'sd18000 : -32'sd18000;
                end else begin
                    x_reg <= x_in;
                    y_reg <= y_in;
                    angle_cdeg <= 32'sd0;
                end
            end else if (busy) begin
                x_reg      <= x_next;
                y_reg      <= y_next;
                angle_cdeg <= angle_next;

                if (iter == 5'd15) begin
                    busy        <= 1'b0;
                    done        <= 1'b1;
                    // Divider-free centi-degree -> 0.1-degree conversion.
                    angle_deg10 <= angle_div10[15:0];
                    magnitude   <= x_next;
                end else begin
                    iter <= iter + 1'b1;
                end
            end
        end
    end
endmodule
