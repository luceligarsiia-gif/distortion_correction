`timescale 1ns/1ps
// Format signed 0.1-degree value for OSD.
// packed = {sign, hundreds, tens, ones, tenths}
// sign=1 means negative.  Magnitude is clamped to 180.0 degrees.
//
// Important for PDS: this module intentionally contains no / or % operators.
// Binary-to-BCD uses the synthesizable shift-add-3 (double-dabble) algorithm.
module angle_format(
    input  wire signed [15:0] angle_deg10,
    output reg  [16:0] packed
);
    reg [15:0] mag;
    reg sign;
    reg [15:0] bcd;
    reg [26:0] shift_reg;
    integer i;

    always @* begin
        sign = angle_deg10 < 0;

        if (angle_deg10 < 0)
            mag = -angle_deg10;
        else
            mag = angle_deg10;

        if (mag > 16'd1800)
            mag = 16'd1800;

        // Convert 0..1800 (tenths of a degree) to four BCD digits.
        // Example: 123 -> 0,1,2,3 -> 12.3 degrees.
        shift_reg = 27'd0;
        shift_reg[10:0] = mag[10:0];

        for (i = 0; i < 11; i = i + 1) begin
            if (shift_reg[14:11] >= 4'd5)
                shift_reg[14:11] = shift_reg[14:11] + 4'd3;
            if (shift_reg[18:15] >= 4'd5)
                shift_reg[18:15] = shift_reg[18:15] + 4'd3;
            if (shift_reg[22:19] >= 4'd5)
                shift_reg[22:19] = shift_reg[22:19] + 4'd3;
            if (shift_reg[26:23] >= 4'd5)
                shift_reg[26:23] = shift_reg[26:23] + 4'd3;

            shift_reg = shift_reg << 1;
        end

        bcd = shift_reg[26:11];
        packed = {sign, bcd};
    end
endmodule
