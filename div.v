module signed_divider(rn, rm, y);
    input wire signed [31:0] rn, rm;
    output wire signed [31:0] y;
    assign y = rn / rm;
endmodule

module abs_value(value, abs_val);
    input wire signed [31:0] value;
    output wire [31:0] abs_val;
    assign abs_val = (value < 0) ? -value : value;
endmodule

module unsigned_divider(rn, rm, y);
    input wire [31:0] rn, rm;
    output wire [31:0] y;
    wire [31:0] abs_rn, abs_rm;

    abs_value av_rn(rn, abs_rn);
    abs_value av_rm(rm, abs_rm);

    assign y = (abs_rm != 0 && abs_rn != 0) ? (abs_rn / abs_rm) : 0;
endmodule

module divider_unit(rn, rm, mode, y);
    input wire [31:0] rn, rm;
    input wire mode;
    output wire [31:0] y;

    wire [31:0] signed_result, unsigned_result;
    signed_divider sd(rn, rm, signed_result);
    unsigned_divider ud(rn, rm, unsigned_result);

    assign y = (mode == 1) ? unsigned_result : signed_result;
endmodule