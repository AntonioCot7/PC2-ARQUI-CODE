module mla(rn, rm, y);
    input wire [31:0] rn, rm;
    output wire [31:0] y;
    assign y = rn * rm;
endmodule