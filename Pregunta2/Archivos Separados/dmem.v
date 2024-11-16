module dmem (
    clk,
    we,
    a,
    wd,
    rd
);
    input wire clk;
    input wire we;
    input wire [31:0] a;
    input wire [31:0] wd;
    output reg [31:0] rd;
    reg [31:0] RAM [63:0];

    always @(posedge clk) begin
        if (we) begin
            RAM[a[31:2]] <= wd;
        end
        rd <= RAM[a[31:2]];
    end
endmodule