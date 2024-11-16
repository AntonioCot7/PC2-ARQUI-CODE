module imem (
    a,
    rd
);
    input wire [31:0] a;
    output wire [31:0] rd;
  	reg [31:0] RAM [15:0]; // Cambiado de [63:0] a [15:0] por ejemplo

    initial $readmemh("memfile.dat", RAM, 0, 15);
    assign rd = RAM[a[31:2]];
endmodule