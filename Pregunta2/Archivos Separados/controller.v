module controller (
    clk,
    reset,
    Instr,
    ALUFlags,
    RegSrc,
    RegWrite,
    ImmSrc,
    ALUSrc,
    ALUControl,
    MemWrite,
    MemtoReg,
    PCSrc,
    DMSrc,
    Shift,
    Div,
    Mul,
    Div_sel,
    Instrnew
);
    input wire clk;
    input wire reset;
    input wire [31:12] Instr;
    input wire [3:0] ALUFlags;
    output wire [1:0] RegSrc;
    output wire RegWrite;
    output wire [1:0] ImmSrc;
    output wire ALUSrc;
    output wire [2:0] ALUControl;
    output wire MemWrite;
    output wire MemtoReg;
    output wire PCSrc;
    output wire DMSrc;
    output wire Shift;    // shift
    output wire Div;      // Div
    output wire Div_sel;
    output wire Mul;      // Mul
    input wire [3:0] Instrnew;

    wire [1:0] FlagW;
    wire PCS;
    wire RegW;
    wire MemW;

    decode dec(
        .Op(Instr[27:26]),
        .Funct(Instr[25:20]),
        .Rd(Instr[15:12]),
        .FlagW(FlagW),
        .PCS(PCS),
        .RegW(RegW),
        .MemW(MemW),
        .MemtoReg(MemtoReg),
        .ALUSrc(ALUSrc),
        .ImmSrc(ImmSrc),
        .RegSrc(RegSrc),
        .ALUControl(ALUControl),
        .Shift(Shift),
        .Div(Div),
        .Mul(Mul),
        .DMSrc(DMSrc),
        .Div_sel(Div_sel),
        .Instrnew(Instrnew)
    );

    condlogic cl(
        .clk(clk),
        .reset(reset),
        .Cond(Instr[31:28]),
        .ALUFlags(ALUFlags),
        .FlagW(FlagW),
        .PCS(PCS),
        .RegW(RegW),
        .MemW(MemW),
        .PCSrc(PCSrc),
        .RegWrite(RegWrite),
        .MemWrite(MemWrite)
    );
endmodule