module datapath (
    input wire clk,
    input wire reset,
    input wire [1:0] RegSrc,
    input wire RegWrite,
    input wire [1:0] ImmSrc,
    input wire ALUSrc,
    input wire [2:0] ALUControl,
    input wire MemtoReg,
    input wire PCSrc,
    input wire DMSrc,
    input wire Shift,
    input wire Div,
    input wire Mul,
    input wire Div_sel,
    output wire [3:0] ALUFlags,
    output wire [31:0] PC,
    input wire [31:0] Instr,
    output wire [31:0] ALUResult,
    output wire [31:0] WriteData,
    input wire [31:0] ReadData
);
    wire [31:0] PCNext;
    wire [31:0] PCPlus4;
    wire [31:0] ExtImm;
    wire [31:0] SrcA;
    wire [31:0] SrcB;
    wire [31:0] Result;
    wire [31:0] MULResult;
    wire [31:0] DIVResult;

    mux2 #(32) pcmux(
        .d0(PCPlus4),
        .d1(Result),
        .s(PCSrc),
        .y(PCNext)
    );

    flopr #(32) pcreg(
        .clk(clk),
        .reset(reset),
        .d(PCNext),
        .q(PC)
    );

    adder #(32) pcadd1(
        .a(PC),
        .b(32'b100),
        .y(PCPlus4)
    );

    regfile rf(
        .clk(clk),
        .we3(RegWrite),
        .ra1(Instr[19:16]),
        .ra2(Instr[3:0]),
        .wa3(Instr[15:12]),
        .wd3(Result),
        .r15(PCPlus4),
        .rd1(SrcA),
        .rd2(WriteData)
    );

    extend ext(
        .Instr(Instr[23:0]),
        .ImmSrc(ImmSrc),
        .ExtImm(ExtImm)
    );

    alu alu1(
        .a(SrcA),
        .b(SrcB),
        .ALUControl(ALUControl),
        .Result(ALUResult),
        .ALUFlags(ALUFlags)
    );

    divider_unit div1(
        .rn(SrcA),
        .rm(SrcB),
        .mode(Div_sel),
        .y(DIVResult)
    );

    mla mla1(
        .rn(SrcA),
        .rm(SrcB),
        .y(MULResult)
    );

    mux2 #(32) srcbmux(
        .d0(WriteData),
        .d1(ExtImm),
        .s(ALUSrc),
        .y(SrcB)
    );

    mux2 #(32) resmux(
        .d0(ALUResult),
        .d1(ReadData),
        .s(MemtoReg),
        .y(Result)
    );
endmodule