module top (
    clk,
    reset,
    WriteData,
    DataAdr,
    MemWrite
);
    input wire clk;
    input wire reset;
    output wire [31:0] WriteData;
    output wire [31:0] DataAdr;
    output wire MemWrite;
    wire [31:0] PC;
    wire [31:0] Instr;
    wire [31:0] ReadData;
    wire DMSrc;
    wire ShftOp;
    wire Division;
    wire Multiplication;
    wire DivMode;
    wire [3:0] NewInstr;

    arm arm_inst(
        .clk(clk),
        .reset(reset),
        .PC(PC),
        .Instr(Instr),
        .MemWrite(MemWrite),
        .ALUResult(DataAdr),
        .WriteData(WriteData),
        .ReadData(ReadData),
        .DMSrc(DMSrc),
        .ShftOp(ShftOp),
        .Division(Division),
        .Multiplication(Multiplication),
        .DivMode(DivMode),
        .NewInstr(Instr[7:4])
    );

    imem imem_inst(
        .a(PC),
        .rd(Instr)
    );

    dmem dmem_inst(
        .clk(clk),
        .we(MemWrite),
        .a(DataAdr),
        .wd(WriteData),
        .rd(ReadData)
    );
endmodule