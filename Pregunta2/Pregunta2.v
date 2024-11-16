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
module arm (
    clk,
    reset,
    PC,
    Instr,
    MemWrite,
    ALUResult,
    WriteData,
    ReadData,
    DMSrc,
    ShftOp,
    Division,
    Multiplication,
    DivMode,
    NewInstr
);
    input wire clk;
    input wire reset;
    output wire [31:0] PC;
    input wire [31:0] Instr;
    output wire MemWrite;
    output wire [31:0] ALUResult;
    output wire [31:0] WriteData;
    input wire [31:0] ReadData;
    output wire DMSrc;
    output wire ShftOp;
    output wire Division;
    output wire Multiplication;
    output wire DivMode;
    input wire [3:0] NewInstr;

    wire [3:0] ALUFlags;
    wire RegWrite;
    wire ALUSrc;
    wire MemtoReg;
    wire PCSrc;
    wire [1:0] RegSrc;
    wire [1:0] ImmSrc;
    wire [2:0] ALUControl;

    controller controller_inst(
    .clk(clk),
    .reset(reset),
    .Instr(Instr[31:12]),
    .ALUFlags(ALUFlags),
    .RegSrc(RegSrc),
    .RegWrite(RegWrite),
    .ImmSrc(ImmSrc),
    .ALUSrc(ALUSrc),
    .ALUControl(ALUControl),
    .MemWrite(MemWrite),
    .MemtoReg(MemtoReg),
    .PCSrc(PCSrc),
    .DMSrc(DMSrc),
    .Shift(ShftOp),
    .Div(Division),
    .Mul(Multiplication),
    .Div_sel(DivMode),
    .Instrnew(NewInstr)
	);

    datapath datapath_inst(
        .clk(clk),
        .reset(reset),
        .RegSrc(RegSrc),
        .RegWrite(RegWrite),
        .ImmSrc(ImmSrc),
        .ALUSrc(ALUSrc),
        .ALUControl(ALUControl),
        .MemtoReg(MemtoReg),
        .PCSrc(PCSrc),
        .ALUFlags(ALUFlags),
        .PC(PC),
        .Instr(Instr),
        .ALUResult(ALUResult),
        .WriteData(WriteData),
        .ReadData(ReadData),
        .DMSrc(DMSrc),
    	.Shift(ShftOp),
    	.Div(Division),
    	.Mul(Multiplication),
    	.Div_sel(DivMode)
    	);
endmodule
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
module decode (
    Op,
    Funct,
    Rd,
    FlagW,
    PCS,
    RegW,
    MemW,
    MemtoReg,
    ALUSrc,
    ImmSrc,
    RegSrc,
    ALUControl,
    Shift,
    Div,
    Mul,
    DMSrc,
    Div_sel,
    Instrnew
);
    input wire [1:0] Op;
    input wire [5:0] Funct;
    input wire [3:0] Rd;
    output reg [1:0] FlagW;
    output wire PCS;
    output wire RegW;
    output wire MemW;
    output wire MemtoReg;
    output wire ALUSrc;
    output wire [1:0] ImmSrc;
    output wire [1:0] RegSrc;
    output reg [2:0] ALUControl;
    output reg Shift;
    output reg Div;
    output reg Mul;
    output reg DMSrc;
    output reg Div_sel;
    input wire [3:0] Instrnew;
    reg [9:0] controls;
    wire Branch;
    wire ALUOp;

    always @(*) begin
        casex (Op)
            2'b00:
                if (Funct[5])
                    controls = 10'b0000101001;    // DPI
                else    
                    controls = 10'b0000001001;    // DPR
            2'b01:
                if (Funct[0])
                    controls = 10'b0001111000;    // LDR
                else
                    controls = 10'b1001110100;    // STR
            2'b10: controls = 10'b0110100010;     // B
            default: controls = 10'b0000000000;  // Default to zero
        endcase
    end

    assign {RegSrc, ImmSrc, ALUSrc, MemtoReg, RegW, MemW, Branch, ALUOp} = controls;

    always @(*) begin
        if (ALUOp) begin
            case (Funct[4:1])
                4'b0100: begin
                    ALUControl = 3'b000;    // add
                    Shift = 1'b0;
                    Div = 1'b0;
                    Mul = 1'b0;
                end
                4'b0010: begin 
                    ALUControl = 3'b001;    // sub
                    Shift = 1'b0;
                    Div = 1'b0;
                    Mul = 1'b0;
                end
                4'b0000: begin 
                    ALUControl = 3'b010;    // and 
                    Shift = 1'b0;
                    Div = 1'b0;
                    Mul = 1'b0;
                end
                4'b1100: begin
                    ALUControl = 3'b011;    // or
                    Shift = 1'b0;
                    Div = 1'b0;
                    Mul = 1'b0;
                end         
                4'b0001: begin
                    ALUControl = 3'b100;    // EOR
                    Shift = 1'b0;
                    Div = 1'b0;
                    Mul = 1'b0;
                end    
                4'b1101: begin
                    ALUControl = 3'b101;    // Shift
                    Shift = 1'b1;
                    Div = 1'b0;
                    Mul = 1'b0;
                end
                4'b1001: begin
                    ALUControl = 3'b110;    // DIV
                    Shift = 1'b0;
                    Div = 1'b1;
                    Mul = 1'b0;
                end
                4'b1111: begin
                    ALUControl = 3'b111;    // MLA
                    Shift = 1'b0;
                    Div = 1'b0;
                    Mul = 1'b1;
                end
                default: begin 
                    ALUControl = 3'b000;
                    Shift = 1'b0;
                    Div = 1'b0;
                    Mul = 1'b0;
                end
            endcase
            FlagW = {Funct[0], Funct[0]};
        end else begin
            ALUControl = 3'b000;
            FlagW = 2'b00;
            Shift = 1'b0;
            Div = 1'b0;
            Mul = 1'b0;
        end
    end

    assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
endmodule
module condlogic (
	clk,
	reset,
	Cond,
	ALUFlags,
	FlagW,
	PCS,
	RegW,
	MemW,
	PCSrc,
	RegWrite,
	MemWrite
);
	input wire clk;
	input wire reset;
	input wire [3:0] Cond;
	input wire [3:0] ALUFlags;
	input wire [1:0] FlagW;
	input wire PCS;
	input wire RegW;
	input wire MemW;
	output wire PCSrc;
	output wire RegWrite;
	output wire MemWrite;
	wire [1:0] FlagWrite;
	wire [3:0] Flags;
	wire CondEx;
	flopenr #(2) flagreg1(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[1]),
		.d(ALUFlags[3:2]),
		.q(Flags[3:2])
	);
	flopenr #(2) flagreg0(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[0]),
		.d(ALUFlags[1:0]),
		.q(Flags[1:0])
	);
	condcheck cc(
		.Cond(Cond),
		.Flags(Flags),
		.CondEx(CondEx)
	);
	assign FlagWrite = FlagW & {2 {CondEx}};
	assign RegWrite = RegW & CondEx;
	assign MemWrite = MemW & CondEx;
	assign PCSrc = PCS & CondEx;
endmodule
module condcheck (
	Cond,
	Flags,
	CondEx
);
	input wire [3:0] Cond;
	input wire [3:0] Flags;
	output reg CondEx;
	wire neg;
	wire zero;
	wire carry;
	wire overflow;
	wire ge;
	assign {neg, zero, carry, overflow} = Flags;
	assign ge = neg == overflow;
	always @(*)
		case (Cond)
			4'b0000: CondEx = zero;
			4'b0001: CondEx = ~zero;
			4'b0010: CondEx = carry;
			4'b0011: CondEx = ~carry;
			4'b0100: CondEx = neg;
			4'b0101: CondEx = ~neg;
			4'b0110: CondEx = overflow;
			4'b0111: CondEx = ~overflow;
			4'b1000: CondEx = carry & ~zero;
			4'b1001: CondEx = ~(carry & ~zero);
			4'b1010: CondEx = ge;
			4'b1011: CondEx = ~ge;
			4'b1100: CondEx = ~zero & ge;
			4'b1101: CondEx = ~(~zero & ge);
			4'b1110: CondEx = 1'b1;
			default: CondEx = 1'bx;
		endcase
endmodule
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


module regfile (
	clk,
	we3,
	ra1,
	ra2,
	wa3,
	wd3,
	r15,
	rd1,
	rd2
);
	input wire clk;
	input wire we3;
	input wire [3:0] ra1;
	input wire [3:0] ra2;
	input wire [3:0] wa3;
	input wire [31:0] wd3;
	input wire [31:0] r15;
	output wire [31:0] rd1;
	output wire [31:0] rd2;
	reg [31:0] rf [14:0];
	always @(posedge clk)
		if (we3)
			rf[wa3] <= wd3;
	assign rd1 = (ra1 == 4'b1111 ? r15 : rf[ra1]);
	assign rd2 = (ra2 == 4'b1111 ? r15 : rf[ra2]);
endmodule
module extend (
	Instr,
	ImmSrc,
	ExtImm
);
	input wire [23:0] Instr;
	input wire [1:0] ImmSrc;
	output reg [31:0] ExtImm;
	always @(*)
		case (ImmSrc)
			2'b00: ExtImm = {24'b000000000000000000000000, Instr[7:0]};
			2'b01: ExtImm = {20'b00000000000000000000, Instr[11:0]};
			2'b10: ExtImm = {{6 {Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
		endcase
endmodule
module adder (
	a,
	b,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] a;
	input wire [WIDTH - 1:0] b;
	output wire [WIDTH - 1:0] y;
	assign y = a + b;
endmodule
module flopenr (
	clk,
	reset,
	en,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire en;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else if (en)
			q <= d;
endmodule
module flopr (
	clk,
	reset,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else
			q <= d;
endmodule
module mux2 (
	d0,
	d1,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire s;
	output wire [WIDTH - 1:0] y;
	assign y = (s ? d1 : d0);
endmodule

module mux4 (
	d0,
	d1,
	d2,
	d3,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire [WIDTH - 1:0] d2;
	input wire [WIDTH - 1:0] d3;
	input wire [1:0] s;
	output wire [WIDTH - 1:0] y;

	assign y = (s == 2'b00) ? d0 :
             (s == 2'b01) ? d1 :
             (s == 2'b10) ? d2 :
             d3;
endmodule

module alu ( input [31:0] a,b,
             input [2:0] ALUControl,
             output reg [31:0] Result,
             output wire [3:0] ALUFlags);
  
  wire negative, zero, carry, overflow;
  wire [32:0] sum;
  
  
  assign sum = a + (ALUControl[0]? ~b: b) + ALUControl[0];
  always @(*)
    casex (ALUControl[2:0])
      3'b00?: Result = sum;
      3'b010: Result = a & b;
      3'b011: Result = a | b;
	  3'b100: Result = a ^ b;
    endcase
  assign negative = Result[31];
  assign zero = (Result == 32'b0);
  assign carry = (ALUControl[1]==1'b0) & sum[32];
  assign overflow = (ALUControl[1]==1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);

  assign ALUFlags = {negative, zero, carry, overflow};

endmodule
module shifter(input  [31:0] a, 
               input  [4:0] shamt, 
               input  [1:0] shtype, 
               output reg [31:0] y); 
 
  always @(*) begin
    case (shtype) 
      2'b00:   y = a << shamt; 
	  2'b01:   y = a >> shamt;
	  2'b10:   y = $signed(a) >>> shamt;
	  2'b11:   y = (a >> shamt ) | (a << (32- shamt));
      default: y = a; 
    endcase 
  end
endmodule 

module mla(rn, rm, y);
    input wire [31:0] rn, rm;
    output wire [31:0] y;
    assign y = rn * rm;
endmodule

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