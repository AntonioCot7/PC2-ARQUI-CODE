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