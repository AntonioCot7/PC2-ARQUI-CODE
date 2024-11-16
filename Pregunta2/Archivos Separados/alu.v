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