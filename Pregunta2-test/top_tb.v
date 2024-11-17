module top_tb;
    reg clk;
    reg reset;
    wire [31:0] WriteData;
    wire [31:0] DataAdr;
    wire MemWrite;

    top dut (
        .clk(clk),
        .reset(reset),
        .WriteData(WriteData),
        .DataAdr(DataAdr),
        .MemWrite(MemWrite)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        reset = 1;
        #15 reset = 0;
    end

    always @(posedge clk) begin
        $display($time, " | PC = %h | Instr = %h | MemWrite = %b | DataAdr = %h | WriteData = %h",
                 dut.arm_inst.PC, dut.imem_inst.RAM[dut.arm_inst.PC[31:2]], MemWrite, DataAdr, WriteData);
    end

    initial begin
        $dumpfile("top_tb.vcd");
        $dumpvars(0, top_tb);
    end

    initial begin
        #400;
        $display("Fin de la simulación.");
        $finish;
    end
endmodule
