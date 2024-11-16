`timescale 1ns / 1ps

module testbench;
    reg clk;
    reg reset;
    wire [31:0] WriteData, DataAdr;
    wire MemWrite;

    // Instancia del módulo top (el procesador pipeline)
    top dut (
        .clk(clk),
        .reset(reset),
        .WriteData(WriteData),
        .DataAdr(DataAdr),
        .MemWrite(MemWrite)
    );

    // Generación del reloj
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Período de 10 unidades de tiempo
    end

    // Secuencia de prueba
    initial begin
        // Inicialización y reset
        reset = 1;
        #15 reset = 0;

        // Observación de las señales por un tiempo determinado para verificar el pipeline
        #500;  // Simulación de 500 unidades de tiempo (ajusta según sea necesario)

        // Condición de prueba para verificar un resultado esperado
        if (MemWrite && (DataAdr == 32'h64) && (WriteData == 32'h07)) begin
            $display("Test passed.");
        end else begin
            $display("Test failed.");
        end

        // Detener la simulación
        $stop;
    end

    // Monitoreo de señales para depuración
    initial begin
        $monitor("Time: %0t | PC: %h | DataAdr: %h | WriteData: %h | MemWrite: %b",
                 $time, dut.dp.PC, DataAdr, WriteData, MemWrite);
    end
endmodule
