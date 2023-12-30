`timescale 1ns / 1ps  // Specify simulation timescale

module ECE331Lab3_tb;

    // Inputs
    reg clk;           // Clock input
    reg reset;         // Reset input
    reg [6:0] opcode;   // Opcode input
    reg [2:0] funct3;   // Funct3 input
    reg [6:0] funct7;   // Funct7 input

    // Outputs
    wire MemRead;       // Memory read control signal output
    wire MemWrite;      // Memory write control signal output
    wire RegWrite;      // Register write control signal output
    wire MemtoReg;      // Memory to register control signal output
    wire [1:0] ALUOp;    // ALU operation control signal output
    wire ALUSrc;        // ALU source control signal output
    wire Branch;        // Branch control signal output

    // Instantiate the Unit Under Test (UUT)
    ECE331Lab3 uut (
        .clk(clk),        // Connect clk to testbench clock
        .reset(reset),    // Connect reset to testbench reset
        .opcode(opcode),  // Connect opcode to testbench opcode
        .funct3(funct3),  // Connect funct3 to testbench funct3
        .funct7(funct7),  // Connect funct7 to testbench funct7
        .MemRead(MemRead),        // Connect MemRead to testbench MemRead
        .MemWrite(MemWrite),      // Connect MemWrite to testbench MemWrite
        .RegWrite(RegWrite),      // Connect RegWrite to testbench RegWrite
        .MemtoReg(MemtoReg),      // Connect MemtoReg to testbench MemtoReg
        .ALUOp(ALUOp),            // Connect ALUOp to testbench ALUOp
        .ALUSrc(ALUSrc),          // Connect ALUSrc to testbench ALUSrc
        .Branch(Branch)           // Connect Branch to testbench Branch
    );

    // Clock generation
    always #10 clk = ~clk; // Generate a 100MHz clock with a period of 10ns

    // Test stimulus
    initial begin
        // Initialize Inputs
        clk = 0;           // Initialize clock to 0
        reset = 1;         // Assert reset
        opcode = 7'b0000000;  // Set initial opcode
        funct3 = 3'b000;   // Set initial funct3
        funct7 = 7'b0000000;  // Set initial funct7

        // Wait for reset
        #100;
        reset = 0;         // Release reset

        // Test LW instruction (lw x5, 12(x2))
        opcode = 7'b0000011;  // Set opcode for LW
        funct3 = 3'b010;    // Set funct3 for LW
        #50;

        // Test SW instruction (sw x7, 16(x22))
        opcode = 7'b0100011;  // Set opcode for SW
        funct3 = 3'b010;    // Set funct3 for SW
        #50;

        // Test SUB instruction (sub x3, x9, x31)
        opcode = 7'b0110011;  // Set opcode for R-type
        funct3 = 3'b000;    // Set funct3 for SUB
        funct7 = 7'b0100000;  // Set funct7 for SUB
        #50;

        // Test BEQ instruction (beq x8, x11, L1)
        opcode = 7'b1100011;  // Set opcode for BEQ
        funct3 = 3'b000;    // Set funct3 for BEQ
        #50;
        
        $finish; // End simulation
    end

endmodule
