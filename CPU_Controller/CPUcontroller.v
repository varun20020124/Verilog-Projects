module CustomProcessorControl(
    input clk,               // Clock input
    input reset,             // Reset input
    input [6:0] opcode,      // Opcode input
    input [2:0] funct3,      // Funct3 input
    input [6:0] funct7,      // Funct7 input
    output reg MemRead,      // Memory read control signal output
    output reg MemWrite,     // Memory write control signal output
    output reg RegWrite,     // Register write control signal output
    output reg MemtoReg,     // Memory to register control signal output
    output reg [1:0] ALUOp,   // ALU operation control signal output
    output reg ALUSrc,       // ALU source control signal output
    output reg Branch        // Branch control signal output
);

    // Define custom state constants
    parameter ST_FETCH      = 3'b000,
              ST_DECODE     = 3'b001,
              ST_EXECUTE    = 3'b010,
              ST_MEMORY     = 3'b011,
              ST_WRITE_BACK = 3'b100;

    // Custom opcode values based on RISC-V ISA
    parameter OP_RTYPE = 7'b0110011,
              OP_LW    = 7'b0000011,
              OP_SW    = 7'b0100011,
              OP_BEQ   = 7'b1100011;

    // Custom funct3 codes for R-type operations
    parameter FNCT3_ADD = 3'b000,
              FNCT3_SUB = 3'b000,
              FNCT3_AND = 3'b111,
              FNCT3_OR  = 3'b110;

    // Custom funct7 codes for ADD and SUB distinction
    parameter FNCT7_ADD = 7'b0000000,
              FNCT7_SUB = 7'b0100000;

    // Current state and next state variables
    reg [2:0] current_state, next_state;

    // FSM state transition and output logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= ST_FETCH;   // Initialize state to FETCH on reset
        end else begin
            current_state <= next_state; // Update state based on next_state
        end
    end

    // Control signal setting logic
    always @(*) begin
        // Default signals
        {MemRead, MemWrite, RegWrite, MemtoReg, ALUOp, ALUSrc, Branch} = 7'b0000000;

        case (current_state)
            ST_FETCH: begin
                next_state = ST_DECODE; // Move to DECODE after FETCH
            end
            ST_DECODE: begin
                // Decide the next state based on the opcode
                if (opcode == OP_RTYPE) begin
                    next_state = ST_EXECUTE; // R-type instructions go to EXECUTE
                end else if (opcode == OP_LW || opcode == OP_SW) begin
                    next_state = ST_MEMORY; // Load and store go to MEMORY
                end else if (opcode == OP_BEQ) begin
                    next_state = ST_EXECUTE; // Branches go to EXECUTE for ALU comparison
                end else begin
                    next_state = ST_FETCH; // Default case goes back to FETCH
                end
            end
            ST_EXECUTE: begin
                // R-type and BEQ instructions come here
                if (opcode == OP_RTYPE) begin
                    RegWrite = 1; // R-type instructions write to register
                    ALUSrc = 0; // R-type uses register values
                    MemtoReg = 0; // R-type does not involve memory to register transfer
                    // Determine the ALU operation
                    if (funct3 == FNCT3_ADD) begin
                        ALUOp = (funct7 == FNCT7_ADD) ? 2'b00 : 2'b01; // ADD or SUB based on funct7
                    end else if (funct3 == FNCT3_AND) begin
                        ALUOp = 2'b10; // AND operation
                    end else if (funct3 == FNCT3_OR) begin
                        ALUOp = 2'b11; // OR operation
                    end
                end else if (opcode == OP_BEQ) begin
                    // Set up for branch comparison (e.g., subtract and check zero)
                    ALUOp = 2'b01;
                    Branch = 1; // Enable branch control signal
                end
                next_state = ST_WRITE_BACK; // Go to WRITE_BACK or FETCH based on instruction
            end
            ST_MEMORY: begin
                // Load and store instructions come here
                if (opcode == OP_LW) begin
                    // Load word control signals
                    MemRead = 1;
                    MemtoReg = 1; // Data coming from memory to register
                    RegWrite = 1; // Write the loaded data into the register
                    ALUSrc = 1; // Use immediate/address offset
                    ALUOp = 2'b00; // ALU should pass through the address for memory access
                end else if (opcode == OP_SW) begin
                    // Store word control signals
                    MemWrite = 1;
                    ALUSrc = 1; // Use immediate/address offset
                    ALUOp = 2'b00; // ALU should pass through the address for memory access
                end
                next_state = ST_FETCH; // After memory operation, fetch the next instruction
            end
            ST_WRITE_BACK: begin
                // Only load instructions need to write back to the register
                if (opcode == OP_LW) begin
                    // Write-back phase for load instruction
                    RegWrite = 1;
                end
                next_state = ST_FETCH; // After write-back, fetch the next instruction
            end
            default: begin
                // If the state is unknown, reset to the FETCH state
                next_state = ST_FETCH;
            end
        endcase
    end
endmodule
