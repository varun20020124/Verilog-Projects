module ECE331Lab2(
	input clk, 
	input write,
	input [4:0] rs1,
	input [4:0] rs2, 
	input [4:0] rd,
	output [31:0] data, 
	output [31:0] rv1,
	output [31:0] rv2,
	input reset,
	input RegWrite,
	input ALUSrc,
	input PCSrc,
	input MemRead,
	input MemWrite,
	input MemToReg,
	input ALUOp0,
	input ALUOp1,
	input func3,
	input func5 ,
	input [11:0] imm
	);
	reg [31:0] registers[31:0];
	reg [31:0] memory [255:0];
	reg [31:0] opcode;
	reg [31:0] counter;
	assign data = opcode;
	assign rv1 = registers[rs1];
	assign rv2 = registers[rs2];
	always @(posedge clk) begin
		if (reset == 0) 
			begin
				registers[rs1] <= 0;
				registers[rs2] <= 0;
			end
		else 
			begin
			if (write) 
				begin
					if (rd == 0) begin
						registers[rd] <= 0;
					end
					if (rs1 == 0) begin
						registers[rs1] <= 0;
					end
					if (rs2 == 0) begin
						registers[rs2] <= 0;
					end
					// beq opcode
					if (ALUSrc == 0 && (MemToReg == 0 || MemToReg == 1) && RegWrite == 0 && MemRead == 0 && MemWrite == 0 && PCSrc == 1 
					&& ALUOp1 == 0 && ALUOp0 == 1) begin
						opcode <= 7'b1100011;
						if(func3==3'b000) begin
							if (registers[rs1] == registers[rs2]) begin
								counter <= counter + imm;
							end
						end
					end
					// sw's opcode
					if (ALUSrc == 1 && (MemToReg == 0 || MemToReg == 1) && RegWrite == 0 && MemRead == 0 && MemWrite == 1 && PCSrc == 0 
					&& ALUOp1 == 0 && ALUOp0 == 1) begin
						opcode <= 7'b0100111;
						memory[registers[rs1] + imm] <= registers[rd];
					end
					// lw's opcode
					if (ALUSrc == 1 && MemToReg == 1 && RegWrite == 1 && MemRead == 1 && MemWrite == 0 && PCSrc == 0 
					&& ALUOp1 == 0 && ALUOp0 == 0) begin
						opcode <= 7'b0000011; 
						registers[rd] <= memory[registers[rs1] + imm];
					end
					// R-type's opcode
					if (ALUSrc == 0 && MemToReg == 0 && RegWrite == 1 && MemRead == 0 && MemWrite == 0 && PCSrc == 0 
					&& ALUOp1 == 1 && ALUOp0 == 0) begin
						opcode <= 7'b0110011; 
						if (func3 == 0 && func5 == 0) begin
							registers[rd]=registers[rs1]+registers[rs2];
						end
						if (func3 == 0 && func5 == 7'b0100000) begin
							registers[rd]=registers[rs1]-registers[rs2];
						end
						if (func3 == 3'b111 && func5 == 7'b0000000) begin
							registers[rd]= (registers[rs1] && registers[rs2]);
						end
						if (func3 == 3'b110 && func5 == 7'b0000000) begin
							registers[rd]= (registers[rs1] || registers[rs2]);
						end
					end
					registers[rd] <= data;
				end
			end 
		end
endmodule