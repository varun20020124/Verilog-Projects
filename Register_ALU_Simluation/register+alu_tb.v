module ECE331Lab2_tb;

reg clk;
reg write;
reg [4:0] rs1;
reg [4:0] rs2;
reg [4:0] rd;
wire[1:0]data;
wire [31:0] rv1;
wire [31:0] rv2;
reg reset;
reg RegWrite;
reg ALUSrc;
reg PCSrc;
reg MemRead;
reg MemWrite;
reg MemToReg;
reg ALUOp0;
reg ALUOp1;
reg [2:0] func3;
reg [6:0]func5;
reg [11:0] imm;
//reg boolean;

ECE331Lab2 dut (
	.clk(clk),
	.write(write),
	.rs1(rs1),
	.rs2(rs2),
	.rd(rd),
	.data(data),
	.rv1(rv1),
	.rv2(rv2),
	.reset(reset),
	.RegWrite(RegWrite),
	.ALUSrc(ALUSrc),
	.PCSrc(PCSrc),
	.MemRead(MemRead),
	.MemWrite(MemWrite),
	.MemToReg(MemToReg),
	.ALUOp0(ALUOp0),
	.ALUOp1(ALUOp1),
	.func3(func3),
	.func5(func5),
	.imm(imm)
);
initial begin 
clk=0;
forever #5 clk=~clk;
end
initial begin
reset=0;
#20;
reset=1;
#20
// Below is just a basic implementation of an add instruction i.e add x7, x5, x6 in RISC-V
/*rs1=5;
rs2=6;
rd=7;
RegWrite=1;
ALUSrc=0;
PCSrc=0;
MemRead=0;
MemWrite=0;
MemToReg=0;
ALUOp0=0;
ALUOp1=1;
func3=3'b000;
func5=7'b0000000;*/
end
endmodule