`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:44:08
// Design Name: 
// Module Name: EX_MEM
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module EX_MEM
(
input clk, // clock signal input
input reset, // asynchronous reset signal input
input Branch_IDEX, // branch signal input from the previous pipeline stage
input MemRead_IDEX, // memory read signal input from the previous pipeline stage
input MemtoReg_IDEX, // memory-to-register signal input from the previous pipeline stage
input MemWrite_IDEX, // memory write signal input from the previous pipeline stage
input RegWrite_IDEX, // register write signal input from the previous pipeline stage
input Zero, // zero flag signal input from the previous pipeline stage
input [63:0] PCSum, // sum of the program counter and sign-extended immediate input from the previous pipeline stage
input [63:0] ALUResult, // result of the arithmetic and logical operation input from the previous pipeline stage
input [63:0] ReadData2, // second operand for the arithmetic and logical operation input from the previous pipeline stage
input [4:0] Rd_IDEX, // destination register address input from the previous pipeline stage
output reg [63:0] PCSum_EXMEM, // sum of the program counter and sign-extended immediate output register
output reg [63:0] ALUResult_EXMEM, // result of the arithmetic and logical operation output register
output reg [63:0] ReadData2_EXMEM, // second operand for the arithmetic and logical operation output register
output reg Branch_EXMEM, // branch signal output register
output reg MemRead_EXMEM, // memory read signal output register
output reg MemtoReg_EXMEM, // memory-to-register signal output register
output reg MemWrite_EXMEM, // memory write signal output register
output reg RegWrite_EXMEM, // register write signal output register
output reg Zero_EXMEM, // zero flag signal output register
output reg [4:0] Rd_EXMEM // destination register address output register
);

    always@(posedge clk) // synchronous always block triggered on the positive edge of clock signal
    begin
    if(reset == 1'b0) // if reset signal is low
        begin
// assign the values of input ports to corresponding output registers
            PCSum_EXMEM = PCSum;
            ALUResult_EXMEM = ALUResult;
            ReadData2_EXMEM = ReadData2;
            MemtoReg_EXMEM = MemtoReg_IDEX;
            RegWrite_EXMEM = RegWrite_IDEX;
            Branch_EXMEM = Branch_IDEX;
            MemWrite_EXMEM = MemWrite_IDEX;
            MemRead_EXMEM = MemRead_IDEX;
            Rd_EXMEM = Rd_IDEX;
            Zero_EXMEM = Zero;
end
	  else
        begin
          PCSum_EXMEM = 0;
          ALUResult_EXMEM = 0;
          ReadData2_EXMEM = 0;
          MemtoReg_EXMEM =  0;
          RegWrite_EXMEM = 0;
          Branch_EXMEM = 0;
          MemWrite_EXMEM = 0;
          MemRead_EXMEM = 0;
          Rd_EXMEM = 0;
          Zero_EXMEM = 0;
        end
	end
	
endmodule