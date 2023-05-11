`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:45:30
// Design Name: 
// Module Name: ID_EX
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
module ID_EX
(
    input clk, // clock signal
    input reset, // reset signal
    input [63:0] PC_IFID, // program counter value from IF/ID pipeline register
    input [63:0] read_data1, read_data2, // data values read from registers
    input [63:0] imm_val, // immediate value
    input [3:0] funct_in, // function code
    input [4:0] rd_in, // destination register
    input MemtoReg, RegWrite, // control signals
    input Branch, MemWrite, MemRead,
    input ALUSrc,
    input [1:0] ALU_op,

  	output reg [63:0] PC_IDEX, // output program counter value to be used in the EX stage
  	output reg [63:0] read_data1_IDEX, read_data2_IDEX, // output register values to be used in the EX stage
  	output reg [63:0] imm_val_IDEX, // output immediate value to be used in the EX stage
  	output reg [3:0] funct_in_IDEX, // output function code to be used in the EX stage
  	output reg [4:0] rd_in_IDEX, // output destination register to be used in the EX stage
    output reg MemtoReg_IDEX, RegWrite_IDEX, // output control signals to be used in the EX stage
    output reg Branch_IDEX, MemWrite_IDEX, MemRead_IDEX,
    output reg ALUSrc_IDEX, // output ALU source control signal to be used in the EX stage
    output reg [1:0] ALU_op_IDEX // output ALU operation control signal to be used in the EX stage
);

  always@(posedge clk) // always block that triggers on the positive edge of the clock
	begin
      if(reset == 1'b0) // if reset signal is not asserted
        begin // initialize outputs with input values
          PC_IDEX = PC_IFID;
          read_data1_IDEX = read_data1;
          read_data2_IDEX = read_data2;
          imm_val_IDEX = imm_val;
          funct_in_IDEX = funct_in;
          rd_in_IDEX = rd_in;
          MemtoReg_IDEX =  MemtoReg;
          RegWrite_IDEX = RegWrite;
          Branch_IDEX = Branch;
          MemWrite_IDEX = MemWrite;
          MemRead_IDEX = MemRead;
          ALUSrc_IDEX = ALUSrc;
          ALU_op_IDEX = ALU_op;
        end
      else // if reset signal is asserted
        begin // initialize outputs with zeros
          PC_IDEX = 64'b0;
          read_data1_IDEX = 64'b0;
          read_data2_IDEX = 64'b0;
          imm_val_IDEX = 64'b0;
          funct_in_IDEX = 4'b0;
          rd_in_IDEX = 5'b0;
          MemtoReg_IDEX =  1'b0;
          RegWrite_IDEX = 1'b0;
          Branch_IDEX = 1'b0;
          MemWrite_IDEX = 1'b0;
          MemRead_IDEX = 1'b0;
          ALUSrc_IDEX = 1'b0;
          ALU_op_IDEX = 2'b0;
        end
	end
endmodule