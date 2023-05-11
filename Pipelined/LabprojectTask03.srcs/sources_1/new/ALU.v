`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:39:55
// Design Name: 
// Module Name: ALU
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
module ALU(
  input [63:0] a, [63:0] b, [3:0] ALUOp,
  output [63:0] Result, 
  output Zero);
  assign Result = (ALUOp == 0) ? (a & b) : (ALUOp == 1) ? (a | b) : (ALUOp == 2) ? (a + b) : (ALUOp == 6) ? (a - b) : (ALUOp == 12) ? ~(a | b) : (ALUOp == 10) ? (b + ~a) : (ALUOp == 3) ? (a - b) : 0;
  assign Zero = (Result == 0);
endmodule