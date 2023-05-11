`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:39:23
// Design Name: 
// Module Name: Imm_Gen
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
module Imm_Gen(
  input [31:0] ins,
  output [63:0] imm_data
);
  wire S = ins [6:5];
  assign imm_data = (S==2'b0) ? {{52{ins[31]}},ins[31:20]} : 
    (S==2'b1) ? {{52{ins[31]}},ins[31:25], ins[11:7]} : 
  	{{53{ins[31]}}, ins[7], ins[30:25], ins[11:8]};
endmodule