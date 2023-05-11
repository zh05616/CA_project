`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:43:28
// Design Name: 
// Module Name: Register_File
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
module Register_File(
  input clk, reset, RegW,
  input [63:0] WriteD, [4:0] RS1, [4:0] RS2, [4:0] RD, 
  output [63:0] ReadD1, [63:0] ReadD2
);
  reg [63:0] Registers [31:0];
  integer i;
  initial 
  begin

    for (i=0; i<32; i=i+1)
      Registers[i] = i;
  end
  
  always @(posedge clk)
    if (RegW) Registers[RD] = WriteD;
  
  assign ReadD1 = (reset)? 0: Registers[RS1];
  assign ReadD2 = (reset)? 0: Registers[RS2];
  
endmodule