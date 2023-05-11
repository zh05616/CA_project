`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:50:48
// Design Name: 
// Module Name: tb
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
module tb;
  reg clk;
  reg reset;

  RISC_V_Processor finally(clk, reset);

  initial begin

    clk = 0;
    reset = 1;
    #10 reset = 0;
    #1000; 
  end
  
  always
    #5 clk = ~clk;
endmodule