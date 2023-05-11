`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:41:52
// Design Name: 
// Module Name: Program_Counter
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
module Program_Counter(
    input clk, reset, write,
    input [63:0] PC_In, // input port to get new program counter value
    output reg [63:0] PC_Out // output port for current program counter value
);
  // Initialize program counter output
  initial PC_Out = 0;
  always @(posedge clk or posedge reset) begin
    if (reset) // reset program counter
      PC_Out = 0;
    else if (write) // write new program counter value to output
      PC_Out <= PC_In;
  end
endmodule
