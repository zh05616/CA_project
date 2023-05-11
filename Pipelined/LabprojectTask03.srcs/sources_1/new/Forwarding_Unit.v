`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:44:42
// Design Name: 
// Module Name: Forwarding_Unit
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
module Forwarding_Unit
  (
    // Inputs to the module
    input [4:0] Rs1_IDEX,     // Value of the first source register from the IDEX pipeline stage
    input [4:0] Rs2_IDEX,     // Value of the second source register from the IDEX pipeline stage
    input [4:0] Rd_EXMEM,     // Value of the destination register from the EXMEM pipeline stage
    input [4:0] Rd_MEMWB,     // Value of the destination register from the MEMWB pipeline stage
    input RegWrite_EXMEM,     // Flag indicating whether the EXMEM pipeline stage should write to a register
    input RegWrite_MEMWB,     // Flag indicating whether the MEMWB pipeline stage should write to a register
    // Outputs from the module
    output reg [1:0] ForwardA,  // Signals indicating how to forward data from the EXMEM and MEMWB pipeline stages to the ID stage
//for the first source register
    output reg [1:0] ForwardB   // Signals indicating how to forward data from the EXMEM and MEMWB pipeline stages to the ID stage
//for the second source register
  );
  
  // Use an always block to update the outputs based on the inputs
  always @ (*)
  begin
    // Determine whether data should be forwarded from the EXMEM pipeline stage to the ID stage for the first source register
    if (RegWrite_EXMEM && Rd_EXMEM != 5'b0 && Rd_EXMEM == Rs1_IDEX)
        ForwardA = 2'b10;
    // Determine whether data should be forwarded from the MEMWB pipeline stage to the ID stage for the first source register
    else if (RegWrite_MEMWB && Rd_MEMWB != 5'b0 &&  Rd_MEMWB == Rs1_IDEX)
       ForwardA = 2'b01;
    // If no forwarding is required, set the signal to zero
    else 
      ForwardA = 2'b00; 
    
    // Determine whether data should be forwarded from the EXMEM pipeline stage to the ID stage for the second source register
   if (RegWrite_EXMEM && Rd_EXMEM != 5'b0 && Rd_EXMEM == Rs1_IDEX)
        ForwardB = 2'b10;
    // Determine whether data should be forwarded from the MEMWB pipeline stage to the ID stage for the second source register
    else if (RegWrite_MEMWB && Rd_MEMWB != 5'b0 &&  Rd_MEMWB == Rs1_IDEX)
       ForwardB = 2'b01;
    // If no forwarding is required, set the signal to zero
    else 
      ForwardB = 2'b00; 
  end
endmodule