`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:46:08
// Design Name: 
// Module Name: IF_ID
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
module IF_ID
(
    input clk, // clock signal input
    input reset, // asynchronous reset signal input
    input write, // write signal input
    input [63:0] PC_Out, // 64-bit program counter output input
    input [31:0] Instruction, // 32-bit instruction input
    output reg [63:0] PC_IFID, // 64-bit program counter output register
    output reg [31:0] Instruction_IFID // 32-bit instruction output register
    );

always@(posedge clk) // synchronous always block triggered on positive edge of clock signal
    begin
    if (reset == 1'b1) // check if reset signal is high
    begin
        PC_IFID = 64'b0; // reset program counter output register to 0
        Instruction_IFID = 64'b0; // reset instruction output register to 0
end
    else if (write) // if write signal is high
        begin
        PC_IFID = PC_Out; // write the value of the program counter output to the program counter output register
        Instruction_IFID = Instruction; // write the value of the instruction input to the instruction output register
end
end

endmodule