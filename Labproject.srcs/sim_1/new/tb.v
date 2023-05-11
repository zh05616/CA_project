`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 01:17:45
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
    
    RISC_V_Processor RISC(clk, reset);
    initial 
    begin
        clk = 0;
        reset = 1;
        #10 reset = 0;
        #100;
    end
    always
        #5 clk = ~clk;
endmodule