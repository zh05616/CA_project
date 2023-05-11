`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:47:57
// Design Name: 
// Module Name: Detect_Hazards
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
module Detect_Hazards(
    input ID_EX_MemRead, // signal indicating if memory access is being performed in EX stage of pipeline
    input [4:0] ID_EX_Rd, // destination register number in EX stage of pipeline
    input [4:0] IF_ID_Rs1, // source register 1 number in IF stage of pipeline
    input [4:0] IF_ID_Rs2, // source register 2 number in IF stage of pipeline
    output reg IF_ID_Write, // control signal for write access to IF/ID register
    output reg PC_Write, // control signal for write access to program counter register
    output reg MUX_Control // control signal for MUX used to select next PC
);
    always @(*)
        begin
    // If a load instruction in EX stage uses a register that is read by an instruction in IF stage, then hazards are detected
            if ((ID_EX_MemRead==1'b1) && ((ID_EX_Rd == IF_ID_Rs1) || (ID_EX_Rd == IF_ID_Rs2)))
                 begin
    // Hazard detected, disable write access to IF/ID register and program counter register and set MUX control to 0
                    PC_Write <= 1'b0;
                    IF_ID_Write <= 1'b0;
                    MUX_Control <= 1'b0;
    end
             else
                begin
    // No hazard detected, enable write access to IF/ID register and program counter register and set MUX control to 1
                PC_Write <= 1'b1;
                IF_ID_Write <= 1'b1;
                MUX_Control <= 1'b1;
    end
    end
endmodule