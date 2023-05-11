`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:48:50
// Design Name: 
// Module Name: RISC_V_Processor
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
//`include "Parser"
//`include "Mux"
//`include "Imm_Gen"
//`include "ALU"
//`include "Control_Unit"
//`include "ALU_Control"
//`include "Adder"
//`include "Program_Counter"
//`include "Instruction_Memory"
//`include "Data_Memory"
//`include "Register_File"
//`include "ID_EX"
//`include "IF_ID"
//`include "EX_MEM"
//`include "MEM_WB"
//`include "Forwarding_Unit"
//`include "Mux2"
//`include "Detect_Hazards"


module RISC_V_Processor(
  input clk, reset);

  wire [63:0] PC_Out;
  wire [63:0] PC_4;
  wire [63:0] PC_offset;
  wire [63:0] PC_offset_EXMEM;
  wire [63:0] PC_to_Mux;
  wire [63:0] PC_In;
  wire [63:0] PC_IFID;
  wire [63:0] PC_IDEX;
  wire [31:0] Instruction;
  wire [31:0] Instruction_IFID;
  wire [6:0] Op_Code;
  wire [4:0] rs1;
  wire [4:0] rs1_IDEX;
  wire [4:0] rs2;
  wire [4:0] rs2_IDEX;
  wire [4:0] rd;
  wire [4:0] rd_IDEX;
  wire [4:0] rd_EXMEM;
  wire [4:0] rd_MEMWB;
  wire [2:0] funct3;
  wire [6:0] funct7;
  wire Branch;
  wire Branch_EXMEM;
  wire Branch_IDEX;
  wire MemRead;
  wire MemRead_EXMEM;
  wire MemRead_IDEX;
  wire MemtoReg;
  wire MemtoReg_EXMEM;
  wire MemtoReg_MEMWB;
  wire MemtoReg_IDEX;
  wire [1:0] ALUOp;
  wire [1:0] ALUOp_IDEX;
  wire MemWrite;
  wire MemWrite_EXMEM;
  wire MemWrite_IDEX;
  wire ALUSrc;
  wire ALUSrc_IDEX;
  wire [63:0] ALU_Mux;
  wire RegWrite;
  wire RegWrite_IDEX;
  wire RegWrite_EXMEM;
  wire RegWrite_MEMWB;
  wire Zero;
  wire Zero_EXMEM;
  wire [63:0] ReadData1;
  wire [63:0] read_data1_IDEX;
  wire [63:0] ReadData2;
  wire [63:0] read_data2_IDEX;
  wire [63:0] read_data2_EXMEM;
  wire [63:0] WriteData;
  wire [63:0] WriteData_EXMEM;
  wire [63:0] ALU_Result;
  wire [63:0] ForwardAresult;
  wire [63:0] ForwardBresult;
  wire [63:0] ALU_Result_EXMEM;
  wire [63:0] ALU_Result_MEMWB;
  wire [3:0] Operation;
  wire [63:0] imm_data;
  wire [63:0] imm_data_IDEX;
  wire [63:0] Read_Data;
  wire [63:0] Read_Data_MEMWB;
  wire [3:0] Funct = {Instruction[30], Instruction[14], Instruction[13], Instruction[12]};
  wire [3:0] Funct_IDEX;
  wire [1:0] ForwardA;
  wire [1:0] ForwardB;

  wire Hazard_IF_ID_Write;
  wire Hazard_PC_Write;
  wire Hazard_MUX_Control;

  Program_Counter pc(clk, reset, Hazard_PC_Write, PC_In, PC_Out);
  Adder pc_to_adder(PC_Out, 64'd4, PC_4);
  Adder pc_offs(PC_Out, imm_data<<1, PC_offset);
  Mux pc_mux(Branch & Zero, PC_4, PC_offset, PC_In);
  Instruction_Memory inst_mem(PC_Out, Instruction);
  IF_ID if_id_buffer1(clk, reset, Hazard_IF_ID_Write, PC_Out, Instruction, PC_IFID, Instruction_IFID);
  Parser inst_parser(Instruction_IFID, Op_Code, rd, funct3, rs1, rs2, funct7);
  Control_Unit cont_unit(Op_Code, Hazard_MUX_Control, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, ALUOp);
  Register_File reg_file(clk, reset, RegWrite, WriteData, rs1, rs2, rd, ReadData1, ReadData2);
  Imm_Gen imm_gen(Instruction, imm_data);
  ID_EX id_ex_buffer2(clk, reset,PC_IFID,ReadData1,ReadData2,imm_data, Funct, rd,MemtoReg, RegWrite, Branch, MemWrite, 
        MemRead,ALUSrc, ALUOp,PC_IDEX,read_data1_IDEX, read_data2_IDEX,imm_data_IDEX,Funct_IDEX,rd_IDEX,MemtoReg_IDEX,RegWrite_IDEX,Branch_IDEX,MemWrite_IDEX,MemRead_IDEX,ALUSrc_IDEX,ALUOp_IDEX);
  ALU_Control alu_control(ALUOp_IDEX, Funct_IDEX, Operation);
  Forwarding_Unit forward_unit(rs1_IDEX,rs2_IDEX, rd_EXMEM, rd_MEMWB,RegWrite_EXMEM,RegWrite_MEMWB,ForwardA,ForwardB);
  Mux2 forwardAmux(ForwardA,read_data1_IDEX,WriteData,ALU_Result_EXMEM,ForwardAresult);
  Mux2 forwardBmux(ForwardB,read_data2_IDEX,WriteData,ALU_Result_EXMEM,ForwardBresult);
  Mux forwardbalu(ALUSrc,ForwardBresult,imm_data_IDEX,ALU_Mux);
  ALU alu(ForwardAresult, ALU_Mux, Operation, ALU_Result, Zero);

  EX_MEM ex_mem_buffer3(clk, reset, Branch_IDEX, MemRead_IDEX, MemtoReg_IDEX, MemWrite_IDEX, RegWrite_IDEX,
   Zero,PC_offset,ALU_Result,ForwardBresult, rd_IDEX,PC_offset_EXMEM,ALU_Result_EXMEM,WriteData,Branch_EXMEM, MemRead_EXMEM, MemtoReg_EXMEM, MemWrite_EXMEM, RegWrite_EXMEM, Zero_EXMEM,rd_EXMEM);
  Data_Memory data_mem(ALU_Result_EXMEM, WriteData, clk, MemWrite_EXMEM, MemRead_EXMEM, Read_Data);
  MEM_WB mem_wb_buffer4(clk, reset,ALU_Result_EXMEM,Read_Data,rd_EXMEM,MemtoReg_EXMEM,RegWrite_EXMEM,ALU_Result_MEMWB,Read_Data_MEMWB,rd_MEMWB,MemtoReg_MEMWB,RegWrite_MEMWB);
  Mux data_mux(MemtoReg_MEMWB, ALU_Result_EXMEM, Read_Data_MEMWB, WriteData);

  Detect_Hazards hazards(MemRead_IDEX, rd_IDEX, rs1_IDEX, rs2_IDEX, Hazard_IF_ID_Write, Hazard_PC_Write, Hazard_MUX_Control);

endmodule