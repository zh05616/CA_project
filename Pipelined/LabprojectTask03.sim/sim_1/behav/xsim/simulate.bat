@echo off
REM ****************************************************************************
REM Vivado (TM) v2020.1 (64-bit)
REM
REM Filename    : simulate.bat
REM Simulator   : Xilinx Vivado Simulator
REM Description : Script for simulating the design by launching the simulator
REM
REM Generated by Vivado on Thu May 04 12:49:45 +0500 2023
REM SW Build 2902540 on Wed May 27 19:54:49 MDT 2020
REM
REM Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
REM
REM usage: simulate.bat
REM
REM ****************************************************************************
echo "xsim tb_behav -key {Behavioral:sim_1:Functional:tb} -tclbatch tb.tcl -view C:/Users/aa05432/Downloads/CAProject (2)/CAProject/LabprojectTask03/tb_behav.wcfg -log simulate.log"
call xsim  tb_behav -key {Behavioral:sim_1:Functional:tb} -tclbatch tb.tcl -view C:/Users/aa05432/Downloads/CAProject (2)/CAProject/LabprojectTask03/tb_behav.wcfg -log simulate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0