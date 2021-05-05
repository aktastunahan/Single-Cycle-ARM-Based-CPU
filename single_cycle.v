// Copyright (C) 2020  Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions 
// and other software and tools, and any partner logic 
// functions, and any output files from any of the foregoing 
// (including device programming or simulation files), and any 
// associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License 
// Subscription Agreement, the Intel Quartus Prime License Agreement,
// the Intel FPGA IP License Agreement, or other applicable license
// agreement, including, without limitation, that your use is for
// the sole purpose of programming logic devices manufactured by
// Intel and sold by Intel or its authorized distributors.  Please
// refer to the applicable agreement for further details, at
// https://fpgasoftware.intel.com/eula.

// PROGRAM		"Quartus Prime"
// VERSION		"Version 20.1.1 Build 720 11/11/2020 SJ Lite Edition"
// CREATED		"Tue Apr 27 18:20:59 2021"

module single_cycle(
	clk,
	rst,
	R0,
	R1,
	R2,
	R3,
	// deneme
	flags
);


input wire	clk;
input wire	rst;
output wire	[31:0] R0, R1, R2, R3;
output wire [3:0] flags;

wire	CO;
wire	N;
wire	OVF;
wire	Z;
wire	[3:0] SYNTHESIZED_WIRE_0;
wire	[5:0] SYNTHESIZED_WIRE_1;
wire	[1:0] SYNTHESIZED_WIRE_2;
wire	SYNTHESIZED_WIRE_3;
wire	SYNTHESIZED_WIRE_4;
wire	SYNTHESIZED_WIRE_5;
wire	SYNTHESIZED_WIRE_6;
wire	SYNTHESIZED_WIRE_7;
wire	[2:0] SYNTHESIZED_WIRE_8;

wire	[3:0] GDFX_TEMP_SIGNAL_0;


assign	GDFX_TEMP_SIGNAL_0 = {CO,OVF,N,Z};


control_unit	b2v_ctrl_unit(
	.clk(clk),
	.i_ALUFlags(GDFX_TEMP_SIGNAL_0),
	.i_cond(SYNTHESIZED_WIRE_0),
	.i_funct(SYNTHESIZED_WIRE_1),
	.i_op(SYNTHESIZED_WIRE_2),
	.o_RegWrite(SYNTHESIZED_WIRE_4),
	.o_MemWrite(SYNTHESIZED_WIRE_6),
	.o_MemtoReg(SYNTHESIZED_WIRE_7),
	.o_ALUSrc(SYNTHESIZED_WIRE_5),
	.o_RegSrc(SYNTHESIZED_WIRE_3),
	.o_ALUControl(SYNTHESIZED_WIRE_8),
	.flags(flags));

datapath datapath_inst
(
	.clk(clk),
	.rst(rst),
	.RegSrc(SYNTHESIZED_WIRE_3),
	.RegWrite(SYNTHESIZED_WIRE_4),
	.ALUSrc(SYNTHESIZED_WIRE_5),
	.MemWrite(SYNTHESIZED_WIRE_6),
	.MemtoReg(SYNTHESIZED_WIRE_7),
	.ALUControl(SYNTHESIZED_WIRE_8),
	.co(CO),
	.ovf(OVF),
	.n(N),
	.z(Z),
	.cond(SYNTHESIZED_WIRE_0),
	.funct(SYNTHESIZED_WIRE_1),
	.op(SYNTHESIZED_WIRE_2),
	.test_r0(R0) ,	// output [31:0] test_r0_sig
	.test_r1(R1) ,	// output [31:0] test_r1_sig
	.test_r2(R2) ,	// output [31:0] test_r2_sig
	.test_r3(R3) 
);


endmodule
