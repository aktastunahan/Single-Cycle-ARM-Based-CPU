/*	
	THIS FILE IS A MODULE LIBRARY FOR ARM BASED COMPUTER DESIGN.
	THE LIBRARY CONSIST OF SEVERAL MODULES TO BE USED FOR THE
	COMPUTER DESIGN.
*/

//----------------------------------------------------------------------------------------------//

/* 1.2.1*/
// Constant value generator. The module outputs the 'VAL' value as 'W' bit wire.
module constant_value_generator #( parameter W=32, VAL=32'd0) (
											  output [W-1:0] o_value // constant VAL value as W bit output
										);
	assign o_value = VAL;
endmodule

//----------------------------------------------------------------------------------------------//

/* 1.2.2*/
// 2x4 decoder module. Takes 2 bit i_data as input and decodes it as 4 bit o_data output.
module decoder2x4( input  [1:0] i_data, // input data to be decoded
						 output [3:0] o_data	 // decoded value
						 );
	// combinational circuits for the decoder
	assign o_data[0] = ~i_data[0] & ~i_data[1];
	assign o_data[1] =  i_data[0] & ~i_data[1];
	assign o_data[2] = ~i_data[0] &  i_data[1];
	assign o_data[3] =  i_data[0] &  i_data[1];

endmodule

//----------------------------------------------------------------------------------------------//

/* 1.2.3*/
// 2x1 multiplexer module. multiplexes 'W' bit 'i_data0' or 'i_data1' depending on 'sel' input.
module mux2x1 #( parameter W=32 )( 
					  input  [W-1:0] i_data0,	// selected when sel = 0
					  input  [W-1:0] i_data1, 	// selected when sel = 1
					  input  sel,					// select signal
					  output [W-1:0] o_data		// selected data
					);
	// combinational circuit for 2x1 multiplexers for each index, i, from 0 to W-1		

		genvar i;
		generate
		for(i=0;i<W;i=i+1) begin : mux2x1_circuit
			assign o_data[i] = (~sel & i_data0[i]) | (sel & i_data1[i]);
		end
		endgenerate
endmodule

// 4x1 multiplexer module. Multiplexes between 'W' bit 'i_data0', 
// 'i_data1', 'i_data2' or 'i_data3' depending on 'sel' input.
module mux4x1 #( parameter W=32 )( 
					  input  [W-1:0] i_data0, // selected when sel = 0
					  input  [W-1:0] i_data1, // selected when sel = 1
					  input  [W-1:0] i_data2, // selected when sel = 2
					  input  [W-1:0] i_data3, // selected when sel = 3
					  input  [1:0] sel,		// select signal
					  output [W-1:0] o_data   // selected data 
					);
									
	// decode the select (sel) signal
	wire [3:0] sel_sig;
	decoder2x4 select_signal(.i_data(sel), .o_data(sel_sig));
	
	// combinational circuit for 4x1 multiplexers for each index, i, from 0 to W-1
		genvar i;
		generate
		for(i=0;i<W;i=i+1) begin : mux4x1_circuit
			assign o_data[i] = (sel_sig[0] & i_data0[i]) | (sel_sig[1] & i_data1[i])
									 | (sel_sig[2] & i_data2[i]) | (sel_sig[3] & i_data3[i]);
		end
		endgenerate
	
endmodule

//----------------------------------------------------------------------------------------------//

/* 1.2.4*/
// W-bit ALU for logic and 2’s complement arithmetic.
module alu #( parameter W=32 )(
				  input  [2:0] i_ALUCtrl,
				  input  [W-1:0] i_SrcA,
				  input  [W-1:0] i_SrcB,
				  output reg [W-1:0] result,
				  output reg CO, OVF, N, Z
				  );
	wire same_sign_ab = i_SrcA[W-1] ~^ i_SrcB[W-1]; // checks whether the inputs have the same sign
	wire same_sign_ar = i_SrcA[W-1] ~^ result[W-1]; // checks whether the input A and the result have the same sign
	// arithmetic and logic operations depending on i_ALUCtrl input
	always@(i_ALUCtrl, i_SrcA, i_SrcB) begin
	case(i_ALUCtrl)
		3'd0: begin
					{CO, result} <= i_SrcA + i_SrcB;
					OVF <= same_sign_ab & ~same_sign_ar; // A and B has the same sign, result has different
		      end
		3'd1: begin
					{CO, result} <= i_SrcA - i_SrcB;
					OVF <= ~same_sign_ab & ~same_sign_ar; // A and B have different signs, result different than A (same as B).
				end
		3'd2: result <= i_SrcB;
		3'd3: result <= i_SrcA & (~i_SrcB);
		3'd4: result <= i_SrcA & i_SrcB;
		3'd5: result <= i_SrcA | i_SrcB;
		3'd6: result <= i_SrcA ^ i_SrcB;
		3'd7: result <= ~(i_SrcA ^ i_SrcB);
	endcase
		N <= result[W-1]; 	// update N flag any way.
		Z <= (result == 0);  // update Z flag any way.
	end

endmodule

//----------------------------------------------------------------------------------------------//

/* 1.2.5-a*/
// Simple register with synchronous reset
module simple_register #( parameter W=32 )(
							input clk,
							input i_rst,
							input [W-1:0] i_data,
							output reg [W-1:0] o_data
							);
							
	// generate W-bit 0 for reset case
	wire [W-1:0] zero;
	constant_value_generator #(.W(W), .VAL(0)) zero_gen_1 (.o_value(zero) );
	
	// select between 0 and i_data depending on the reset input
	// it is the combinational logic on D input of D-FF
	wire [W-1:0] data;
	mux2x1 #(.W(W)) reg_sel(.i_data0(i_data), .i_data1(zero), .sel(i_rst), .o_data(data));

	// sequential logic
	always@(posedge clk)
		o_data <= data;
endmodule

//----------------------------------------------------------------------------------------------//

/* 1.2.5-b*/
// Register with synchronous reset and write enable
module register #( parameter W=32 )(
							input clk,
							input i_rst,
							input i_wrt_en,
							input [W-1:0] i_data,
							output reg [W-1:0] o_data
							);
							
	// generate W-bit 0 for reset case
	wire [W-1:0] zero;
	constant_value_generator #(.W(W), .VAL(0)) zero_gen_2 (.o_value(zero) );
	
	// {rst, wrt_en}: {00} -> o_data <= o_data, {01} -> o_data <= i_data, {10} -> o_data <= 0, {11} -> o_data <= 0
	// it is the combinational logic on D input of D-FF
	wire [W-1:0] data;
	mux4x1 #(.W(W)) reg_sel(.i_data0(o_data), .i_data1(i_data), .i_data2(zero), .i_data3(zero), .sel({i_rst, i_wrt_en}), .o_data(data));

	// sequential logic
	always@(posedge clk)
		o_data <= data;
		
endmodule

//----------------------------------------------------------------------------------------------//

/* 1.2.5-c*/
// Shift register with parallel and serial load
module shift_register #( parameter W=32 )(
							input clk,
							input i_rst,
							input i_parallel,
							input i_right,
							input [W-1:0] i_parallel_data,
							input i_serial_rdata, i_serial_ldata,
							output reg [W-1:0] o_data
							);
							
	// multiplexer inputs for serial shift case (or D inputs of D-FF): serial shift left and right
	wire [W-1:0] serial_shl;
	wire [W-1:0] serial_shr;
	assign serial_shl = {o_data[W-2: 0] ,i_serial_rdata};  // shift left A and load most sig. bit of the serial input right
	assign serial_shr = {i_serial_ldata, o_data[W-1: 1]};    // shift right A and load most sig. bit of the serial input left
	
	// multiplexer input for reset case. generate W-bit 0
	wire [W-1:0] zero;
	constant_value_generator #(.W(W), .VAL(0)) zero_gen_3 (.o_value(zero) );
	
	
	
	// generate 2-bit select for 4x1 mux ( code the three input into 2 bit )
	wire [1:0] sel;
	assign sel[0] = i_rst | ((~i_parallel) & i_right);
	assign sel[1] = i_rst | i_parallel;
	// if i_rst = 0            : we will have sel=2'b11, independent of other inputs, the mux will select 0
	// else if i_parallel_data is 1 : we will have sel=2'b10, the mux will select parallel input
	// else:     we will have sel={0,i_right}, that is, 00 or 01 the mux will select serial input right or left
	


	// sel: {00} -> o_data <= Shift Left A, {01} -> o_data <= Shift Right A, {10} -> o_data <= A ← Parallel Input, {11} -> o_data <= 0
	// it is the combinational logic on D input of D-FF
	wire [W-1:0] data;
	mux4x1 #(.W(W)) reg_sel(.i_data0(serial_shl), .i_data1(serial_shr), .i_data2(i_parallel_data), .i_data3(zero), .sel(sel), .o_data(data));

	// sequential logic
	always@(posedge clk)
		o_data <= data;
endmodule
		
//***************************************************************************************//
//******************************* EXTRA MODULES FOR LAB 3 *******************************//
//***************************************************************************************//

// PC' = (PC + 4) + 4 adder 
module adder #(parameter W=32)(
					input [W-1:0] srcA,
					input [W-1:0] srcB,
					output [W-1:0] result
					);
assign result = srcA + srcB;					
endmodule

//----------------------------------------------------------------------------------------------//

// Instruction memory
module instruction_memory #(parameter W=32)(
									input [W-1:0] i_A,
									output [W-1:0] o_RD
);
// RAM memory 4096x32-bit
reg [W-1:0] memory [0:255];

	integer i;
	initial begin		// test the architecture													
		memory[0]  = 32'b1110_01_011001_0001_0000_000000000000;  	// LDR R0,[R1,#0] 	(R0 <- M[0]=15)
		memory[1]  = 32'b1110_01_011001_0010_0001_000000000001;  	// LDR R1,[R2,#1] 	(R1 <- M[1]=5)	
		memory[2]  = 32'b1110_01_011001_0001_0010_000000000010;  	// LDR R2,[R1,#2] 	(R2 <- M[7]=7)		
		memory[3]  = 32'b1110_00_010100_0110_0000_00000000_0111;		// CMP R6, R7 			(0==0: EQ FLAG)
		memory[4]  = 32'b0001_00_000100_0000_0000_00000000_0001;		// SUBNE R0,R0,R1 	(NOT EXECUTE)
		memory[5]  = 32'b0000_00_000101_0000_0000_00000000_0001;		// SUBSEQ R0, R0, R1 (R0 <- 15-5=10, UPD. FLAGS)
		memory[6]  = 32'b1011_00_001000_0000_0000_00000000_0001;		// ADDLT R0, R0, R1 	(NOT EXECUTE)
		memory[7]  = 32'b1100_00_001000_0000_0000_00000000_0010;		// ADDGT R0, R0, R2 	(R0 <- 10+7=17)
		memory[8]  = 32'b1110_01_011000_0010_0000_000000000100;   	// STR R0,[R2, #4] 	(M[11] <- 17
		memory[9]  = 32'b1110_00_001000_0010_0000_00000000_0001;		// ADD R0, R2, R1   	(R0 <- 7+5=12)
		memory[10] = 32'b1110_01_011001_0010_0011_000000000100;  	// LDR R3,[R2,#4] 	(R3 <- M[11])	
		memory[11] = 32'b1110_00_011010_0000_0011_00011_00_0_0001;	// LSL R3, R1, #3 	(R3 <- 40)
		memory[12] = 32'b1110_00_011010_0000_0010_00001_01_0_0011;	// LSR R2, R3, #1 	(R2 <- 20)
		memory[13] = 32'b1110_01_011001_0001_0010_000000000101;  	// LDR R2,[R1, #5] 	(R2 <- M[10]=0xAA)	
		memory[14] = 32'b1110_00_000001_0010_0011_00000000_0001;		// ANDS R3, R2, R1  
		memory[15] = 32'b1110_00_011000_0010_0010_00000000_0000;		// ORR R2, R2, R0
		memory[16] = 32'b1110_00_000101_0011_0010_00000000_0010;		// SUBS R2, R3, R2
		memory[17] = 32'b0000_00_000000_0010_0011_00000000_0001;		// ANDEQ R3, R2, R1
		memory[18] = 32'b1011_00_000000_0010_0011_00000000_0001;		// ANDLT R3, R2, R1
		
		for(i=19; i<255; i= i+1) begin
			memory[i] = 32'b0;
		end
	end
	
// read the instruction, pointed by 'i_A'
assign o_RD = memory[i_A[7:0]];

endmodule

//----------------------------------------------------------------------------------------------//

module extend( input [11:0] i_imm, 	  // immediate to be extended
					output [31:0] o_ExtImm // extended output
);

assign o_ExtImm = {20'b0, i_imm};		 // memory extention
endmodule

//----------------------------------------------------------------------------------------------//

// Data memory
module data_memory #(parameter W=32)(
									input clk,
									input i_WE,
									input [W-1:0] i_WD,
									input [W-1:0] i_A,
									output [W-1:0] o_RD
);
// RAM memory 4096x32-bit
reg [W-1:0] memory [0:255];

// initially, read the instructions to our memory

	integer i;
	initial begin		// initializing memory for debug purposes		
		memory[0] = 32'd15;  	
		memory[1] = 32'd5;  																						
		memory[2] = 32'd0;	  
		memory[2] = 32'd0;	
		memory[3] = 32'd0;	
		memory[4] = 32'd0;
		memory[5] = 32'd0;
		memory[6] = 32'd0;
		memory[7] = 32'd7;
		memory[8] = 32'd0;
		memory[9] = 32'd0;
		memory[10] = 32'h000000AA;
		for(i=11; i<255; i= i+1) begin
			memory[i] = 32'b0;
		end
	end

// read the instruction, pointed by 'i_A'
assign o_RD = memory[i_A[7:0]];

always@(posedge clk)
	if(i_WE)
		memory[i_A[7:0]] <= i_WD;
endmodule


//----------------------------------------------------------------------------------------------//

// Shifter
module shifter #(parameter W=32)(
						input [6:0] i_opcode,
						input [W-1:0] i_data,
						output [W-1:0] o_data
);

wire [31:0] shfl [0:31];
wire [31:0] shfr [0:31];
wire [31:0] ashr [0:31];
wire [31:0] ror [0:31];


assign shfl[0] = i_data;
assign shfr[0] = i_data;
assign ashr[0] = i_data;
assign ror[0] = i_data;
genvar i;
generate
for(i=1;i<32;i=i+1) 
begin : shift

	// shift left
	assign shfl[i][31:1] = shfl[i-1][30:0];
	assign shfl[i][0] = 0;
	
	// shift right
	assign shfr[i][30:0] = shfr[i-1][31:1];
	assign shfr[i][31] = 0;
	
	// arithmetic shift right
	assign ashr[i][30:0] = ashr[i-1][31:1];
	assign ashr[i][31] = ashr[i-1][31];
	
	// rotate right
	assign ror[i][30:0] = ror[i-1][31:1];
	assign ror[i][31] = ror[i-1][0];
end
endgenerate

wire [3:0] op_code;
decoder2x4 select_signal(.i_data(i_opcode), .o_data(op_code));
	
assign o_data = (op_code[0]) ? shfl[i_opcode[6:2]] :
					 (op_code[1]) ? shfr[i_opcode[6:2]] :
					 (op_code[2]) ? ashr[i_opcode[6:2]] :
					 (op_code[3]) ?  ror[i_opcode[6:2]] :
					 32'b0;

endmodule
