module main_decoder(
							input [1:0] i_op,
							input [5:0] i_funct,
							output o_RegW,
							output o_MemW,
							output o_ALUOp,
							output o_MemtoReg,
							output o_ALUSrc,
							output o_RegSrc
);
/*
MEMORY TYPE:
funct[5]=I';i_funct[4]=P;i_funct[3]=U; 
funct[2]=B ;i_funct[1]=W;i_funct[0]=L;

DP:
funct[5]=I;i_funct[4:1]=cmd;
funct[0]=S;
*/
// decode the instruction
wire dp_instr, mem_instr;
assign dp_instr = ~i_op[1] & ~i_op[0];
assign mem_instr = ~i_op[1] & i_op[0];

wire ldr_instr, str_instr;
wire cmp_instr, lsl_instr;

// LDR imm: 011001
assign ldr_instr = mem_instr & (~i_funct[5] & i_funct[4] 
			& i_funct[3] & ~i_funct[2] & ~i_funct[1] &  i_funct[0]);

// STR imm: 011000		
assign str_instr = mem_instr & (~i_funct[5] & i_funct[4] 
			& i_funct[3] & ~i_funct[2] & ~i_funct[1] & ~i_funct[0]);


		// assign outputs
assign o_RegW = ldr_instr | dp_instr;
assign o_MemW = str_instr;
assign o_ALUOp = dp_instr;
assign o_MemtoReg = ~dp_instr;
assign o_ALUSrc = ~dp_instr;
assign o_RegSrc = str_instr;

endmodule

//----------------------------------------------------------------------------------------------//

module alu_decoder( input [4:0] i_funct,
						  input i_ALUOp,
						  output o_NoWrite,
						  output [2:0] o_ALUControl,
						  output [1:0] o_FlagW
						  );
// decode the instruction

wire s;
wire add_instr, sub_instr, and_instr, orr_instr;
wire mov_instr, cmp_instr, lsl_instr;

assign s = i_funct[0] & i_ALUOp;

// ADD reg: 0_0100
assign add_instr = ~i_funct[4] & i_funct[3] 
							& ~i_funct[2] & ~i_funct[1];

// SUB reg: 0_0010
assign sub_instr = ~i_funct[4] & ~i_funct[3] 
							&  i_funct[2] & ~i_funct[1];

// AND reg: 0_0000
assign and_instr = ~i_funct[4] & ~i_funct[3] 
							& ~i_funct[2] & ~i_funct[1];
			
// ORR reg: 0_1100
assign orr_instr =  i_funct[4] & i_funct[3] 
							& ~i_funct[2] & ~i_funct[1];

// SHFT reg: 0_1101
assign mov_instr =  i_funct[4] & i_funct[3]
							& ~i_funct[2] &  i_funct[1];

// CMP reg: 0_1010
assign cmp_instr =  i_funct[4] & ~i_funct[3] 
							&  i_funct[2] & ~i_funct[1];

// assign outputs
assign o_NoWrite = cmp_instr;

assign o_ALUControl[2:0] = ~i_ALUOp  ? 3'b000 :
								  add_instr  ? 3'b000 :
			    (sub_instr | cmp_instr) ? 3'b001 :
									and_instr ? 3'b100 :
									orr_instr ? 3'b101 :
									mov_instr ? 3'b010 :
													3'b000 ;
									
assign o_FlagW[1] = cmp_instr | (s & (and_instr | orr_instr | add_instr | sub_instr)) ? 1'b1 : 1'b0;
assign o_FlagW[0] = cmp_instr | (s & (add_instr | sub_instr)) ? 1'b1 : 1'b0;
																								
endmodule

//----------------------------------------------------------------------------------------------//

module conditional_logic( input clk,
								  input [3:0] i_cond,
								  input [3:0] i_ALUFlags,
								  input [1:0] i_FlagW,
								  input i_NoWrite,
								  input i_RegW,
								  input i_MemW,
								  output o_RegWrite,
								  output o_MemWrite,
								  // deneme
								  output [3:0] flags
								  );
// flag write
reg CO, OVF, N, Z;
always@(posedge clk)
begin
	if(i_FlagW[1])
	begin
		CO <= i_ALUFlags[3];
		OVF <= i_ALUFlags[2];
	end
	if(i_FlagW[0])
	begin
		N <= i_ALUFlags[1];
		Z <= i_ALUFlags[0];
	end
end

// assign RegWrite and MemWrite
wire CondEx;
assign CondEx = (i_cond[3:0] == 4'd0) ? Z : (i_cond[3:0] == 4'd1) ? ~Z :
					 (i_cond[3:0] == 4'd2) ? CO : (i_cond[3:0] == 4'd3) ? ~CO :
					 (i_cond[3:0] == 4'd4) ? N : (i_cond[3:0] == 4'd5) ? ~N :
					 (i_cond[3:0] == 4'd6) ? OVF : (i_cond[3:0] == 4'd7) ? ~OVF :
					 (i_cond[3:0] == 4'd8) ? (~Z) & CO : (i_cond[3:0] == 4'd9) ? Z | (~CO) :
					 (i_cond[3:0] == 4'd10) ? ~(N ^ OVF) : (i_cond[3:0] == 4'd11) ? (N ^ OVF) :
					 (i_cond[3:0] == 4'd12) ? (~Z) & ~(N ^ OVF) : (i_cond[3:0] == 4'd13) ? Z | (N ^ OVF) :
					 1'b1;
assign o_MemWrite = CondEx & i_MemW;
assign o_RegWrite = CondEx & ~i_NoWrite & i_RegW;

// deneme
assign flags = {CO, OVF, N, Z};
endmodule

//----------------------------------------------------------------------------------------------//

module control_unit( input clk,
							input [1:0] i_op,
							input [5:0] i_funct,
							input [3:0] i_cond,
							input [3:0] i_ALUFlags,
							output o_RegWrite,
							output o_MemWrite,
							output o_MemtoReg,
							output o_ALUSrc,
							output o_RegSrc,
							output [2:0] o_ALUControl,
							// deneme
							output [3:0] flags
							);
							
wire RegW, MemW, NoWrite, ALUOp;
wire [1:0] FlagW;


main_decoder main_dec( .i_op(i_op), .i_funct(i_funct), .o_RegW(RegW), 
							  .o_MemW(MemW), .o_ALUOp(ALUOp),
							  .o_MemtoReg(o_MemtoReg), .o_ALUSrc(o_ALUSrc),
							  .o_RegSrc(o_RegSrc) );
							  
alu_decoder alu_dec ( .i_funct(i_funct[4:0]), .i_ALUOp(ALUOp),
							 .o_NoWrite(NoWrite), .o_ALUControl(o_ALUControl),
						    .o_FlagW(FlagW) );

conditional_logic ( .clk(clk), .i_cond(i_cond), .i_ALUFlags(i_ALUFlags),
						  .i_FlagW(FlagW), .i_RegW(RegW), .i_MemW(MemW),
						  .i_NoWrite(NoWrite), .o_RegWrite(o_RegWrite), 
						  .o_MemWrite(o_MemWrite), .flags(flags) );
endmodule
