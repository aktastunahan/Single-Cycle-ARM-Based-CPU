/*
	REGISTER FILE FOR THE ARM BASED SINGLE CYCLE COMPUTER DESIGN.
	
	INPUTS:
	(Read)  Address1: A1 W-bits
	(Read)  Address2: A2 W-bits 
	(Write) Address3: A3 W-bits
	(Write) Enable: WE3: 1 bit
	(Write) Content: WD3 W-bits
	PC+8: R15
	
	OUTPUTS:
	RD1: R[A1] W-bits
	RD2: R[A2] W-bits
*/
module register_file #(parameter W=32) (
								input clk,
								input rst,
								input [3:0]  i_A1, 			// Address1
								input [3:0]  i_A2, 			// Address2
								input [3:0]  i_A3, 			// Address3
								input i_WE3,					// Write enable 3
								input [W-1:0] i_WD3, 		// Write content 3
								input [W-1:0] i_R15, 		// PC + 8
								output [W-1:0] o_RD1,  		// Content in register R[A1]
								output [W-1:0] o_RD2,   	// Content in register R[A2]
								output [W-1:0] o_test_r0,
								output [W-1:0] o_test_r1,
								output [W-1:0] o_test_r2,
								output [W-1:0] o_test_r3
								);
// Combinational read part
// 32-bit 14 registers
reg [W-1:0] registers [0:14];

// assign read ports
assign o_RD1 = registers[i_A1];
assign o_RD2 = registers[i_A2];

// test register
assign o_test_r0 = registers[0];
assign o_test_r1 = registers[1];
assign o_test_r2 = registers[2];
assign o_test_r3 = registers[3];
	
// sequential write part
always@(posedge clk)
begin
	if(rst)
	begin
		registers[0]  <= 0;
		registers[1]  <= 0;
		registers[2]  <= 0;
		registers[3]  <= 0;
		registers[4]  <= 0;
		registers[5]  <= 0;
		registers[6]  <= 0;
		registers[7]  <= 0;
		registers[8]  <= 0;
		registers[9]  <= 0;
		registers[10] <= 0;
		registers[11] <= 0;
		registers[12] <= 0;
		registers[13] <= 0;
		registers[14] <= 0;
	end
	else if(i_WE3)
		registers[i_A3] <= i_WD3;
end
endmodule
