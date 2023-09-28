//===========================================================
//
//			Mahdi Hemasian & 98101458
//
//			Implemented Instructions are:
//			R format:  add(u), sub(u), and, or, xor, nor, slt, sltu;
//			I format:  beq, bne, lw, sw, addiu, slti, sltiu, andi, ori, xori, lui.
//
//===========================================================

`timescale 1ns/1ns
`define ADD  4'b0010
`define SUB  4'b0110
`define SLT  4'b0111
`define SLTU 4'b0011
`define AND  4'b0000
`define XOR  4'b0100
`define OR   4'b0001
`define NOR  4'b0101
`define LUI	4'b1000

module pipelined_mips
(
	input clk,
	input reset
);
 
	initial begin
		$display("Pipelined ACA-MIPS Implemention");
		$display("Mahdi Hemasian & 98101458");
	end

	reg [31:0] PC;          // Keep PC as it is, its name is used in higher level test bench
	reg [11:0] regControl2;
	reg [2:0] regControl3;
	reg [1:0] regControl4;
	reg [31:0] regPc1;
	reg [31:0] regPc2;
	reg [31:0] reg1;
	reg	[31:0] reg2RD1;
	reg	[31:0] reg2RD2;
	reg	[14:0] reg2RE;
	reg	[31:0] reg2SignImm;
	reg	[31:0] reg3ALU;
	reg	[31:0] reg3WD;
	reg	[4:0] reg3WR;
	reg	[31:0] reg4ReadData;
	reg	[31:0] reg4ALU;
	reg	[4:0] reg4WR;
	
	
	wire [31:0]instr;
	wire MemtoReg;
	wire MemWrite;
	wire Branch1;
	wire Branch2;
	wire [3:0] ALUcontrol;
	wire ALUSrc;
	wire RegDst;
	wire RegWrite;
	wire ext;
	wire [31:0]SignImm;
	wire [31:0] PC2;
	wire [31:0]ALUResult;
	wire [31:0]SrcA;
	wire [31:0]SrcB;
	wire [31:0]RD1;
	wire [31:0]RD2;
	wire zero;
	wire PCSrc;
	wire [31:0]PCPlus4;
	wire [31:0]PCBranch;
	wire [31:0]WriteData;
	wire [31:0]ReadData;
	wire [31:0]Result;
	wire [4:0] WriteReg;
	wire Stall;
	wire [1:0] ForwardAE;
	wire [1:0] ForwardBE;
	
	// YOUR DESIGN COMES HERE
	assign SignImm = ext ? ({{16{reg1[15]}},reg1[15:0]}) : ({16'h0000,reg1[15:0]});
	assign PCSrc = regControl2[9] ? (zero ? 1 : 0) : (regControl2[8] ? (zero ? 0 : 1) : 0);
	assign PCPlus4 = PC+4;
	assign PCBranch = (regPc2 + (reg2SignImm << 2));
	assign PC2 = PCSrc ? PCBranch : PCPlus4;
	assign SrcB = regControl2[3] ? reg2SignImm : WriteData;
	assign Result = regControl4[1] ? reg4ReadData : reg4ALU;
	assign WriteReg = regControl2[2] ? reg2RE[4:0] : reg2RE[10:5];
	assign SrcA = ForwardAE[1] ? reg3ALU : ForwardAE[0] ? Result : reg2RD1;
	assign WriteData = ForwardBE[1] ? reg3ALU : ForwardBE[0] ? Result : reg2RD2;
	
	
	
	always @ (posedge clk , posedge reset)
	begin
	
		if (reset) begin
			PC = 32'h00000000;
			regControl2 = 12'h000;
			reg1 = 32'h00000000;
		end
			
		if(!Stall) begin
			PC = PC2;
		
			regControl4 = {regControl3[2], regControl3[0]};
			regControl3 = {regControl2[11:10], regControl2[1]};
			regControl2 = PCSrc ? 12'h000 : {MemtoReg, MemWrite, Branch1, Branch2, ALUcontrol, ALUSrc, RegDst, RegWrite, ext};
			
			regPc2 = regPc1;
			regPc1 = PCPlus4;
		
			reg1 = PCSrc ? 32'h0000 : instr;
		
			reg2RD1 <= RD1;
			reg2RD2 <= RD2;
			reg2RE <= reg1[25:11];
			reg2SignImm <= SignImm;
		
			reg3ALU <= ALUResult;
			reg3WD <= WriteData;
			reg3WR <= WriteReg;
		
			reg4ReadData <= ReadData;
			reg4ALU <= reg3ALU;
			reg4WR <= reg3WR;
		end
		else begin
			PC <= PC;
		
			regControl2 <= 12'h000;
			regControl3 <= {regControl2[11:10], regControl2[1]};
			regControl4 <= {regControl3[2], regControl3[0]};
		
			regPc1 <= regPc1;
			regPc2 <= 0;
		
			reg1 <= reg1;
		
			reg2RD1 <= 0;
			reg2RD2 <= 0;
			reg2RE <= 0;
			reg2SignImm <= 0;
		
			reg3ALU <= ALUResult;
			reg3WD <= WriteData;
			reg3WR <= WriteReg;
		
			reg4ReadData <= ReadData;
			reg4ALU <= reg3ALU;
			reg4WR <= reg3WR;
		end
	end
	
//========================================================== 
//	instantiated modules
//========================================================== 

//	Instruction Memory
	async_mem imem			// keep the exact instance name
	(
		.clk		   (1'b0),
		.write		(1'b0),		// no write for instruction memory
		.address	   (PC),		   // address instruction memory with pc
		.write_data	(32'bx),
		.read_data	(instr)
	);
	
// Data Memory
	async_mem dmem			// keep the exact instance name
	(
		.clk		   (clk),
		.write		(regControl3[1]),
		.address	   (reg3ALU),
		.write_data	(reg3WD),
		.read_data	(ReadData)
	);
// Control unit
	ControlUnit cont
	(
		.instr(reg1),
		.MemtoReg(MemtoReg),
		.MemWrite(MemWrite),
		.Branch1(Branch1),
		.Branch2(Branch2),
		.ALUcontrol(ALUcontrol),
		.ALUSrc(ALUSrc),
		.RegDst(RegDst),
		.RegWrite(RegWrite),
		.ext(ext)
	);
//register file 
	reg_file RF
	(
		.clk(clk),
		.WR(reg4WR),
		.WD(Result),
		.RR1(reg1[25:21]),
		.RR2(reg1[20:16]),
		.RD1(RD1),
		.RD2(RD2),
		.write(regControl4[0])
	);
// alu
	my_alu alu
	(
		.Op(regControl2[7:4]),
		.A(SrcA),
		.B(SrcB),
		.X(ALUResult),
        .Z(zero)
	);
// hazard unit
	hazardUnit haz
	(
		.Stall(Stall),
		.RsD(reg1[25:21]),
		.RtD(reg1[20:16]),
		.RsE(reg2RE[14:10]),
		.RtE(reg2RE[9:5]),
		.ForwardAE(ForwardAE),
		.ForwardBE(ForwardBE),
		.MemtoRegE(regControl2[11]),
		.WriteRegM(reg3WR),
		.RegWriteM(regControl3[0]),
		.WriteRegW(reg4WR),
		.RegWriteW(regControl4[0])
	);
endmodule

module hazardUnit
	(
		input wire RsD,
		input wire RtD,
		input wire RsE,
		input wire RtE,
		input wire MemtoRegE,
		input wire WriteRegM,
		input wire RegWriteM,
		input wire WriteRegW,
		input wire RegWriteW, 
		output reg Stall,
		output reg [1:0] ForwardAE,
		output reg [1:0] ForwardBE
	);
	
	always @(*) begin
	////forward A
		if ((RsE != 0) && (RsE == WriteRegM) && RegWriteM)
			ForwardAE = 2'b10;
		else if ((RsE != 0) && (RsE == WriteRegW) && RegWriteW)
			ForwardAE = 2'b01;
		else
			ForwardAE = 2'b00;
	////forward B		
		if ((RtE != 0) && (RtE == WriteRegM) && RegWriteM)
			ForwardBE = 2'b10;
		else if ((RtE != 0) && (RtE == WriteRegW) && RegWriteW)
			ForwardBE = 2'b01;
		else
			ForwardBE = 2'b00;
	////stall for lw		
		if(((RsD == RtE) || (RtD == RtE)) && MemtoRegE)
			Stall = 1;
		else 
			Stall = 0;
	end
	
endmodule


module ControlUnit
	(
		input wire [31:0]instr,
		output reg MemtoReg,
		output reg MemWrite,
		output reg Branch1,
		output reg Branch2,
		output reg [3:0] ALUcontrol,
		output reg ALUSrc,
		output reg RegDst,
		output reg RegWrite,
		output reg ext
	);
	reg [5:0] op;
	reg [5:0] funct;
	always @ (*)
	begin
	op =instr[31:26];
	funct = instr[5:0];
	if (!op)
	begin
		ext=1'bx;
		RegWrite = 1;
		RegDst = 1;
		ALUSrc = 0;
		Branch1 = 0;
		Branch2 = 0;
		MemWrite = 0;
		MemtoReg = 0;
		case(funct)
			6'b000000 : RegWrite = 0;
			6'b100000 : ALUcontrol = 4'b0010;
			6'b100010 : ALUcontrol = 4'b0110;
			6'b100001 : ALUcontrol = 4'b0010;
			6'b100011 : ALUcontrol = 4'b0110;
			6'b100100 : ALUcontrol = 4'b0000;
			6'b100101 : ALUcontrol = 4'b0001;
			6'b100110 : ALUcontrol = 4'b0100;
			6'b100111 : ALUcontrol = 4'b0101;
			6'b101010 : ALUcontrol = 4'b0111;
			6'b101011 : ALUcontrol = 4'b0011;
		endcase
	end   
	else begin
		ext = 1;
		RegWrite = 1;
		RegDst = 0;
		ALUSrc = 1;
		Branch1 = 0;
		Branch2 = 0;
		MemWrite = 0;
		MemtoReg = 0;
		case (op)
			6'b001000 : ALUcontrol = 4'b0010; //addi
			6'b001001 : ALUcontrol = 4'b0010; //addiu
			6'b001100 : 
						begin
							ALUcontrol = 4'b0000; //andi
							ext=0;
						end
			6'b001101 : 
						begin
							ALUcontrol = 4'b0001; //ori
							ext=0;
						end
			6'b001010 : ALUcontrol = 4'b0111; //slti
			6'b001011 : 
						begin
							ALUcontrol = 4'b0011; //sltiu
							ext = 0;
						end
			6'b001110 : 
						begin
							ALUcontrol = 4'b0100; //xori
							ext = 0;
						end
			6'b100011 : //lw
						begin
							RegWrite = 1;
							RegDst = 0;
							ALUSrc = 1;
							Branch1 = 0;
							Branch2 = 0;
							MemWrite = 0;
							MemtoReg = 1;
							ALUcontrol = 4'b0010;
						end
			6'b101011 : //sw
						begin
							RegWrite = 0;
							RegDst = 1'bx;
							ALUSrc = 1;
							Branch1 = 0;
							Branch2 = 0;
							MemWrite = 1;
							MemtoReg = 1'bx;
							ALUcontrol = 4'b0010;
						end
			6'b001111 : ALUcontrol = 4'b1000;//lui
			6'b000100 : //beq
						begin
							RegWrite = 0;
							RegDst = 1'bx;
							ALUSrc = 0;
							Branch1 = 1;
							Branch2 = 0;
							MemWrite = 0;
							MemtoReg = 1'bx;
							ALUcontrol = 4'b0110;
						end	
							
			6'b000101 : //bne
						begin
							RegWrite = 0;
							RegDst = 1'bx;
							ALUSrc = 0;
							Branch1 = 0;
							Branch2 = 1;
							MemWrite = 0;
							MemtoReg = 1'bx;
							ALUcontrol = 4'b0110;
						end	
										
				
		endcase
	end 
end		
endmodule

///////////////////////////////////////////////

module my_alu
	(
		input [3:0]  Op,
		input [31:0]  A,
		input [31:0]  B,
		output [31:0] X,
		output        Z
	);

   wire sub = Op != `ADD;
   wire [31:0] bb = sub ? ~B : B;
   wire [32:0] sum = A + bb + sub;
   wire sltu = ! sum[32];
   wire v = sub ? 
        ( A[31] != B[31] && A[31] != sum[31] )
      : ( A[31] == B[31] && A[31] != sum[31] );

   wire slt = v ^ sum[31];
   reg [31:0] x;

   always @( * )
      case( Op )
         `ADD : x = sum;
         `SUB : x = sum;
         `SLT : x = slt;
         `SLTU: x = sltu;
         `AND : x =   A & B;
         `OR  : x =   A | B;
         `NOR : x = ~(A | B);
         `XOR : x =   A ^ B;
		  `LUI : x = {B[15:0],16'h0000};
         default : x = 32'hxxxxxxxx;
      endcase

   assign  X = x;
   assign  Z = x == 32'h00000000;

endmodule
