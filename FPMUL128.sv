module FPMUL128 (
	input wire CLK, RESET,
	input wire ACT,
	input wire [3:0] DSTI,
	input wire [127:0] A, B,
	input wire [2:0] SA, SB,
	// 32-bit channel
	output reg RDYSD, RDYQ, ZEROSD, SIGNSD, INFSD, NANSD, ZEROQ, SIGNQ, INFQ, NANQ, SR,
	output reg [3:0] DSTSD, DSTQ,
	output wire [63:0] RSD,
	output wire [127:0] RQ
	);

logic [24:0] RM32;
logic [53:0] RM64;
logic [113:0] RM128;

reg [63:0] RSDReg;
reg [127:0] RQReg;

reg [107:0] MantissaA, MantissaB;
reg [15:0] ExponentA, ExponentB;
reg SignA, ZeroA, InfA, NaNA, InfB, NaNB, SignB, ZeroB;
reg [1:0] SizeR;
reg [5:0] Zero, Nan, Inf, Sign;
reg [4:0] ACTReg;
reg [1:0] Size [4:0];
reg [15:0] Exp [5:0];
reg [3:0] DSTReg [4:0];

reg [7:0] Exp32Reg;
reg [22:0] Mantissa32Reg;
logic [10:0] Exp64Bus;
logic [51:0] Mantissa64Bus;
logic [15:0] Exp128Bus;
reg InfSDReg, ZeroSDReg;
logic InfQWire, ZeroQWire;

Mult108 mult (.CLK(CLK), .A(MantissaA), .B(MantissaB), .R32(RM32), .R64(RM64), .R128(RM128));

//=================================================================================================
//			assignments part

assign RSD=RSDReg;
assign RQ=RQReg;


//=================================================================================================
//			Asynchronous part
always_comb
begin

Exp64Bus=(Exp[2]-15'd15360+{14'd0, RM64[53]}) | {11{(Exp[2]>15'd17406)|(Exp[2]<15361)}};
Mantissa64Bus=((RM64[53] ? RM64[52:1] : RM64[51:0]) & {52{~Zero[2] & ~Inf[2]}}) | {52{Nan[2]}};

Exp128Bus=Exp[5]+{15'd0, RM128[113]};
InfQWire=Inf[5] | (Exp128Bus[15] & ~Exp128Bus[14]);
ZeroQWire=Zero[5] | (Exp128Bus[15] & Exp128Bus[14]);

end

//=================================================================================================
//			Synchronous part by low frequency clock
always_ff @(posedge CLK)
begin
	
casez (SA)
		// 64-bit source operand A
		3'b011:	begin
				ExponentA<=16'd15360+{5'd0,A[62:52]};
				MantissaA<={1'b1, A[51:0], 55'd0};
				SignA<=A[63];
				ZeroA<=~(|A[62:0]);
				InfA<=(&A[62:52])&(~(|A[51:0]));
				NaNA<=(&A[62:52])&(|A[51:0]);
				end
		// 128-bit source operand A
		3'b1??:	begin
				ExponentA<={1'b0, A[126:112]};
				MantissaA<={1'b1, A[111:5]};
				SignA<=A[127];
				ZeroA<=~(|A[126:0]);
				InfA<=(&A[126:112])&(~(|A[111:0]));
				NaNA<=(&A[126:112])&(|A[111:0]);
				end
		// 32-bit source operand A
		default:begin
				ExponentA<=16'd16256+{8'd0, A[30:23]};
				MantissaA<={1'b1, A[22:0], 84'd0};
				SignA<=A[31];
				ZeroA<=~(|A[30:0]);
				InfA<=(&A[30:23])&(~(|A[22:0]));
				NaNA<=(&A[30:23])&(|A[22:0]);
				end
		endcase

casez (SB)
		// 64-bit source operand B
		3'b011:	begin
				ExponentB<=16'd15360+{5'd0,B[62:52]};
				MantissaB<={1'b1, B[51:0], 55'd0};
				SignB<=B[63];
				ZeroB<=~(|B[62:0]);
				InfB<=(&B[62:52])&(~(|B[51:0]));
				NaNB<=(&B[62:52])&(|B[51:0]);
				end
		// 128-bit source operand B
		3'b1??:	begin
				ExponentB<={1'b0,B[126:112]};
				MantissaB<={1'b1, B[111:5]};
				SignB<=B[127];
				ZeroB<=~(|B[126:0]);
				InfB<=(&B[126:112])&(~(|B[111:0]));
				NaNB<=(&B[126:112])&(|B[111:0]);
				end
		// 32-bit source operand B
		default:begin
				ExponentB<=16'd16256+{8'd0, B[30:23]};
				MantissaB<={1'b1, B[22:0], 84'd0};
				SignB<=B[31];
				ZeroB<=~(|B[30:0]);
				InfB<=(&B[30:23])&(~(|B[22:0]));
				NaNB<=(&B[30:23])&(|B[22:0]);
				end
		endcase

// result size
SizeR[0]<=SA[0] | SB[0];
SizeR[1]<=SA[2] | SB[2];

//-----------------------------------------------
//			temporary registers

Sign[0]<=SignA ^ SignB;
Zero[0]<=(ZeroA | ZeroB) & ~NaNA & ~NaNB;
Nan[0]<=NaNA | NaNB;
Inf[0]<=(InfA | InfB) & ~NaNA & ~NaNB;
Exp[0]<=ExponentA+ExponentB-16'd16383;
Size[0]<=SizeR;

Sign[1]<=Sign[0];
Zero[1]<=Zero[0];
Nan[1]<=Nan[0];
Inf[1]<=Inf[0];
Exp[1]<=Exp[0];
Size[1]<=Size[0];

Sign[2]<=Sign[1];
Zero[2]<=Zero[1];
Nan[2]<=Nan[1];
Inf[2]<=Inf[1];
Exp[2]<=Exp[1];
Size[2]<=Size[1];

Sign[3]<=Sign[2];
Zero[3]<=Zero[2];
Nan[3]<=Nan[2];
Inf[3]<=Inf[2];
Exp[3]<=Exp[2];
Size[3]<=Size[2];

Sign[4]<=Sign[3];
Zero[4]<=Zero[3];
Nan[4]<=Nan[3];
Inf[4]<=Inf[3];
Exp[4]<=Exp[3];
Size[4]<=Size[3];

Sign[5]<=Sign[4];
Zero[5]<=Zero[4];
Nan[5]<=Nan[4];
Inf[5]<=Inf[4];
Exp[5]<=Exp[4];
//Size[5]<=Size[4];

// activation
ACTReg[0]<=ACT & RESET;
ACTReg[1]<=ACTReg[0] & RESET;
ACTReg[2]<=ACTReg[1] & RESET;
ACTReg[3]<=ACTReg[2] & RESET;
ACTReg[4]<=ACTReg[3] & RESET;

DSTReg[0]<=DSTI;
DSTReg[1]<=DSTReg[0];
DSTReg[2]<=DSTReg[1];
DSTReg[3]<=DSTReg[2];
DSTReg[4]<=DSTReg[3];


//-----------------------------------------------
// 	create 32-bit or 64-bit result

// data ready signal and tag
RDYSD<=ACTReg[1] & ~Size[0][1] & RESET;
DSTSD<=DSTReg[1];

Exp32Reg<=(Exp[1]-15'd16256+{14'd0, RM32[24]}) | {8{(Exp[1]>15'd16510) | (Exp[1]<16257)}};
InfSDReg<=(Inf[1] | (Size[1][0] ? (Exp[1]>15'd17406) : (Exp[1]>15'd16510))) & ~Nan[1];
ZeroSDReg<=(Zero[1] | (Size[1][0] ? (Exp[1]<15361) : (Exp[1]<16257))) & ~Nan[1];
Mantissa32Reg<=((RM32[24] ? RM32[23:1] : RM32[22:0]) & {23{~Zero[1] & ~Inf[1]}}) | {23{Nan[1]}};

// mantissa
RSDReg<=(Size[2][0] ? {Sign[2], Exp64Bus | {11{Nan[2] | InfSDReg}}, (Mantissa64Bus & {52{~InfSDReg}})|{52{Nan[2]}}} : 
					{32'd0, Sign[2], Exp32Reg | {8{Nan[2] | InfSDReg}}, (Mantissa32Reg & {23{~InfSDReg}})|{23{Nan[2]}}}) & {64{~ZeroSDReg}};

// flags
ZEROSD<=ZeroSDReg;
NANSD<=Nan[2];
SIGNSD<=Sign[2];
INFSD<=InfSDReg;
// size of the result
SR<=Size[2][0];

//-----------------------------------------------
// create 128-bit result
RDYQ<=ACTReg[4] & Size[3][1] & RESET;
DSTQ<=DSTReg[4];

RQReg[111:0]<=((RM128[113] ? RM128[112:1] : RM128[111:0]) & {112{~ZeroQWire & ~InfQWire}}) | {112{Nan[5]}};
RQReg[126:112]<=(Exp128Bus[14:0] & {15{~ZeroQWire}}) | {15{InfQWire | Nan[5]}};
RQReg[127]<=Sign[5];

//flags
ZEROQ<=ZeroQWire;
NANQ<=Nan[5];
SIGNQ<=Sign[5];
INFQ<=InfQWire;

end

endmodule
