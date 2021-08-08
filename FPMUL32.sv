module FPMUL32 (
	input wire CLK, RESET,
	input wire ACT, TAGi,
	input wire [31:0] A, B,
	// 32-bit channel
	output reg RDY, TAGo,
	output reg [31:0] R
	);

logic [47:0] RM32;

logic [23:0] MantissaA, MantissaB;
logic [7:0] ExponentA, ExponentB;
reg [8:0] ExpR;
logic SignA, ZeroA, InfA, NaNA, InfB, NaNB, SignB, ZeroB;
reg SignR, ZeroR, NanR, InfR, SignRR, NanRR;
reg [1:0] ACTReg, TagReg;
reg [24:0] MantissaR;

reg [7:0] Exp32Reg;
reg [22:0] Mantissa32Reg;
reg InfSDReg, ZeroSDReg;


//=================================================================================================
//			Asynchronous part
always_comb
begin

ExponentA=A[30:23];
MantissaA={1'b1, A[22:0]};
SignA=A[31];
ZeroA=~(|A[30:0]);
InfA=(&A[30:23])&(~(|A[22:0]));
NaNA=(&A[30:23])&(|A[22:0]);

ExponentB=B[30:23];
MantissaB={1'b1, B[22:0]};
SignB=B[31];
ZeroB=~(|B[30:0]);
InfB=(&B[30:23])&(~(|B[22:0]));
NaNB=(&B[30:23])&(|B[22:0]);

RM32=MantissaA * MantissaB;

end

//=================================================================================================
//			Synchronous part 
always_ff @(posedge CLK)
begin


ACTReg[0]<=ACT & RESET;
TagReg[0]<=TAGi;

//-----------------------------------------------
//			temporary registers

SignR<=SignA ^ SignB;
ZeroR<=(ZeroA | ZeroB) & ~NaNA & ~NaNB;
NanR<=NaNA | NaNB;
InfR<=(InfA | InfB) & ~NaNA & ~NaNB;
ExpR<={1'b0,ExponentA}+{1'b0,ExponentB};
MantissaR<=RM32[47:23];
ACTReg[1]<=ACTReg[0] & RESET;
TagReg[1]<=TagReg[0];


// data ready signal and tag
RDY<=ACTReg[1] & RESET;
TAGo<=TagReg[1];

Exp32Reg<=(ExpR-9'd127+{8'd0, MantissaR[24]}) | {8{(ExpR>9'd507)}};
InfSDReg<=(InfR | (ExpR>9'd507)) & ~NanR;
ZeroSDReg<=(ZeroR | (ExpR<127)) & ~NanR;
Mantissa32Reg<=((MantissaR[24] ? MantissaR[23:1] : MantissaR[22:0]) & {23{~ZeroR & ~InfR}}) | {23{NanR}};
SignRR<=SignR;
NanRR<=NanR;

// result
R<=({SignRR, Exp32Reg | {8{NanRR | InfSDReg}}, (Mantissa32Reg & {23{~InfSDReg}})|{23{NanRR}}}) & {32{~ZeroSDReg}};

end

endmodule
