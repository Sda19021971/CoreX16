module FPDIV128 (
	input wire CLK, ACT, RST, CMD,
	input wire [2:0] SA, SB,
	input wire [127:0] A, B,
	input wire [3:0] DSTi,
	
	output reg [127:0] R,
	output reg [3:0] DSTo,
	output reg [2:0] SR,
	output reg RDY, Zero, Sign, Inf, NaN, NEXT
);

logic CanSubtractWire;
reg ProcFlag, SignA, SignB, StartFlag, StopFlag, IntNan, NanA, NanB, IntSign,
	IntInf, InfA, InfB, IntZero, ZeroA, ZeroB, CMDReg, SqrtStartFlag, SqrtStopFlag, SqrtCycleFlag;
reg [111:0] FractionA, FractionB;
reg [14:0] ExponentA, ExponentB, IntExponent, SqrtExponent;
reg [112:0] Dividend, Divisor, Quotient;
reg [113:0] NumReg, BitReg, ResReg, SumReg;
reg [6:0] CntReg, LimitReg;
reg [2:0] Size;
reg [3:0] DSTReg;


assign NEXT=~ProcFlag;


//=================================================================================================
//			Asynchronous logic
always_comb
begin

CanSubtractWire=(Dividend>=Divisor);

end

//=================================================================================================
//			Synchronous
always_ff @(posedge CLK)
begin
//-----------------------------------------------
//	receive source operands
if (~ProcFlag)
	begin
	// mantisa
	FractionA<=SA[2] ? A[111:0] : (SA[0] ? {A[51:0], 60'd0} : {A[22:0], 89'd0});
	FractionB<=SB[2] ? B[111:0] : (SB[0] ? {B[51:0], 60'd0} : {B[22:0], 89'd0});
	// exponent
	ExponentA<=SA[2] ? A[126:112] : (SA[0] ? {4'd0, A[62:52]}+15'd15360 : {7'd0, A[30:23]}+15'd16256);
	ExponentB<=SB[2] ? B[126:112] : (SB[0] ? {4'd0, B[62:52]}+15'd15360 : {7'd0, B[30:23]}+15'd16256);
	// sign
	SignA<=(SA[2] ? A[127] : (SA[0] ? A[63] : A[31])) & ~CMD;
	SignB<=SB[2] ? B[127] : (SB[0] ? B[63] : B[31]);
	// result size
	Size<=CMD ? SB : SA;
	// flags
	NanA<=(SA[2] ? &A[126:112] & (|A[111:0]) : (SA[0] ? &A[62:52] & (|A[51:0]) : &A[30:23] & (|A[22:0]))) & ~CMD;
	NanB<=SB[2] ? &B[126:112] & (|B[111:0]) : (SB[0] ? &B[62:52] & (|B[51:0]) : &B[30:23] & (|B[22:0]));
	InfA<=(SA[2] ? &A[126:112] & (~(|A[111:0])) : (SA[0] ? &A[62:52] & (~(|A[51:0])) : &A[30:23] & (~(|A[22:0])))) & ~CMD;
	InfB<=SB[2] ? &B[126:112] & (~(|B[111:0])) : (SB[0] ? &B[62:52] & (~(|B[51:0])) : &B[30:23] & (~(|B[22:0])));
	ZeroA<=(SA[2] ? ~(|A[126:0]) : (SA[0] ? ~(|A[62:0]) : ~(|A[30:0]))) & ~CMD;
	ZeroB<=SB[2] ? ~(|B[126:0]) : (SB[0] ? ~(|B[62:0]) : ~(|B[30:0]));
	// destination
	DSTReg<=DSTi;
	end
// command
if (~ProcFlag | ~RST) CMDReg<=CMD & RST;

// processing flag
ProcFlag<=(ProcFlag | ACT) & (~StopFlag | ~ProcFlag) & RST & (~ProcFlag | ~SqrtStopFlag);


// division start flag
StartFlag<=ACT & ~ProcFlag & ~CMD;

// square root start flag
SqrtStartFlag<=ACT & ~ProcFlag & CMD & RST;

// square cycle flag
SqrtCycleFlag<=~SqrtCycleFlag & ~SqrtStartFlag & RST;

// sum register
SumReg<=ResReg+BitReg;

if (SqrtCycleFlag | SqrtStartFlag | ~RST) BitReg<=(SqrtStartFlag ? {~ExponentB[0], ExponentB[0], 112'd0} : BitReg >> 2) & {114{RST}};
if (SqrtCycleFlag | SqrtStartFlag)
	begin
	// number register for square root
	if ((NumReg>=SumReg) | SqrtStartFlag) NumReg<=SqrtStartFlag ? {2'b01,FractionB} : NumReg - SumReg;
	// bit register for square root
	// result register for square root
	if (NumReg>=SumReg) ResReg<=((ResReg>>1) + BitReg)  & {114{~SqrtStartFlag}};
		else ResReg<=(ResReg >> 1) & {114{~SqrtStartFlag}};
	end

if (CMDReg & (|BitReg[1:0]) & SqrtCycleFlag) SqrtExponent<={ExponentB[14],~ExponentB[14],ExponentB[13:1]-{12'd0,~ExponentB[0]}};

// square root stop flag
SqrtStopFlag<=CMDReg & (|BitReg[1:0]) & SqrtCycleFlag;

// division stop flag
StopFlag<=(CntReg==LimitReg) & RST;

// dividend
if (StartFlag | CanSubtractWire) Dividend<=StartFlag ? {1'b1, FractionA} : Dividend - Divisor;

// divisor
Divisor<=StartFlag ? {1'b1, FractionB} : Divisor >> 1;

// quotient
if (ProcFlag & ~CMDReg & (CntReg<=LimitReg))
	if (StartFlag) Quotient<=113'd0;
		else Quotient[8'd112-CntReg]<=CanSubtractWire;

// counter
if (StartFlag | ~RST) CntReg<=0;
else if ((CntReg==7'd1) & (IntNan | IntInf | IntZero)) CntReg<=LimitReg;
	else CntReg<=CntReg+{6'd0, ProcFlag & ~CMDReg & (CntReg<=LimitReg)};

// counter limit value
if (StartFlag | ~RST)
	casez ({Size[2],Size[0]})
		2'b00:	LimitReg<=RST ? 7'd23 : 7'd120;
		2'b01:	LimitReg<=RST ? 7'd52 : 7'd120;
		2'b1?:	LimitReg<=RST ? 7'd112 : 7'd120;
		default: LimitReg<=7'd120;
		endcase

// output result
RDY<=(StopFlag | (CMDReg & (|BitReg[1:0]) & SqrtCycleFlag)) & RST;
DSTo<=DSTReg;
SR<=Size;

//-----------------------------------------------
// internal exponent and flags
IntExponent<=ExponentA-ExponentB+15'd16382+{14'd0, Quotient[112]};
IntNan<=NanA | NanB;
IntSign<=SignA ^ SignB;

// create result
casez ({Size[2], Size[0]})
	2'b00:	begin
			IntInf<=(InfA | (IntExponent>15'd16510) | ZeroB) & ~CMDReg;
			IntZero<=(ZeroA | (IntExponent<15'd16255) | (InfB & ~InfA)) & ~ZeroB & ~NanB & ~CMDReg;
			R[22:0]<=CMDReg ? ResReg[55:33] : ((Quotient[112] ? Quotient[111:89] : Quotient[110:88]) & {23{~IntZero & ~IntInf}}) | {23{IntNan}};
			R[30:23]<=(((CMDReg ? SqrtExponent : IntExponent)-15'd16256) & {8{~IntZero}}) | {8{IntInf | IntNan}};
			R[31]<=IntSign;
			R[127:32]<=96'd0;
			end
	2'b01:	begin
			IntInf<=(InfA | (IntExponent>15'd17406) | ZeroB) & ~CMDReg;
			IntZero<=(ZeroA | (IntExponent<15'd15359) | (InfB & ~InfA)) & ~ZeroB & ~NanB & ~CMDReg;
			R[51:0]<=CMDReg ? ResReg[55:4] : ((Quotient[112] ? Quotient[111:60] : Quotient[110:59]) & {52{~IntZero & ~IntInf}}) | {52{IntNan}};
			R[62:52]<=(((CMDReg ? SqrtExponent : IntExponent)-15'd15360) & {11{~IntZero}}) | {11{IntInf | IntNan}};
			R[63]<=IntSign;
			R[127:64]<=64'd0;
			end
	2'b1?:	begin
			IntInf<=(InfA | ZeroB) & ~CMDReg;
			IntZero<=(ZeroA | (InfB & ~InfA)) & ~ZeroB & ~NanB & ~CMDReg;
			R[111:0]<=CMDReg ? {ResReg[55:0], 56'd0} : ((Quotient[112] ? Quotient[111:0] : {Quotient[110:0], 1'b0}) & {112{~IntZero & ~IntInf}}) | {112{IntNan}};
			R[126:112]<=((CMDReg ? SqrtExponent : IntExponent) & {15{~IntZero}}) | {15{IntInf | IntNan}};
			R[127]<=IntSign;
			end
	endcase

Zero<=IntZero;
Sign<=IntSign;
Inf<=IntInf;
NaN<=IntNan;

end

endmodule
