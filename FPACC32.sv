module FPACC32 (
	input wire CLK, CLEAR, ACT, TAGi,
	input wire [31:0] A,
	output wire SIGN, ZERO, INF, NAN, RDY, TAGo,
	output wire [31:0] R
	);

integer i;

reg [31:0] AccReg;
reg ACTFlag, ClearFlag, TagFlag, NanReg, InfReg, ZeroReg;
logic [7:0] SParA, SParB, IntExponent, AddExponent, AdderExponent, OutExponent;
logic SignA, SignB, NanA, NanB, InfA, InfB, IntSignA, IntSignB, IntNan, IntInf, AddNan, AddInf, AdderNan, AdderInf, OutSign, OutInf, OutNan;
logic [22:0] MantisaA, MantisaB;
logic [23:0] IntMantisaA, IntMantisaB;
logic [25:0] AddMantisaA, AddMantisaB, AdderMantisa, OutMantisa, ResultMantisa;
logic [4:0] NormValWire, NormValue;

assign RDY=ACTFlag, R=AccReg, TAGo=TagFlag, INF=InfReg, NAN=NanReg, ZERO=ZeroReg, SIGN=AccReg[31];

//=================================================================================================
//			Logic
always_comb
begin

// creating shift parameters
SParA=(R[30:23]-A[30:23]) & {8{A[30:23]<R[30:23]}};
SParB=(A[30:23]-R[30:23]) & {8{R[30:23]<A[30:23]}};

// source mantisa
MantisaA=A[22:0];
MantisaB=R[22:0];

// sign of operand
SignA=A[31];
SignB=R[31];

// Nan detection
NanA=&A[30:23] & (|A[22:0]);
NanB<=&R[30:23] & (|R[22:0]);

// infinity detection
InfA=&A[30:23] & (~(|A[22:0]));
InfB=&R[30:23] & (~(|R[22:0]));

//-----------------------------------------------
//		mantisa shift stage
IntMantisaA=({1'b1, MantisaA}>>SParA[4:0]) & {24{SParA<8'd24}};
IntMantisaB=({1'b1, MantisaB}>>SParB[4:0]) & {24{SParB<8'd24}};
// common exponent
IntExponent=(A[30:23] & {8{(A[30:23]>=R[30:23])}})|(R[30:23] & {8{R[30:23]>A[30:23]}});
// flags
IntSignA=SignA;
IntSignB=SignB;
IntNan=NanA | NanB;
IntInf=(InfA | InfB) & ~NanA & ~NanB;

//-----------------------------------------------
//		operand negation
AddMantisaA={{2{IntSignA}}, (IntMantisaA ^ {24{IntSignA}})+{23'd0, IntSignA}};
AddMantisaB={{2{IntSignB}}, (IntMantisaB ^ {24{IntSignB}})+{23'd0, IntSignB}};
AddExponent=IntExponent;
AddNan=IntNan;
AddInf=IntInf;

//-----------------------------------------------
//		mantisa adder
//AdderMantisa=AddMantisaA+AddMantisaB;
//AdderExponent=AddExponent;
//AdderNan=AddNan;
//AdderInf=AddInf;

//-----------------------------------------------
//		result negation
OutMantisa=(AdderMantisa ^ {26{AdderMantisa[25]}})+{25'd0, AdderMantisa[25]};
OutExponent=AdderExponent;
OutSign=AdderMantisa[25];
OutInf=AdderInf;
OutNan=AdderNan;

NormValWire=5'd0;
for (i=0; i<26; i=i+1)
		if (OutMantisa[i]) NormValWire=i[4:0];
NormValue=5'd25-NormValWire;

ResultMantisa=OutMantisa << NormValue;

end

//=================================================================================================
//			clock
always_ff @(posedge CLK)
begin

//-----------------------------------------------
//		operand negation
//AddMantisaA<={{2{IntSignA}}, (IntMantisaA ^ {24{IntSignA}})+{23'd0, IntSignA}};
//AddMantisaB<={{2{IntSignB}}, (IntMantisaB ^ {24{IntSignB}})+{23'd0, IntSignB}};
//AddExponent<=IntExponent;
//AddNan<=IntNan;
//AddInf<=IntInf;

//-----------------------------------------------
//		mantisa adder
AdderMantisa<=AddMantisaA+AddMantisaB;
AdderExponent<=AddExponent;
AdderNan<=AddNan;
AdderInf<=AddInf;


//-----------------------------------------------
// 		create the result
ACTFlag<=ACT;
ClearFlag<=CLEAR;
TagFlag<=TAGi;

if (ACTFlag | ClearFlag)
	begin
	AccReg[31]<=OutSign & ~ClearFlag;
	AccReg[30:23]<=((OutExponent-{3'd0,NormValue}+8'd2) | {8{OutNan | OutInf}}) & {8{~ClearFlag}};
	AccReg[22:0]<=((ResultMantisa[24:2] & {23{~OutInf}}) | {23{OutNan}})  & {23{~ClearFlag}};
	NanReg<=(NanReg | OutNan) & ~ClearFlag;
	InfReg<=(InfReg | OutInf) & ~ClearFlag;
	end
ZeroReg<=~(|AccReg[30:0]);
	
end

endmodule
