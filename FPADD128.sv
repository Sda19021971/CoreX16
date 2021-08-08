module FPADD128 (
	input wire CLK, RESET, ACT, CMD,
	input wire [127:0] A, B,
	input wire [2:0] SA, SB,
	input wire [3:0] DSTI,
	output wire RDY, SIGN, ZERO, INF, NAN,
	output wire [2:0] SR,
	output wire [3:0] DSTO,
	output wire [127:0] R
	);

integer i;

logic [14:0] ExpABus, ExpBBus;
reg SignA, SignB, IntSignA, IntSignB, NegSignA, NegSignB, NanA, NanB, IntNan, InfA, InfB, IntInf, NegNan, NegInf, AddNan, AddInf, ZeroA, ZeroB;
reg AdderNan, AdderInf, OutNegNan, OutNegInf, OutSign, OutNan, NormNan, ResultNan, ResultInf, ResNan, ResZero, IntAdderNan, IntAdderInf;
reg StartACT, IntACT, NegACT, AddACT, IntAdderACT, AdderACT, OutNegACT, OutACT, ResultACT; 
reg ResSign, ResInf, NormSign, NormInf, ResultZero, ResultSign, OutInf;
reg [14:0] ExpA, ExpB, SParA, SParB, IntExponent, NegExponent, AddExponent, IntAdderExponent, AdderExponent, OutNegExponent, OutExponent,
			NormExponent, ResultExponent;
reg [111:0] MantisaA, MantisaB;
reg [112:0] IntMantisaA, IntMantisaB;
reg [114:0] AddMantisaA, AddMantisaB, AdderMantisa, NormMantisa, OutMantisa, ResultMantisa;
reg [58:0] NegMantisaA_Low, NegMantisaB_Low, IntAdder_Low, OutNegMantisa_Low;
reg [54:0] NegMantisaA_High, NegMantisaB_High;
reg [56:0] IntAdderA_High, IntAdderB_High, OutNegMantisa_High;
reg [2:0] StartSR, IntSR, NegSR, AddSR, AdderSR, OutNegSR, OutSR, NormSR, ResultSR, IntAdderSR, ResSR;
reg [127:0] ResReg;
reg [7:0] NormValue;
reg [3:0] StartDST, IntDST, NegDST, AddDST, IntAdderDST, AdderDST, OutNegDST, OutDST, ResultDST;
logic [7:0] NormValWire;


assign R=ResReg;
assign RDY=ResultACT;
assign DSTO=ResultDST;
assign SIGN=ResSign;
assign INF=ResInf;
assign NAN=ResNan;
assign ZERO=ResZero;
assign SR=ResSR;

//=================================================================================================
//			logic
always_comb
begin

ExpABus=SA[2] ? A[126:112] : (SA[0] ? {4'd0, A[62:52]} + 15'd15360 : {7'd0, A[30:23]} + 15'd16256);
ExpBBus=SB[2] ? B[126:112] : (SB[0] ? {4'd0, B[62:52]} + 15'd15360 : {7'd0, B[30:23]} + 15'd16256);

NormValWire=8'd0;
for (i=0; i<114; i=i+1)
		if (OutMantisa[i]) NormValWire=i[7:0];

end

//=================================================================================================
//			clock
always_ff @(posedge CLK)
begin

//-----------------------------------------------
// 		first stage
// source exponent for single and double channel
ExpA<=ExpABus;
ExpB<=ExpBBus;
// source mantisa
MantisaA<=SA[2] ? A[111:0] : (SA[0] ? {A[51:0], 60'd0} : {A[22:0], 89'd0});
MantisaB<=SB[2] ? B[111:0] : (SB[0] ? {B[51:0], 60'd0} : {B[22:0], 89'd0});
// creating shift parameters
SParA<=(ExpBBus-ExpABus) & {15{ExpABus<ExpBBus}};
SParB<=(ExpABus-ExpBBus) & {15{ExpBBus<ExpABus}};
// sign of operand
SignA<=SA[2] ? A[127] : (SA[0] ? A[63] : A[31]);
SignB<=(SB[2] ? B[127] : (SB[0] ? B[63] : B[31]))^CMD;
// Nan detection
NanA<=SA[2] ? &A[126:112] & (|A[111:0]) : (SA[0] ? &A[62:52] & (|A[51:0]) : &A[30:23] & (|A[22:0]));
NanB<=SB[2] ? &B[126:112] & (|B[111:0]) : (SB[0] ? &B[62:52] & (|B[51:0]) : &B[30:23] & (|B[22:0]));
// infinity detection
InfA<=SA[2] ? &A[126:112] & (~(|A[111:0])) : (SA[0] ? &A[62:52] & (~(|A[51:0])) : &A[30:23] & (~(|A[22:0])));
InfB<=SB[2] ? &B[126:112] & (~(|B[111:0])) : (SB[0] ? &B[62:52] & (~(|B[51:0])) : &B[30:23] & (~(|B[22:0])));
// zero flags
ZeroA<=((~(|A[126:63]) | ~SA[2])&(~(|A[62:31])|(SA==3'd2))&(~(|A[30:0])));
ZeroB<=((~(|B[126:63]) | ~SB[2])&(~(|B[62:31])|(SB==3'd2))&(~(|B[30:0])));
// result size selection
StartSR[0]<=(SA[0] | SB[0]) & ~SA[2] & ~SB[2];
StartSR[1]<=~SA[2] & ~SB[2];
StartSR[2]<=SA[2] | SB[2];
// activation and destination
StartACT<=ACT & RESET;
StartDST<=DSTI;

//-----------------------------------------------
//		mantisa shift stage
IntMantisaA<=({1'b1, MantisaA}>>(SParA[7:0] & {8{~ZeroA & ~ZeroB}})) & {113{SParA<15'd113}};
IntMantisaB<=({1'b1, MantisaB}>>(SParB[7:0] & {8{~ZeroA & ~ZeroB}})) & {113{SParB<15'd113}};
// common exponent
IntExponent<=(ExpA & {15{~ZeroA & (ExpA>=ExpB)}})|(ExpB & {15{~ZeroB & (ExpB>ExpA)}});
// sign
IntSignA<=SignA & ~ZeroA;
IntSignB<=SignB & ~ZeroB;
IntNan<=NanA | NanB;
IntInf<=(InfA | InfB) & ~NanA & ~NanB;
IntSR<=StartSR;
IntACT<=StartACT & RESET;
IntDST<=StartDST;

//-----------------------------------------------
//		first stage of the operand negation
NegMantisaA_Low<={1'b0, IntMantisaA[57:0] ^ {58{IntSignA}}}+{58'd0, IntSignA};
NegMantisaA_High<=IntMantisaA[112:58];
NegMantisaB_Low<={1'b0, IntMantisaB[57:0] ^ {58{IntSignB}}}+{58'd0, IntSignB};
NegMantisaB_High<=IntMantisaB[112:58];
NegExponent<=IntExponent;
NegSignA<=IntSignA & (|IntMantisaA);
NegSignB<=IntSignB & (|IntMantisaB);
NegNan<=IntNan;
NegInf<=IntInf;
NegSR<=IntSR;
NegACT<=IntACT & RESET;
NegDST<=IntDST;

//-----------------------------------------------
//		second stage of the operand negation
AddMantisaA<={{2{NegSignA}}, ((NegMantisaA_High ^ {55{NegSignA}})+{54'd0, NegMantisaA_Low[58]}), NegMantisaA_Low[57:0]};
AddMantisaB<={{2{NegSignB}}, ((NegMantisaB_High ^ {55{NegSignB}})+{54'd0, NegMantisaB_Low[58]}), NegMantisaB_Low[57:0]};
AddExponent<=NegExponent;
AddNan<=NegNan;
AddInf<=NegInf;
AddSR<=NegSR;
AddACT<=NegACT & RESET;
AddDST<=NegDST;

//-----------------------------------------------
//		first stage of the mantisa adder
IntAdder_Low<={1'b0, AddMantisaA[57:0]}+{1'b0, AddMantisaB[57:0]};
IntAdderA_High<=AddMantisaA[114:58];
IntAdderB_High<=AddMantisaB[114:58];
IntAdderExponent<=AddExponent;
IntAdderNan<=AddNan;
IntAdderInf<=AddInf;
IntAdderSR<=AddSR;
IntAdderACT<=AddACT & RESET;
IntAdderDST<=AddDST;

//-----------------------------------------------
//		second stage of the mantisa adder
AdderMantisa<={IntAdderA_High+IntAdderB_High+{56'd0, IntAdder_Low[58]}, IntAdder_Low[57:0]};
AdderExponent<=IntAdderExponent;
AdderNan<=IntAdderNan;
AdderInf<=IntAdderInf;
AdderSR<=IntAdderSR;
AdderACT<=IntAdderACT & RESET;
AdderDST<=IntAdderDST;

//-----------------------------------------------
//		First stage of result negation
OutNegMantisa_Low<={1'b0, AdderMantisa[57:0] ^ {58{AdderMantisa[114]}}}+{58'd0, AdderMantisa[114]};
OutNegMantisa_High<=AdderMantisa[114:58];
OutNegExponent<=AdderExponent;
OutNegNan<=AdderNan;
OutNegInf<=AdderInf;
OutNegSR<=AdderSR;
OutNegACT<=AdderACT & RESET;
OutNegDST<=AdderDST;

//-----------------------------------------------
//		Second stage result negation
OutMantisa<={(OutNegMantisa_High ^ {57{OutNegMantisa_High[56]}})+{56'd0, OutNegMantisa_Low[58]}, OutNegMantisa_Low[57:0]};
OutExponent<=OutNegExponent;
OutSign<=OutNegMantisa_High[56];
OutInf<=OutNegInf;
OutNan<=OutNegNan;
OutSR<=OutNegSR;
OutACT<=OutNegACT & RESET;
OutDST<=OutNegDST;

//-----------------------------------------------
//		result normalisation
NormMantisa<=OutMantisa;
NormExponent<=OutExponent;
NormValue<=8'd113-NormValWire;
NormSign<=OutSign;
NormInf<=OutInf;
NormNan<=OutNan;
NormSR<=OutSR;

//-----------------------------------------------
// 		finaly result
ResultMantisa<=NormMantisa << NormValue;
// create infinity value
case ({NormSR[2], NormSR[0]})
	2'b00:	begin
			ResultInf<=NormInf | (((NormExponent-NormValue)>15'd16509) & ~NormNan);
			ResultZero<=(~(|NormMantisa) | ((NormExponent-NormValue)<15'd16257)) & ~NormNan & ~NormInf;
			ResultExponent<=NormExponent-NormValue-15'd16255;
			end
	2'b01:	begin
			ResultInf<=NormInf | (((NormExponent-NormValue)>15'd17405) & ~NormNan);
			ResultZero<=(~(|NormMantisa) | ((NormExponent-NormValue)<15'd15362)) & ~NormNan & ~NormInf;
			ResultExponent<=NormExponent-NormValue-15'd15359;
			end
	default:	begin
				ResultInf<=NormInf;
				ResultZero<=1'b0;
				ResultExponent<=NormExponent-NormValue+15'd1;
				ResultZero<=(~(|NormMantisa)) & ~NormNan & ~NormInf;
				end
	endcase

ResultSign<=NormSign;
ResultNan<=NormNan;
ResultSR<=NormSR;
ResultACT<=OutACT & RESET;
ResultDST<=OutDST;

//-----------------------------------------------
//		create result
casez ({ResultSR[2], ResultSR[0]})
	2'b00:	ResReg<={96'd0, ResultSign, (ResultExponent[7:0] & {8{~ResultZero}}) | {8{ResultNan | ResultInf}}, (ResultMantisa[112:90] & {23{~ResultInf}}) | {23{ResultNan}}};
	2'b01:	ResReg<={64'd0, ResultSign, (ResultExponent[10:0] & {11{~ResultZero}}) | {11{ResultNan | ResultInf}}, (ResultMantisa[112:61] & {52{~ResultInf}}) | {52{ResultNan}}};
	default:	ResReg<={ResultSign, (ResultExponent & {15{~ResultZero}}) | {15{ResultNan | ResultInf}}, (ResultMantisa[112:1] & {112{~ResultInf}}) | {112{ResultNan}}};
	endcase

ResSign<=ResultSign;
ResNan<=ResultNan;
ResInf<=ResultInf;
ResZero<=ResultZero;
ResSR<=ResultSR;

end

endmodule
