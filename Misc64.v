/*
BSWAP	010
NEG		011
DAA		000
DAS		001
POS		110
CFZ		100
CFN		101
LOOP	111
*/

module Misc64 (
	input wire CLK, ACT,
	input wire [2:0] OpCODE,
	input wire [2:0] SA, SD,
	
	input wire [3:0] DSTi,
	input wire [15:0] CIN,
	input wire [127:0] A,
	
	output wire RDY,
	output reg ZERO, SIGN, OVR, COUT, NaN,
	output reg [2:0] SR,
	output wire [3:0] DSTo,
	output reg [127:0] R
	);
	
integer i;

reg [2:0] TempOpCODE;
reg [63:0] ABus, DAABus, BSWAPBus, NEGBus;
// CFBus format: {Sign, NAN, INF, Zero, 128-bit value}
reg [131:0] CFBus;
wire [15:0] CTemp;
reg [7:0] PosBus, TempPos;
reg TempINF, TempNaN, TempZero, TempSign;
reg [3:0] CReg;
reg [23:0] Rnd64To32Bus, Rnd128To32Bus;
reg [52:0] Rnd128To64Bus;

	
assign CTemp[0]=(A[3:0]>9);
assign CTemp[1]=(A[7:4]>9) | (CTemp[0] & (A[7:4]==9));
assign CTemp[2]=(A[11:8]>9) | (CTemp[1] & (A[11:8]==9));
assign CTemp[3]=(A[15:12]>9) | (CTemp[2] & (A[15:12]==9));
assign CTemp[4]=(A[19:16]>9) | (CTemp[3] & (A[19:16]==9));
assign CTemp[5]=(A[23:20]>9) | (CTemp[4] & (A[23:20]==9));
assign CTemp[6]=(A[27:24]>9) | (CTemp[5] & (A[27:24]==9));
assign CTemp[7]=(A[31:28]>9) | (CTemp[6] & (A[31:28]==9));
assign CTemp[8]=(A[35:32]>9) | (CTemp[7] & (A[35:32]==9));
assign CTemp[9]=(A[39:36]>9) | (CTemp[8] & (A[39:36]==9));
assign CTemp[10]=(A[43:40]>9) | (CTemp[9] & (A[43:40]==9));
assign CTemp[11]=(A[47:44]>9) | (CTemp[10] & (A[47:44]==9));
assign CTemp[12]=(A[51:48]>9) | (CTemp[11] & (A[51:48]==9));
assign CTemp[13]=(A[55:52]>9) | (CTemp[12] & (A[55:52]==9));
assign CTemp[14]=(A[59:56]>9) | (CTemp[13] & (A[59:56]==9));
assign CTemp[15]=(A[63:60]>9) | (CTemp[14] & (A[63:60]==9));

assign DSTo=DSTi;
assign RDY=ACT;

always @(*)
	begin
	PosBus[5:0]=6'b000000;
	for (i=0; i<64; i=i+1)
		if (A[i]) PosBus[5:0]=i[5:0];
	PosBus[7:6]=2'b00;
	// forming flags (NaN, INF, Zero)
	TempINF=(SA[2] ? &A[126:112] : (SA[0] ? &A[62:52] : &A[30:23])) & (SA[2] ? ~(|A[111:0]) : (SA[0] ? ~(|A[51:0]) : ~(|A[22:0])));
	TempNaN=(SA[2] ? &A[126:112] : (SA[0] ? &A[62:52] : &A[30:23])) & (SA[2] ? |A[111:0] : (SA[0] ? |A[51:0] : |A[22:0]));
	TempZero=SA[2] ? ~(|A[126:0]) : (SA[0] ? ~(|A[62:0]) : ~(|A[30:0]));
	TempSign=SA[2] ? A[127] : (SA[0] ? A[63] : A[31]);
	NEGBus=ABus+DAABus+{(TempOpCODE==3'b001),(TempOpCODE==3'b011)};
	
	// round buses
	Rnd64To32Bus={1'b0, A[51:29]}+{23'd0, OpCODE[0] & A[28]};
	Rnd128To32Bus={1'b0, A[111:89]}+{23'd0, OpCODE[0] & A[88]};
	Rnd128To64Bus={1'b0, A[111:60]}+{52'd0, OpCODE[0] & A[59]};
	end


always @(posedge CLK)
	begin
	TempOpCODE<=OpCODE;
	TempPos<=PosBus;
	// preparing the source operands
	ABus<=A[63:0] ^ {64{(~OpCODE[2] & OpCODE[1] & OpCODE[0])}};
	// BSWAP result
	case (SA[1:0])
		2'b00: for (i=0; i<4; i=i+1)
					begin
					BSWAPBus[i]<=A[7-i];
					BSWAPBus[7-i]<=A[i];
					end
		2'b01: for (i=0; i<8; i=i+1)
					begin
					BSWAPBus[i]<=A[15-i];
					BSWAPBus[15-i]<=A[i];
					end
		2'b10: for (i=0; i<16; i=i+1)
					begin
					BSWAPBus[i]<=A[31-i];
					BSWAPBus[31-i]<=A[i];
					end
		2'b11: for (i=0; i<32; i=i+1)
					begin
					BSWAPBus[i]<=A[63-i];
					BSWAPBus[63-i]<=A[i];
					end
		endcase
	// floating point rounding
	// CFBus format: {Sign, NAN, INF, Zero, 128-bit value}
	casez ({SD[2],SD[0],SA[2],SA[0]})
		// 32-bit to 32 bit
		4'b0000:	CFBus<={TempSign, TempNaN, TempINF, TempZero, A};
		// 64-bit to 32-bit
		4'b0001:	if (TempINF | TempNaN | TempZero) CFBus<={TempSign, TempNaN, TempINF, TempZero, 96'd0, A[63], {8{TempINF | TempNaN}}, {23{TempNaN}}};
						else if ((A[62:52]<11'd1151) & (A[62:52]>11'd896)) 
									begin
									CFBus[22:0]<=Rnd64To32Bus[22:0]; 
									CFBus[30:23]<=(A[62:52]-11'd896)+{7'd0,Rnd64To32Bus[23]};
									CFBus[31]<=A[63];
									CFBus[127:32]<=96'd0;
									CFBus[130:128]<=3'd0;
									CFBus[131]<=TempSign;
									end
								else if (A[62:52]>11'd896) CFBus<={TempSign, 3'b010, 96'd0, A[63], {8{1'b1}}, {23{1'b0}}};
										else CFBus<={TempSign, 3'd1, 128'd0};
		// 128 bit to 32 bit
		4'b001?:	if (TempINF | TempNaN | TempZero) CFBus<={TempSign, TempNaN, TempINF, TempZero, 96'd0, A[127], {8{TempINF | TempNaN}}, {23{TempNaN}}};
						else if ((A[126:112]<15'd16511) & (A[126:112]>15'd16256)) 
									begin
									CFBus[22:0]<=Rnd128To32Bus[22:0];
									CFBus[30:23]<=(A[126:112]-15'd16256)+{7'd0, Rnd128To32Bus[23]};
									CFBus[31]<=A[127];
									CFBus[127:32]<=96'd0;
									CFBus[130:128]<=3'd0;
									CFBus[131]<=TempSign;
									end
								else if (A[126:112]>15'd16256) CFBus<={TempSign, 3'b010, 96'd0, A[63], {8{1'b1}}, {23{1'b0}}};
										else CFBus<={TempSign, 3'd1, 128'd0};
		// 32-bit to 64 bit
		4'b0100:	if (TempINF | TempNaN | TempZero) CFBus<={TempSign, TempNaN, TempINF, TempZero, 64'd0, A[31], {11{TempINF | TempNaN}}, {52{TempNaN}}};
						else begin
							CFBus[28:0]<=29'd0;
							CFBus[51:29]<=A[22:0];
							CFBus[62:52]<=A[30:23]+11'd896;
							CFBus[63]<=A[31];
							CFBus[127:64]<=64'd0;
							CFBus[130:128]<=3'd0;
							CFBus[131]<=TempSign;
							end
		// 64-bit to 64 bit
		4'b0101:	CFBus<={TempSign, TempNaN, TempINF, TempZero, A};
		// 128 bit to 64 bit
		4'b011?:	if (TempINF | TempNaN | TempZero) CFBus<={TempSign, TempNaN, TempINF, TempZero, 64'd0, A[127], {11{TempINF | TempNaN}}, {52{TempNaN}}};
						else if ((A[126:112]<15'd17407) & (A[126:112]>15'd15360)) 
									begin
									CFBus[51:0]<=Rnd128To64Bus[51:0];
									CFBus[62:52]<=(A[126:112]-15'd15360)+{10'd0, Rnd128To64Bus[52]};
									CFBus[63]<=A[127];
									CFBus[127:64]<=64'd0;
									CFBus[130:128]<=3'd0;
									CFBus[131]<=TempSign;
									end
								else if (A[126:112]>15'd15360) CFBus<={TempSign, 3'b010, 64'd0, A[127], {11{1'b1}}, {52{1'b0}}};
										else CFBus<={TempSign, 3'd1, 128'd0};
		// 32 bit to 128 bit
		4'b1?00:	if (TempINF | TempNaN | TempZero) CFBus<={TempSign, TempNaN, TempINF, TempZero, A[31], {15{TempINF | TempNaN}}, {112{TempNaN}}};
						else begin
							CFBus[88:0]<=89'd0;
							CFBus[111:89]<=A[22:0];
							CFBus[126:112]<={7'd0, A[30:23]}+15'd16256;
							CFBus[127]<=A[31];
							CFBus[130:128]<=3'd0;
							CFBus[131]<=TempSign;
							end
		// 64 bit to 128 bit
		4'b1?01:	if (TempINF | TempNaN | TempZero) CFBus<={TempSign, TempNaN, TempINF, TempZero, A[63], {15{TempINF | TempNaN}}, {112{TempNaN}}};
						else begin
							CFBus[59:0]<=60'd0;
							CFBus[111:60]<=A[51:0];
							CFBus[126:112]<={4'd0, A[62:52]}+15'd15360;
							CFBus[127]<=A[63];
							CFBus[130:128]<=3'd0;
							CFBus[131]<=TempSign;
							end
		// 128-bit to 128 bit
		4'b1?1?:	CFBus<={TempSign, TempNaN, TempINF, TempZero, A};
		endcase
	// DAA operand
	DAABus[0]<=(OpCODE==3'b001);
	DAABus[2:1]<={2{(((CIN[0]^OpCODE[0]) | (A[3:0]>9))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[4:3]<={2{(OpCODE==3'b001)}};
	DAABus[6:5]<={2{(((CIN[1]^OpCODE[0]) | (A[7:4]>9) | (~OpCODE[0] & CTemp[0] & (A[7:4]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[8:7]<={2{(OpCODE==3'b001)}};
	DAABus[10:9]<={2{(((CIN[2]^OpCODE[0]) | (A[11:8]>9) | (~OpCODE[0] & CTemp[1] & (A[11:8]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[12:11]<={2{(OpCODE==3'b001)}};
	DAABus[14:13]<={2{(((CIN[3]^OpCODE[0]) | (A[15:12]>9) | (~OpCODE[0] & CTemp[2] & (A[15:12]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[16:15]<={2{(OpCODE==3'b001)}};
	DAABus[18:17]<={2{(((CIN[4]^OpCODE[0]) | (A[19:16]>9) | (~OpCODE[0] & CTemp[3] & (A[19:16]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[20:19]<={2{(OpCODE==3'b001)}};
	DAABus[22:21]<={2{(((CIN[5]^OpCODE[0]) | (A[23:20]>9) | (~OpCODE[0] & CTemp[4] & (A[23:20]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[24:23]<={2{(OpCODE==3'b001)}};
	DAABus[26:25]<={2{(((CIN[6]^OpCODE[0]) | (A[27:24]>9) | (~OpCODE[0] & CTemp[5] & (A[27:24]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[28:27]<={2{(OpCODE==3'b001)}};
	DAABus[30:29]<={2{(((CIN[7]^OpCODE[0]) | (A[31:28]>9) | (~OpCODE[0] & CTemp[6] & (A[31:28]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[32:31]<={2{(OpCODE==3'b001)}};
	DAABus[34:33]<={2{(((CIN[8]^OpCODE[0]) | (A[35:32]>9) | (~OpCODE[0] & CTemp[7] & (A[35:32]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[36:35]<={2{(OpCODE==3'b001)}};
	DAABus[38:37]<={2{(((CIN[9]^OpCODE[0]) | (A[39:36]>9) | (~OpCODE[0] & CTemp[8] & (A[39:36]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[40:39]<={2{(OpCODE==3'b001)}};
	DAABus[42:41]<={2{(((CIN[10]^OpCODE[0]) | (A[43:40]>9) | (~OpCODE[0] & CTemp[9] & (A[43:40]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[44:43]<={2{(OpCODE==3'b001)}};
	DAABus[46:45]<={2{(((CIN[11]^OpCODE[0]) | (A[47:44]>9) | (~OpCODE[0] & CTemp[10] & (A[47:44]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[48:47]<={2{(OpCODE==3'b001)}};
	DAABus[50:49]<={2{(((CIN[12]^OpCODE[0]) | (A[51:48]>9) | (~OpCODE[0] & CTemp[11] & (A[51:48]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[52:51]<={2{(OpCODE==3'b001)}};
	DAABus[54:53]<={2{(((CIN[13]^OpCODE[0]) | (A[55:52]>9) | (~OpCODE[0] & CTemp[12] & (A[55:52]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[56:55]<={2{(OpCODE==3'b001)}};
	DAABus[58:57]<={2{(((CIN[14]^OpCODE[0]) | (A[59:56]>9) | (~OpCODE[0] & CTemp[13] & (A[59:56]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[60:59]<={2{(OpCODE==3'b001)}};
	DAABus[62:61]<={2{(((CIN[15]^OpCODE[0]) | (A[63:60]>9) | (~OpCODE[0] & CTemp[14] & (A[63:60]==9)))^OpCODE[0])&(~OpCODE[1])}};
	DAABus[63]<=(OpCODE==3'b001);
	
	// forming result
	if (TempOpCODE==3'b010) R<={64'd0, BSWAPBus};
		else if (TempOpCODE==3'b110) R<={64'd0, {56{1'b0}}, TempPos};
			else if (TempOpCODE[2:1]==2'b10) R<=CFBus[127:0];
				else if (TempOpCODE==3'b111) R<={64'd0, ABus+{64{1'b1}}};
					else R<={64'd0, NEGBus};
	SR<=(OpCODE[2:1]==2'b10) ? SD:SA;
	CReg[0]<=CTemp[1];
	CReg[1]<=CTemp[3];
	CReg[2]<=CTemp[7];
	CReg[3]<=CTemp[15];
	COUT<=~TempOpCODE[2] & (TempOpCODE[2] | ~TempOpCODE[1] | TempOpCODE[0]) & 
			((CReg[0] & ~SR[1] & ~SR[0]) | (CReg[1] & ~SR[1] & SR[0]) | (CReg[2] & SR[1] & ~SR[0]) | (CReg[3] & SR[1] & SR[0]));
	// flags
	// ZERO flag
	casez (SR)
		3'd0: if (TempOpCODE==3'b010) ZERO<=~(|BSWAPBus[7:0]);
				else if (TempOpCODE==3'b110) ZERO<=~(|TempPos);
					else if (TempOpCODE[2:1]==2'b10) ZERO<=~(|CFBus[7:0]);
						else if (TempOpCODE==3'b111) ZERO<=1'b0;
							else ZERO<=~(|NEGBus[7:0]);
		3'd1: if (TempOpCODE==3'b010) ZERO<=~(|BSWAPBus[15:0]);
				else if (TempOpCODE==3'b110) ZERO<=~(|TempPos);
					else if (TempOpCODE[2:1]==2'b10) ZERO<=~(|CFBus[15:0]);
						else if (TempOpCODE==3'b111) ZERO<=1'b0;
							else ZERO<=~(|NEGBus[15:0]);
		3'd2: if (TempOpCODE==3'b010) ZERO<=~(|BSWAPBus[31:0]);
				else if (TempOpCODE==3'b110) ZERO<=~(|TempPos);
					else if (TempOpCODE[2:1]==2'b10) ZERO<=~(|CFBus[30:0]);
						else if (TempOpCODE==3'b111) ZERO<=1'b0;
							else ZERO<=~(|NEGBus[31:0]);
		3'd3: if (TempOpCODE==3'b010) ZERO<=~(|BSWAPBus[63:0]);
				else if (TempOpCODE==3'b110) ZERO<=~(|TempPos);
					else if (TempOpCODE[2:1]==2'b10) ZERO<=~(|CFBus[62:0]);
						else if (TempOpCODE==3'b111) ZERO<=1'b0;
							else ZERO<=~(|NEGBus[63:0]);
		3'b1??:	ZERO<=~(|CFBus[126:0]);
		endcase
	// Sign flag
	casez (SR)
		3'd0: if (TempOpCODE==3'b010) SIGN<=BSWAPBus[7];
				else if (TempOpCODE==3'b110) SIGN<=1'b0;
					else if (TempOpCODE[2:1]==2'b10) SIGN<=CFBus[7];
						else if (TempOpCODE==3'b111) SIGN<=1'b0;
							else SIGN<=NEGBus[7];
		3'd1: if (TempOpCODE==3'b010) SIGN<=BSWAPBus[15];
				else if (TempOpCODE==3'b110) SIGN<=1'b0;
					else if (TempOpCODE[2:1]==2'b10) SIGN<=CFBus[15];
						else if (TempOpCODE==3'b111) SIGN<=1'b0;
							else SIGN<=NEGBus[15];
		3'd2: if (TempOpCODE==3'b010) SIGN<=BSWAPBus[31];
				else if (TempOpCODE==3'b110) SIGN<=1'b0;
					else if (TempOpCODE[2:1]==2'b10) SIGN<=CFBus[31];
						else if (TempOpCODE==3'b111) SIGN<=1'b0;
							else SIGN<=NEGBus[31];
		3'd3: if (TempOpCODE==3'b010) SIGN<=BSWAPBus[63];
				else if (TempOpCODE==3'b110) SIGN<=1'b0;
					else if (TempOpCODE[2:1]==2'b10) SIGN<=CFBus[63];
						else if (TempOpCODE==3'b111) SIGN<=1'b0;
							else SIGN<=NEGBus[63];
		3'b1??: SIGN<=CFBus[127];
		endcase
	casez ({SR[2], SR[0]})
		2'b00:	OVR<=CFBus[129] | (&CFBus[30:23] & ~CFBus[130]);
		2'b01:	OVR<=CFBus[129] | (&CFBus[62:52] & ~CFBus[130]);
		2'b1?:	OVR<=CFBus[129] | (&CFBus[126:112] & ~CFBus[130]);
		endcase
	NaN<=CFBus[130];
	end
	
endmodule
