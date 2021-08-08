module Shifter64 (
	input wire CLK, ACT, CIN,
	input wire [63:0] A,
	input wire [5:0] B,
	input wire [3:0] DSTi,
	input wire [1:0] SA,
	input wire [2:0] OPR,
	
	output reg [63:0] R,
	output reg [3:0] DSTo,
	output reg RDY, OVR, ZERO, COUT, SIGN,
	output reg [1:0] SR
	);
	
reg [63:0] TempR [4:0];
reg [63:0] IntR;
reg [2:0] TempOPR;
reg [1:0] TempSA;
reg TempCOUT, TempMSB, IntCOUT, IntMSB;
wire [5:0] ShiftPar, CSMask;
wire [127:0] LSLBus, LSRBus, ASRBus;

integer i;

// reduce shift parameter by operand size
assign ShiftPar[2:0]=B[2:0], ShiftPar[3]=B[3] & (|SA), ShiftPar[4]=B[4] & SA[1], ShiftPar[5]=B[5] & SA[1] & SA[0];
assign CSMask[2:0]=3'b111, CSMask[3]=|SA, CSMask[4]=SA[1], CSMask[5]=&SA;

assign LSLBus={A,{64{CIN}}};

assign LSRBus[7:0]=A[7:0], LSRBus[15:8]=(SA==2'b00) ? {8{CIN}} : A[15:8], LSRBus[31:16]=~SA[1] ? {16{CIN}} : A[31:16];
assign LSRBus[63:32]=(SA==2'b11) ? A[63:32] : {32{CIN}}, LSRBus[127:64]={64{CIN}};

assign ASRBus[7:0]=A[7:0], ASRBus[15:8]=(SA==2'b00) ? {8{A[7]}} : A[15:8], ASRBus[31:16]=~SA[1] ? {16{A[15]}} : A[31:16];
assign ASRBus[63:32]=(SA==2'b11) ? A[63:32] : {32{A[31]}}, ASRBus[127:64]={64{A[63]}};

always @(posedge CLK)
	begin
	TempMSB<=((SA==2'b00) & A[7]) | ((SA==2'b01) & A[15]) | ((SA==2'b10) & A[31]) | ((SA==2'b11) & A[63]);
	TempSA<=SA;
	TempOPR<=OPR;

	TempR[0][0]<=(|ShiftPar) ? CIN : LSLBus[64];
	TempR[1][0]<=A[ShiftPar];
	TempR[2][0]<=(A[8-ShiftPar] & (SA==2'b00)) | (A[16-ShiftPar] & (SA==2'b01)) | (A[32-ShiftPar] & (SA==2'b10)) | (A[64-ShiftPar] & (SA==2'b11));
	TempR[3][0]<=A[ShiftPar];
	TempR[4][0]<=A[ShiftPar];
	
	for (i=1; i<63; i=i+1)
		begin
		TempR[0][i]<=LSLBus[7'd64+i-ShiftPar];
		TempR[1][i]<=LSRBus[i+ShiftPar];
		TempR[2][i]<=A[(i-ShiftPar) & CSMask];
		TempR[3][i]<=A[(i+ShiftPar) & CSMask];
		TempR[4][i]<=ASRBus[i+ShiftPar];
		end
	TempR[0][63]<=LSLBus[127-ShiftPar];
	TempR[1][63]<=(|ShiftPar) ? CIN : LSRBus[64];
	TempR[2][63]<=A[(63-ShiftPar) & 6'b111111];
	TempR[3][63]<=A[(63+ShiftPar) & 6'b111111];
	TempR[4][63]<=A[63];

	for (i=0; i<64; i=i+1)
		IntR[i]<=(TempR[0][i] & (TempOPR==3'b000)) |
				(TempR[1][i] & (TempOPR==3'b001)) |
				(TempR[2][i] & (TempOPR==3'b010)) |
				(TempR[3][i] & (TempOPR==3'b011)) |
				(TempR[4][i] & (TempOPR==3'b101));

	R<=IntR;		
	ZERO<=~(|IntR[7:0] | (|IntR[15:8] & (|SR)) | (|IntR[31:16] & SR[1]) | (|IntR[63:32] & SR[1] & SR[0]));
	SIGN<=(IntR[7] & ~SR[1] & ~SR[0])|(IntR[15] & ~SR[1] & SR[0])|(IntR[31] & SR[1] & SR[0])|(IntR[63] & SR[1] & SR[0]);
	// COUT forming
	TempCOUT<=OPR[0] ? A[ShiftPar-1] : (A[8-ShiftPar] & (SA==2'b00)) | (A[16-ShiftPar] & (SA==2'b01)) | (A[32-ShiftPar] & (SA==2'b10)) | (A[64-ShiftPar] & (SA==2'b11));
	IntCOUT<=TempCOUT;
	// overflow forming
	IntMSB<=TempMSB;
	OVR<=IntMSB ^ (((SR==2'b00) & IntR[7]) | ((SR==2'b01) & IntR[15]) | ((SR==2'b10) & IntR[31]) | ((SR==2'b11) & IntR[63]));

	COUT<=IntCOUT;
	RDY<=ACT;
	DSTo<=DSTi;
	SR<=TempSA;
	end

endmodule
