module ALU64 (
	input wire CLK, ACT, CIN,
	input wire [3:0] DSTi,
	input wire [63:0] A,B,C,D,
	input wire [1:0] SA, SB, SC, SD,
	input wire [2:0] OpCODE,
	
	output reg [63:0] R,
	output reg [15:0] COUT,
	output wire [3:0] DSTo,
	output reg [1:0] SR,
	output wire RDY,
	output reg OVR, Zero, Sign
	);
	
reg [63:0] TempR;
reg [15:0] RA, RB;
reg [1:0] TempAB, TempCD;
reg [63:0] SourceReg, MaskReg, ValueReg;
reg OpFlag;
wire [6:0] ShiftParA, ShiftParB;
wire [127:0] MaskABus, MaskBBus, ValueBus;

integer i, i0, i1;

assign ShiftParA={1'b0,A[5:0]};
assign ShiftParB={1'b0,A[5:0]}+B[6:0];
assign MaskABus={{64{1'b1}}, 64'd0};
assign MaskBBus={64'd0, {64{1'b1}}};
assign ValueBus={C, 64'd0};

assign RDY=ACT;
assign DSTo=DSTi;

always @*
begin
if (SA<SB)	TempAB=SB;
	else	TempAB=SA;
if (SC<SD)	TempCD=SD;
	else	TempCD=SC;


end

always @(posedge CLK)
	begin
	// high bit of opcode
	OpFlag<=OpCODE[2];
	
	// for FIELDCOPY
	//	A - start position
	//	B - number of bits
	//	C - bits for copy
	//	D - modified value
	// for BITCOPY
	// A - bits to copy
	// B - mask
	// C - modified value
	
	// source value
	SourceReg<=OpCODE[0] ? C : D;
	
	// mask value
	for (i0=0; i0<64; i0=i0+1) MaskReg[i0]<=OpCODE[0] ? B[i0] : MaskABus[7'd64+i0-ShiftParA] & MaskBBus[7'd64+i0-ShiftParB];
	
	// added value
	for (i1=0; i1<64; i1=i1+1) ValueReg[i1]<=OpCODE[0] ? A[i1] : ValueBus[7'd64+i1-ShiftParA];
	
	// result size
	if (TempAB<TempCD)	SR<=TempCD;
		else			SR<=TempAB;
	case (SR)
		2'b00:
			begin
			OVR<=(~RA[1]&~RB[1]&TempR[7])|(RA[1]&RB[7]&~TempR[7]);
			COUT[15]<=(RA[1]&RB[1])|((RA[1]^RB[1])&!TempR[7]);
			end
		2'b01:
			begin
			OVR<=(~RA[3]&~RB[3]&TempR[15])|(RA[3]&RB[3]&~TempR[15]);
			COUT[15]<=(RA[3]&RB[3])|((RA[3]^RB[3])&!TempR[15]);
			end
		2'b10:
			begin
			OVR<=(~RA[7]&~RB[7]&TempR[31])|(RA[7]&RB[7]&~TempR[31]);
			COUT[15]<=(RA[7]&RB[7])|((RA[7]^RB[7])&!TempR[31]);
			end
		2'b11:
			begin
			OVR<=(~RA[15]&~RB[15]&TempR[63])|(RA[15]&RB[15]&~TempR[63]);
			COUT[15]<=(RA[15]&RB[15])|((RA[15]^RB[15])&!TempR[63]);
			end
		endcase
	for (i=0; i<15; i=i+1) COUT[i]<=(RA[i]&RB[i])|((RA[i]^RB[i])&!TempR[((i+1)<<2)-1]);
	for (i=0; i<16; i=i+1)
		begin
		RA[i]<=A[((i+1)<<2)-1];
		RB[i]<=B[((i+1)<<2)-1];
		end
	casez (OpCODE[1:0])
		2'b00: TempR<=A & B;
		2'b01: TempR<=A | B;
		2'b10: TempR<=A ^ B;
		2'b11: TempR<=A + B + CIN;
		endcase
	// output result
	R<=OpFlag ? (SourceReg & ~MaskReg) | (ValueReg & MaskReg) : TempR;
	// zero flag
	Zero<=~((|TempR[7:0]) |
			(|TempR[15:8] & (|SR)) |
			(|TempR[31:16] & SR[1]) |
			(|TempR[63:32] & SR[1] & SR[0]));
	// sign flag
	case (SR)
		2'b00: Sign<=TempR[7];
		2'b01: Sign<=TempR[15];
		2'b10: Sign<=TempR[31];
		2'b11: Sign<=TempR[63];
		endcase
	end

endmodule
