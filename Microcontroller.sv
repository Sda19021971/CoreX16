module Microcontroller (
	input wire CLK, RESET, CF,
	// IO Bus
	output reg [3:0] IOWSTB,
	output reg [3:0] IORSTB,
	input wire [31:0] IOSRC [3:0],
	// memory interface
	input wire NEXT,
	output wire ACT, CMD,
	output wire [1:0] SIZE,
	output wire [44:0] ADDRESS,
	output wire [31:0] DTo,
	input wire DRDY,
	input wire [31:0] DTi,
	// external requests
	input wire [7:0] REQ,
	input wire [7:0] IMASK,
	// external base sources
	input wire [39:0] BASE [1:0]
);

parameter Code = "code.mif";

integer i0, i1;

reg [10:0] PCntReg;
logic [10:0] ABus, VectorBus;
logic [18:0] ROMBus;
reg [18:0] ROMReg, InstReg;
reg ROMValidFlag, ValidFlag, IntModeFlag, CarryFlag, InstBit, DRDYFlag, LDFlag, JmpFlag, ZSTBFlag, IntFlag;
reg ZeroFlag, SignFlag, AGBFlag, AEBFlag, ALBFlag, DLDFlag, INFlag, OpReloadFlag, AddReloadFlag, DDRDYFlag, OutPortFlag;
logic CCWire, EnableFlag, ResetValidFlag;
reg [10:0] StackReg [0:3];

// general purpose registers
reg [31:0] GPR [7:0];
reg [7:0] GPRWriteFlag;

// flags
reg DataTestBit, ShiftOutBit;

// immediate data registers
reg [31:0] ImmediateReg;
reg [31:0] ImmediateSource;

// interrupt request register
reg [7:0] IRQReg;

// operand registers
reg [31:0] OpA, OpB;
logic [32:0] ResultBus;

// memory interface resources
reg ACTReg, CMDReg;
reg [44:0] AddressReg;
reg [1:0] SizeReg;
logic [39:0] BaseBus;
reg [39:0] BSR [0:1];
reg [31:0] DoReg;

reg [31:0] IOValue, ResReg;

// microcode ROM
altsyncram	McROM (.clock0 (CLK),.address_a (ABus),.q_a (ROMBus), .aclr0 (1'b0), .aclr1 (1'b0), .address_b (1'b1),
				.addressstall_a (1'b0), .addressstall_b (1'b0), .byteena_a (1'b1), .byteena_b (1'b1), .clock1 (1'b1),
				.clocken0 (EnableFlag), .clocken1 (1'b1), .clocken2 (1'b1), .clocken3 (1'b1), .data_a ({19{1'b1}}), .data_b (1'b1),
				.eccstatus (), .q_b (), .rden_a (1'b1), .rden_b (1'b1), .wren_a (1'b0), .wren_b (1'b0));
	defparam
		McROM.clock_enable_input_a = "NORMAL",
		McROM.clock_enable_output_a = "BYPASS",
		McROM.init_file = Code,
		McROM.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
		McROM.lpm_type = "altsyncram",
		McROM.numwords_a = 2048,
		McROM.operation_mode = "ROM",
		McROM.outdata_aclr_a = "NONE",
		McROM.outdata_reg_a = "UNREGISTERED",
		McROM.widthad_a = 11,
		McROM.width_a = 19,
		McROM.width_byteena_a = 1;

assign ACT=ACTReg, CMD=CMDReg, SIZE=SizeReg, ADDRESS=AddressReg, DTo=DoReg;

/*
===================================================================================================
		Logic part
===================================================================================================
*/
always_comb
begin

// generate result bus
case ({InstReg[4:2], InstBit})
	4'b0000:	ResultBus={1'b0,OpA}+{1'b0,OpB}+CarryFlag;
	4'b0001:	ResultBus={1'b0,OpA[30:0],1'b0};
	4'b0010:	ResultBus={1'b0,OpA}-{1'b0,OpB}-CarryFlag;
	4'b0011:	ResultBus={2'b00,OpA[31:1]};
	4'b0100:	ResultBus={1'b0,OpA & OpB};
	4'b0101:	ResultBus={OpA[31:0],CarryFlag};
	4'b0110:	ResultBus={1'b0,OpA | OpB};
	4'b0111:	ResultBus={OpA[0],CarryFlag,OpA[31:1]};
	4'b1000:	ResultBus={1'b0,OpA ^ OpB};
	4'b1001:	ResultBus=InstReg[8] ? {1'b0,OpA[23:0],8'd0} : {1'b0,OpA[29:0],2'b00};
	4'b1010:	ResultBus=InstReg[1] ? {1'b0,OpB} : {1'b0,ImmediateReg};
	4'b1011:	ResultBus=InstReg[8] ? {1'b0, 8'd0, OpA[31:8]} : {3'b000,OpA[31:2]};
	4'b1100:	ResultBus={1'b0,OpA};
	4'b1101:	ResultBus={4'b0000,OpA[23:0],5'b00000};
	4'b1110:	ResultBus={1'b0,IOValue};
	4'b1111:	ResultBus={1'b0,OpA[31],OpA[31:1]};
	endcase

// generate interrupt vector
casez (IRQReg & IMASK)
	8'b???????1:	VectorBus=11'h7F0;
	8'b??????10:	VectorBus=11'h7F2;
	8'b?????100:	VectorBus=11'h7F4;
	8'b????1000:	VectorBus=11'h7F6;
	8'b???10000:	VectorBus=11'h7F8;
	8'b??100000:	VectorBus=11'h7FA;
	8'b?1000000:	VectorBus=11'h7FC;
	8'b10000000:	VectorBus=11'h7FE;
	default:		VectorBus=11'h7F0;
	endcase

// branch condition
case (InstReg[7:5])
	3'd0:	CCWire=CarryFlag;
	3'd1:	CCWire=ZeroFlag;
	3'd2:	CCWire=SignFlag;
	3'd3:	CCWire=AGBFlag;
	3'd4:	CCWire=AEBFlag;
	3'd5:	CCWire=ALBFlag;
	3'd6:	CCWire=~ZeroFlag;
	3'd7:	CCWire=CF;
	endcase

// microinstruction address
if (IntModeFlag) ABus=VectorBus;
	else if (ValidFlag & (InstReg[4:0]==5'd9)) ABus=StackReg[0];
		else if (ValidFlag & ((InstReg[4:0]==5'd5) | ((InstReg[4:0]==5'd1) & CCWire))) ABus=InstReg[18:8];
			else ABus=PCntReg;

// enable microinstruction flag
EnableFlag=(~ACTReg | ((InstReg[4:0]!=5'h1A) & (InstReg[4:0]!=5'h18))) & ((InstReg[4:0]!=5'h1E) | DDRDYFlag | ~ValidFlag) & 
			(~IntModeFlag | (|(IRQReg & IMASK))) & ~JmpFlag & ~OpReloadFlag & ~AddReloadFlag & ~OutPortFlag &
			(NEXT | ~ACTReg | CMDReg);

// generate base address
case (InstReg[9:8])
	2'b00:	BaseBus=BASE[0];
	2'b01:	BaseBus=BASE[1];
	2'b10:	BaseBus=BSR[0];
	2'b11:	BaseBus=BSR[1];
	endcase

// tested data bit
DataTestBit=OpA[InstReg[17:13]];

// shift data bit
case (InstReg[4:2])
	3'd0:	ShiftOutBit=OpA[31];
	3'd1:	ShiftOutBit=OpA[0];
	3'd2:	ShiftOutBit=OpA[31];
	3'd3:	ShiftOutBit=OpA[0];
	3'd4:	ShiftOutBit=1'b0;
	3'd5:	ShiftOutBit=1'b0;
	3'd6:	ShiftOutBit=1'b0;
	3'd7:	ShiftOutBit=OpA[0];
	endcase


ResetValidFlag=(~ValidFlag | ((InstReg[4:0]!=5'd5) & (InstReg[4:0]!=5'd9) & ((InstReg[4:0]!=5'd1) | ~CCWire)) & (InstReg[4:0]!=5'h11));

end


/*
===================================================================================================
		Synchronous part
===================================================================================================
*/
always_ff @(negedge RESET or posedge CLK)
begin

if (!RESET) begin
				IRQReg<=0;
				ROMValidFlag<=0;
				ValidFlag<=0;
				PCntReg<=0;
				JmpFlag<=0;
				IntModeFlag<=0;
				LDFlag<=0;
				INFlag<=0;
				ACTReg<=0;
				IOWSTB<=0;
				IORSTB<=0;
				GPR[0]<=0;
				GPR[1]<=0;
				GPR[2]<=0;
				GPR[3]<=0;
				GPR[4]<=0;
				GPR[5]<=0;
				GPR[6]<=0;
				GPR[7]<=0;
				GPRWriteFlag<=0;
				InstReg<=19'd13;
				ROMReg<=19'd13;
				OutPortFlag<=0;
				AddReloadFlag<=0;
				OpReloadFlag<=0;
				DDRDYFlag<=0;
				IntFlag<=0;
				end
else begin
// interrupt reception
IRQReg[0]<=(IRQReg[0] | REQ[0]) & ((PCntReg != 11'h7F1) | ~IntFlag);
IRQReg[1]<=(IRQReg[1] | REQ[1]) & ((PCntReg != 11'h7F3) | ~IntFlag);
IRQReg[2]<=(IRQReg[2] | REQ[2]) & ((PCntReg != 11'h7F5) | ~IntFlag);
IRQReg[3]<=(IRQReg[3] | REQ[3]) & ((PCntReg != 11'h7F7) | ~IntFlag);
IRQReg[4]<=(IRQReg[4] | REQ[4]) & ((PCntReg != 11'h7F9) | ~IntFlag);
IRQReg[5]<=(IRQReg[5] | REQ[5]) & ((PCntReg != 11'h7FB) | ~IntFlag);
IRQReg[6]<=(IRQReg[6] | REQ[6]) & ((PCntReg != 11'h7FD) | ~IntFlag);
IRQReg[7]<=(IRQReg[7] | REQ[7]) & ((PCntReg != 11'h7FF) | ~IntFlag);

IntFlag<=(InstReg==19'h00011);



if (EnableFlag)
	begin
	// generate operand A value
	OpA<=GPR[ROMReg[7:5]];
	// generate operand B value
	OpB<=(ROMReg[1] | ROMReg[0]) ? GPR[ROMReg[12:10]] : ImmediateSource;

	// immediade datas
	case (ROMReg[9:8])
		2'b00:	ImmediateReg<={GPR[ROMReg[7:5]][31:8], ROMReg[17:10]};
		2'b01:	ImmediateReg<={GPR[ROMReg[7:5]][31:16], ROMReg[17:10], GPR[ROMReg[7:5]][7:0]};
		2'b10:	ImmediateReg<={GPR[ROMReg[7:5]][31:24], ROMReg[17:10], GPR[ROMReg[7:5]][15:0]};
		2'b11:	ImmediateReg<={ROMReg[17:10], GPR[ROMReg[7:5]][23:0]};
		endcase
	// immediade data
	case (ROMBus[9:8])
		2'b00: ImmediateSource<={{24{ROMBus[4:0]==5'd8}}, ROMBus[17:10]};
		2'b01: ImmediateSource<={{16{ROMBus[4:0]==5'd8}}, ROMBus[17:10], {8{ROMBus[4:0]==5'd8}}};
		2'b10: ImmediateSource<={{8{ROMBus[4:0]==5'd8}}, ROMBus[17:10], {16{ROMBus[4:0]==5'd8}}};
		2'b11: ImmediateSource<={ROMBus[17:10], {24{ROMBus[4:0]==5'd8}}};
		endcase

	// MCROM connection
	ROMReg<=ROMBus;
	// instruction register
	InstReg<=ROMReg;
	InstBit<=ROMReg[1] & ROMReg[0];
	// validation flags
	ROMValidFlag<=ResetValidFlag;
	ValidFlag<=ROMValidFlag & ResetValidFlag;
	// carry flag
	end

// microinstruction address register
if (EnableFlag) PCntReg<=ABus+11'd1;

ResReg<=ResultBus[31:0];

if (EnableFlag & ValidFlag)
	begin
	if (InstReg[4:0]==5'h15) CarryFlag<=1'b0;
		else if (InstReg[4:0]==5'h19) CarryFlag<=1'b1;
			else if (InstReg[4:0]==5'h1D) CarryFlag<=DataTestBit;
				else if (InstReg[1:0]==2'b11) CarryFlag<=ShiftOutBit;
					else if (~InstReg[4] & ~InstReg[3] & ~InstReg[0]) CarryFlag<=ResultBus[32];
	end

// flags
if (EnableFlag & ValidFlag & ~InstReg[0]) SignFlag<=ResultBus[31];
if ((InstReg[4:0]==5'h1D) & EnableFlag & ValidFlag)
		begin
		// A great than B Flag
		AGBFlag<=(OpA>OpB);
		// A equal to B
		AEBFlag<=(OpA==OpB);
		// A less than B
		ALBFlag<=(OpA<OpB);
		end

// flag for zero strobe
ZSTBFlag<=EnableFlag & ValidFlag & ~InstReg[0];
if (ZSTBFlag) ZeroFlag<=~(|ResReg);

// wait flag. Setup to 1 if next command use result of prev. command as source
OpReloadFlag<=~OpReloadFlag & EnableFlag & ResetValidFlag & ROMValidFlag & 
			(((ROMReg[7:5]==ROMBus[7:5]) & (ROMReg[4:0]!=5'h1E) & (~ROMReg[4] | ~ROMReg[3] | ROMReg[2] | ROMReg[0]) & (ROMReg[1] | ~ROMReg[0]) & (ROMBus[1] | ~ROMBus[0] | (&ROMBus[4:2]))) |
				((ROMReg[7:5]==ROMBus[12:10]) & (ROMReg[1] | ~ROMReg[0]) & ((ROMBus[1] & ~ROMBus[0]) | (ROMBus[4:0]==5'h1D))));

// additional reload flag
AddReloadFlag<=OpReloadFlag & (InstReg[4:0]==5'h1C);

// JMP flag
JmpFlag<=~JmpFlag & ((ROMReg[4:0]==5'h01) | (ROMReg[4:0]==5'h1C) | (ROMReg[4:0]==5'h18)) & ROMValidFlag & EnableFlag & ResetValidFlag;

// data from input ports and from memory
// IO Value selection
if (EnableFlag | LDFlag | INFlag)
	begin
	for (i1=0; i1<32; i1=i1+1)
		begin
		if (i1<8) IOValue[i1]<=(InstReg[10] & ~LDFlag & BSR[0][i1]) | (InstReg[11] & ~LDFlag & BSR[0][i1+32]) | 
								(InstReg[12] & ~LDFlag & BSR[1][i1]) | (InstReg[13] & ~LDFlag & BSR[1][i1+32]) |
								(InstReg[14] & ~LDFlag & IOSRC[0][i1]) | (InstReg[15] & ~LDFlag & IOSRC[1][i1]) |
								(InstReg[16] & ~LDFlag & IOSRC[2][i1]) | (InstReg[17] & ~LDFlag & IOSRC[3][i1]) |
								(InstReg[18] & ~LDFlag & IRQReg[i1]) | (LDFlag & DTi[i1]);
				else IOValue[i1]<=(InstReg[10] & ~LDFlag & BSR[0][i1]) | (InstReg[12] & ~LDFlag & BSR[1][i1]) |
								(InstReg[14] & ~LDFlag & IOSRC[0][i1]) | (InstReg[15] & ~LDFlag & IOSRC[1][i1]) |
								(InstReg[16] & ~LDFlag & IOSRC[2][i1]) | (InstReg[17] & ~LDFlag & IOSRC[3][i1]) |
								(LDFlag & DTi[i1]);
		end	
	end

// setting interrupt mode
IntModeFlag<=(IntModeFlag & (~(|(IRQReg & IMASK)))) | (ROMValidFlag & EnableFlag & (ROMReg[4:0]==5'h11) & ResetValidFlag);
// data load from memory flag
DRDYFlag<=DRDY;
DDRDYFlag<=DRDYFlag;
// data load mode flag
LDFlag<=(LDFlag | ((ROMReg[4:0]==5'b11110) & EnableFlag & ROMValidFlag & ResetValidFlag)) & ~DRDY;
if (~ACTReg) DLDFlag<=LDFlag;
// data input mode flag
INFlag<=(ROMReg[4:0]==5'b11100) & EnableFlag & ROMValidFlag & ResetValidFlag;
// output port flag
OutPortFlag<=(InstReg[4:0]==5'b11000) & ValidFlag & EnableFlag;

// stack register file
if (~InstReg[4] & (InstReg[3] ^ InstReg[2]) & ~InstReg[1] & InstReg[0] & EnableFlag & ValidFlag) StackReg[0]<=InstReg[2] ? (PCntReg-11'd2) : StackReg[1];
if (~InstReg[4] & (InstReg[3] ^ InstReg[2]) & ~InstReg[1] & InstReg[0] & EnableFlag & ValidFlag) StackReg[1]<=InstReg[2] ? StackReg[0] : StackReg[2];
if (~InstReg[4] & (InstReg[3] ^ InstReg[2]) & ~InstReg[1] & InstReg[0] & EnableFlag & ValidFlag) StackReg[2]<=InstReg[2] ? StackReg[1] : StackReg[3];
if (~InstReg[4] & (InstReg[3] ^ InstReg[2]) & ~InstReg[1] & InstReg[0] & EnableFlag & ValidFlag) StackReg[3]<=InstReg[2] ? StackReg[2] : 11'd0;

// Base Address registers
if (InstReg[10] & (InstReg[4:0]==5'b11000) & ~EnableFlag & ValidFlag) BSR[0][31:0]<=OpA;
if (InstReg[11] & (InstReg[4:0]==5'b11000) & ~EnableFlag & ValidFlag) BSR[0][39:32]<=OpA[7:0];
if (InstReg[12] & (InstReg[4:0]==5'b11000) & ~EnableFlag & ValidFlag) BSR[1][31:0]<=OpA;
if (InstReg[13] & (InstReg[4:0]==5'b11000) & ~EnableFlag & ValidFlag) BSR[1][39:32]<=OpA[7:0];

// store data into GPR
for (i0=0; i0<8; i0=i0+1)
	begin
	if (EnableFlag) GPRWriteFlag[i0]<=(ROMReg[1] | ~ROMReg[0]) & (ROMReg[7:5]==i0) & ROMValidFlag;
	if (ValidFlag & GPRWriteFlag[i0]) GPR[i0][7:0]<=ResultBus[7:0];
	if (ValidFlag & (GPRWriteFlag[i0]) & ((InstReg[4:0]!=5'h1e) | InstReg[14] | InstReg[13])) GPR[i0][15:8]<=ResultBus[15:8];
	if (ValidFlag & (GPRWriteFlag[i0]) & ((InstReg[4:0]!=5'h1e) | InstReg[14])) GPR[i0][31:16]<=ResultBus[31:16];
	end

// memory and IO interface
ACTReg<=(ACTReg | (LDFlag & ~DLDFlag) | (InstReg[4] & InstReg[3] & ~InstReg[2] & InstReg[1] & ~InstReg[0] & ValidFlag & EnableFlag)) & (~NEXT | ~ACTReg);

if ((EnableFlag & ValidFlag & (InstReg[4:0]==5'h1A)) | (LDFlag & ~DLDFlag & (NEXT | ~ACTReg | CMDReg)) | (EnableFlag & ValidFlag & (InstReg[4:0]==5'h18)))
	begin
	// command
	CMDReg<=&InstReg[4:2];
	// address to memory
	AddressReg<={BaseBus,InstReg[17:15],2'd0}+{13'd0,OpB};
	// operand size
	SizeReg<=InstReg[14:13];
	// data to memory or IO
	DoReg<=OpA;
	end

// generate IO stb signals
IOWSTB[0]<=InstReg[14] & (InstReg[4:0]==5'b11000) & ValidFlag & EnableFlag;
IOWSTB[1]<=InstReg[15] & (InstReg[4:0]==5'b11000) & ValidFlag & EnableFlag;
IOWSTB[2]<=InstReg[16] & (InstReg[4:0]==5'b11000) & ValidFlag & EnableFlag;
IOWSTB[3]<=InstReg[17] & (InstReg[4:0]==5'b11000) & ValidFlag & EnableFlag;
// IO read strobe
IORSTB[0]<=ROMReg[14] & (ROMReg[4:0]==5'b11100) & ResetValidFlag & ROMValidFlag & EnableFlag;
IORSTB[1]<=ROMReg[15] & (ROMReg[4:0]==5'b11100) & ResetValidFlag & ROMValidFlag & EnableFlag;
IORSTB[2]<=ROMReg[16] & (ROMReg[4:0]==5'b11100) & ResetValidFlag & ROMValidFlag & EnableFlag;
IORSTB[3]<=ROMReg[17] & (ROMReg[4:0]==5'b11100) & ResetValidFlag & ROMValidFlag & EnableFlag;
end
end

endmodule
