module fft(
	input wire CLK, RESET,
	input wire [7:0] CPU,
	input wire [39:0] DTBASE,
	input wire [23:0] DTLIMIT,
	input wire [1:0] CPL,
	input wire [15:0] TASKID,
	input wire [23:0] CPSR,
	// command interface
	output wire NEXT,
	input wire ACT,
	input wire [31:0] DataObject, ControlObject,
	input wire [33:0] DataOffset, ControlOffset,
	input wire [4:0] PAR,
	input wire [18:0] INDEX,
	// memory interface
	input wire MemNEXT, NetNEXT,
	output reg MemACT, NetACT, MemCMD,
	output reg [41:0] MemADDR,
	output reg [33:0] MemOFFSET,
	output reg [31:0] MemSEL,
	output reg [63:0] MemDO,
	output reg [7:0] MemTO,
	input wire MemDRDY,
	input wire [63:0] MemDI,
	input wire [7:0] MemTI,
	// error report interface
	output wire ESTB,
	output wire [28:0] ECD,
	output wire [23:0] ECPSR
	);

integer i0, i1, i2;

reg [31:0] ADR [0:1];	
reg [155:0] DTR [0:1];
reg BusyFlag, IntACTReg, IntCMDReg, CheckACT, IntSelReg, CheckSEL, CheckCMD, CheckNetwork, DTRLoadedFlag;
reg DescriptorLoadState, RetryTransactionFlag, ValidDescriptor, DRDYFlag, InvalidType, InvalidCPL, InvalidTaskID;
reg ErrorFlag;
reg [4:0] StageReg, ParReg;
reg [18:0] GroupReg, GroupCnt, OffsetReg, GroupCntr, ScaleOffset, IndexReg;
logic [4:0] PARBus;
logic [18:0] LengthBus, GroupRegBus, GroupCntBus, ScaleOffsetBus, IndexMask;
logic [19:0] ScaleLengthBus;
reg [3:0] FFTMach;
parameter RS=4'd0, WS=4'd1, SPS=4'd2, RSS=4'd3, RYS=4'd4, RXS=4'd5, PPS=4'd6, ES=4'd7, FES=4'd8;
reg [1:0] WRMach;
parameter WMRS=2'd0, WMWS=2'd1, WMWES=2'd2, WMWOS=2'd3;

// cache buffer
logic FCacheINEXT;

logic TWFIFOEmpty, YFIFOEmpty, XFIFOEmpty, XDFIFOEmpty, AFIFOEmpty, YrWrRDY, FifoReadB, CheckNext, IntNext, NrAdderRDY, NXAdderRDY, RFIFOEmpty;
logic OutNEXT, LoadNewDSC, LoadLowerDSC, LoadUpperDSC, AccessError, EndFlag, FCacheIDRDY, FCacheEACT, FCacheECMD, FCacheESEL;
logic [1:0] FCacheITAGo, FCacheETAGo;
logic [63:0] TWBus, YBus, XBus, XDBus, NBus, NXBus, NYBus, FCacheIDATo, FCacheEDATo;
logic [3:0] TWusedw, Yusedw, Xusedw, XDusedw, Ausedw, Rusedw;
logic [127:0] RBus;
logic [31:0] YrWrBus, YiWiBus, YrWiBus, YiWrBus, FCacheESELECTOR;
reg [63:0] IntDataReg, CheckData;
reg [33:0] IntOffsetReg, DataOffsetReg, CheckOffset, ControlOffsetReg;
logic [33:0] ReadEvenOffset, ReadOddOffset, WriteEvenOffset, WriteOddOffset, FCacheEOFFSET;
logic [39:0] ABus;
reg [1:0] InTagReg, CheckTag, CPLReg, TAGiReg;
reg [31:0] CheckSelector, CheckLL, CheckUL;
reg [39:0] CheckBase;
reg [23:0] CheckLowerSel, CheckUpperSel, DLSelector, InvalidSelector, CPSRReg;
reg [3:0] CheckAR;
reg [2:0] DTRWriteFlag;
reg [15:0] TASKIDReg;
reg [3:0] DLMachine;
parameter DLWS=4'd0, CSS=4'd1, LBS=4'd2, LLSS=4'd3, LSLS=4'd4, RWS=4'd5, ZSS=4'd6, INVSS=4'd7, INVOBS=4'd8, AERSS=4'd9, STS=4'd10;

// FIFO for twiddle factors
sc_fifo TWFIFO(.data(FCacheIDATo), .wrreq(FCacheIDRDY & (FCacheITAGo==2'h0)), .rdreq(FifoReadB), .clock(CLK), .sclr(~RESET | ErrorFlag),
				.q(TWBus), .empty(TWFIFOEmpty), .usedw(TWusedw));
defparam TWFIFO.LPM_WIDTH=64, TWFIFO.LPM_NUMWORDS=16, TWFIFO.LPM_WIDTHU=4;

// Fifo for Y operand
sc_fifo YFIFO(.data(FCacheIDATo), .wrreq(FCacheIDRDY & (FCacheITAGo==2'h2)), .rdreq(FifoReadB), .clock(CLK), .sclr(~RESET | ErrorFlag),
				.q(YBus), .empty(YFIFOEmpty), .usedw(Yusedw));
defparam YFIFO.LPM_WIDTH=64, YFIFO.LPM_NUMWORDS=16, YFIFO.LPM_WIDTHU=4;

// Fifo for X operand
sc_fifo XFIFO(.data(FCacheIDATo), .wrreq(FCacheIDRDY & (FCacheITAGo==2'h1)), .rdreq(FifoReadB), .clock(CLK), .sclr(~RESET | ErrorFlag),
				.q(XBus), .empty(XFIFOEmpty), .usedw(Xusedw));
defparam XFIFO.LPM_WIDTH=64, XFIFO.LPM_NUMWORDS=16, XFIFO.LPM_WIDTHU=4;

// Fifo for X operand delayed
sc_fifo XDFIFO(.data(XBus), .wrreq(FifoReadB), .rdreq(NrAdderRDY), .clock(CLK), .sclr(~RESET | ErrorFlag),
				.q(XDBus), .empty(XDFIFOEmpty), .usedw(XDusedw));
defparam XDFIFO.LPM_WIDTH=64, XDFIFO.LPM_NUMWORDS=16, XDFIFO.LPM_WIDTHU=4;

// Fifo for write address
sc_fifo AFIFO(.data({EndFlag,{1'b0,OffsetReg}+{1'd0,GroupCntBus}+20'd1, OffsetReg}), .wrreq((FFTMach==RSS) & IntNext & (WRMach==WMWS)), 
				.rdreq((WRMach==WMWOS) & IntNext), .clock(CLK), .sclr(~RESET | ErrorFlag),
				.q(ABus), .empty(AFIFOEmpty), .usedw(Ausedw));
defparam AFIFO.LPM_WIDTH=40, AFIFO.LPM_NUMWORDS=16, AFIFO.LPM_WIDTHU=4;

// Yr*Wr multiplier
FPMUL32 YrWr(.CLK(CLK), .RESET(RESET), .ACT(FifoReadB), .TAGi(1'b0), .A(TWBus[31:0]), .B(YBus[31:0]), .RDY(YrWrRDY), .R(YrWrBus));
// Yi*Wi multiplier
FPMUL32 YiWi(.CLK(CLK), .RESET(RESET), .ACT(FifoReadB), .TAGi(1'b0), .A(TWBus[63:32]), .B(YBus[63:32]), .R(YiWiBus));
// Yr*Wi multiplier
FPMUL32 YrWi(.CLK(CLK), .RESET(RESET), .ACT(FifoReadB), .TAGi(1'b0), .A(TWBus[63:32]), .B(YBus[31:0]), .R(YrWiBus));
// Yi*Wr multiplier
FPMUL32 YiWr(.CLK(CLK), .RESET(RESET), .ACT(FifoReadB), .TAGi(1'b0), .A(TWBus[31:0]), .B(YBus[63:32]), .R(YiWrBus));
// Yr*Wr-Yi*Wi adder
FPADD32 NrAdder(.CLK(CLK), .RESET(~RESET), .ACT(YrWrRDY), .TAGi(1'b0), .A(YrWrBus), .B({~YiWiBus[31],YiWiBus[30:0]}), .RDY(NrAdderRDY), .R(NBus[31:0]));
// Yr*Wi+Yi*Wr adder
FPADD32 NiAdder(.CLK(CLK), .RESET(~RESET), .ACT(YrWrRDY), .TAGi(1'b0), .A(YrWiBus), .B(YiWrBus), .R(NBus[63:32]));
// Xr+Nr adder
FPADD32 NXrAdder(.CLK(CLK), .RESET(~RESET), .ACT(NrAdderRDY), .TAGi(1'b0), .A(XDBus[31:0]), .B(NBus[31:0]), .RDY(NXAdderRDY), .R(NXBus[31:0]));
// Xi+Ni adder
FPADD32 NXiAdder(.CLK(CLK), .RESET(~RESET), .ACT(NrAdderRDY), .TAGi(1'b0), .A(XDBus[63:32]), .B(NBus[63:32]), .R(NXBus[63:32]));
// Xr-Nr adder
FPADD32 NYrAdder(.CLK(CLK), .RESET(~RESET), .ACT(NrAdderRDY), .TAGi(1'b0), .A(XDBus[31:0]), .B({~NBus[31],NBus[30:0]}), .R(NYBus[31:0]));
// Xi-Ni adder
FPADD32 NYiAdder(.CLK(CLK), .RESET(~RESET), .ACT(NrAdderRDY), .TAGi(1'b0), .A(XDBus[63:32]), .B({~NBus[63],NBus[62:32]}), .R(NYBus[63:32]));

// Fifo fo NX and NY results
sc_fifo RFIFO(.data({NYBus, NXBus}), .wrreq(NXAdderRDY), .rdreq((WRMach==WMWOS) & IntNext), .clock(CLK), .sclr(~RESET),
				.q(RBus), .empty(RFIFOEmpty), .usedw(Rusedw));
defparam RFIFO.LPM_WIDTH=128, RFIFO.LPM_NUMWORDS=16, RFIFO.LPM_WIDTHU=4;

// data and twiddle cache
FFTCache FCache(.RESET(RESET), .CLK(CLK),
				// internal ISI
				.INEXT(FCacheINEXT), .IACT(IntACTReg), .ICMD(IntCMDReg), .ISEL(IntSelReg), .IOFFSET(IntOffsetReg), .ISELECTOR(ADR[IntSelReg][31:0]),
				.IDATi(IntDataReg), .ITAGi(InTagReg), .IDRDY(FCacheIDRDY), .IDATo(FCacheIDATo), .ITAGo(FCacheITAGo),
				// external ISI
				.ENEXT(CheckNext), .EACT(FCacheEACT), .ECMD(FCacheECMD), .ESEL(FCacheESEL), .EOFFSET(FCacheEOFFSET), .ESELECTOR(FCacheESELECTOR),
				.EDATo(FCacheEDATo), .ETAGo(FCacheETAGo), .EDRDY(DRDYFlag), .EDATi(MemDI), .ETAGi(TAGiReg));

assign NEXT=~BusyFlag;

// error reporting
assign ESTB=(DLMachine==ZSS) | (DLMachine==INVSS) | (DLMachine==AERSS) | (DLMachine==INVOBS);
assign ECD[23:0]=(DLMachine==INVOBS) ? InvalidSelector : CheckSelector[23:0];
assign ECD[24]=LoadLowerDSC | LoadUpperDSC;
assign ECD[25]=(DLMachine==AERSS);
assign ECD[26]=InvalidCPL;
assign ECD[27]=InvalidTaskID;
assign ECD[28]=InvalidType;
assign ECPSR=CPSRReg;


always_comb
begin

// OUTNEXT forming
OutNEXT=(~MemACT | MemNEXT) & (~NetACT | NetNEXT);

// CheckNext forming
CheckNext=~CheckACT | (OutNEXT & ((CheckAR[3] & (CheckOffset[33:2]>=CheckLL) & (CheckOffset[33:2]<CheckUL) & (CheckCMD | CheckAR[1]) & 
													(~CheckCMD | CheckAR[0])) | (CheckAR[3] & CheckAR[2]) | CheckNetwork)) | (DLMachine==STS);

// connecting pins
IntNext=~IntACTReg | FCacheINEXT;

// read YFIFO and WFIFO
FifoReadB=~TWFIFOEmpty & ~YFIFOEmpty & ~XFIFOEmpty;

// FFT length parameter
case (ParReg)
	5'd0: 	begin
			LengthBus=19'd0;
			ScaleLengthBus=20'd0;
			end
	5'd1: 	begin
			LengthBus=19'd1;
			ScaleLengthBus=20'd2;
			end
	5'd2: 	begin
			LengthBus=19'd3;
			ScaleLengthBus=20'd4;
			end
	5'd3: 	begin
			LengthBus=19'd7;
			ScaleLengthBus=20'd8;
			end
	5'd4: 	begin
			LengthBus=19'd15;
			ScaleLengthBus=20'd16;
			end
	5'd5: 	begin
			LengthBus=19'd31;
			ScaleLengthBus=20'd32;
			end
	5'd6: 	begin
			LengthBus=19'd63;
			ScaleLengthBus=20'd64;
			end
	5'd7: 	begin
			LengthBus=19'd127;
			ScaleLengthBus=20'd128;
			end
	5'd8: 	begin
			LengthBus=19'd255;
			ScaleLengthBus=20'd256;
			end
	5'd9: 	begin
			LengthBus=19'd511;
			ScaleLengthBus=20'd512;
			end
	5'd10: 	begin
			LengthBus=19'd1023;
			ScaleLengthBus=20'd1024;
			end
	5'd11: 	begin
			LengthBus=19'd2047;
			ScaleLengthBus=20'd2048;
			end
	5'd12: 	begin
			LengthBus=19'd4095;
			ScaleLengthBus=20'd4096;
			end
	5'd13: 	begin
			LengthBus=19'd8191;
			ScaleLengthBus=20'd8192;
			end
	5'd14: 	begin
			LengthBus=19'd16383;
			ScaleLengthBus=20'd16384;
			end
	5'd15: 	begin
			LengthBus=19'd32767;
			ScaleLengthBus=20'd32768;
			end
	5'd16: 	begin
			LengthBus=19'd65535;
			ScaleLengthBus=20'd65536;
			end
	5'd17: 	begin
			LengthBus=19'd131071;
			ScaleLengthBus=20'd131072;
			end
	5'd18: 	begin
			LengthBus=19'd262143;
			ScaleLengthBus=20'd262144;
			end
	default: begin
			LengthBus=19'd524287;
			ScaleLengthBus=20'd524288;
			end
	endcase

// number of group parameter
GroupRegBus=LengthBus >> (StageReg+{4'd0,(GroupReg==0) & (FFTMach==PPS)});

// number of FFT's per group
case (StageReg+{4'd0,(GroupReg==0) & (FFTMach==PPS)})
	5'd0: GroupCntBus=19'd0;
	5'd1: GroupCntBus=19'd1;
	5'd2: GroupCntBus=19'd3;
	5'd3: GroupCntBus=19'd7;
	5'd4: GroupCntBus=19'd15;
	5'd5: GroupCntBus=19'd31;
	5'd6: GroupCntBus=19'd63;
	5'd7: GroupCntBus=19'd127;
	5'd8: GroupCntBus=19'd255;
	5'd9: GroupCntBus=19'd511;
	5'd10: GroupCntBus=19'd1023;
	5'd11: GroupCntBus=19'd2047;
	5'd12: GroupCntBus=19'd4095;
	5'd13: GroupCntBus=19'd8191;
	5'd14: GroupCntBus=19'd16383;
	5'd15: GroupCntBus=19'd32767;
	5'd16: GroupCntBus=19'd65535;
	5'd17: GroupCntBus=19'd131071;
	5'd18: GroupCntBus=19'd262143;
	default: GroupCntBus=19'd524287;
	endcase
	
case (StageReg+{4'd0,(GroupReg==0) & (FFTMach==PPS)})
	5'd0:	IndexMask=19'h7FFFF;
	5'd1:	IndexMask={18'h3FFFF,~(|IndexReg)};
	5'd2:	IndexMask={17'h1FFFF,{2{~(|IndexReg)}}};
	5'd3:	IndexMask={16'hFFFF,{3{~(|IndexReg)}}};
	5'd4:	IndexMask={15'h7FFF,{4{~(|IndexReg)}}};
	5'd5:	IndexMask={14'h3FFF,{5{~(|IndexReg)}}};
	5'd6:	IndexMask={13'h1FFF,{6{~(|IndexReg)}}};
	5'd7:	IndexMask={12'hFFF,{7{~(|IndexReg)}}};
	5'd8:	IndexMask={11'h7FF,{8{~(|IndexReg)}}};
	5'd9:	IndexMask={10'h3FF,{9{~(|IndexReg)}}};
	5'd10:	IndexMask={9'h1FF,{10{~(|IndexReg)}}};
	5'd11:	IndexMask={8'hFF,{11{~(|IndexReg)}}};
	5'd12:	IndexMask={7'h7F,{12{~(|IndexReg)}}};
	5'd13:	IndexMask={6'h3F,{13{~(|IndexReg)}}};
	5'd14:	IndexMask={5'h1F,{14{~(|IndexReg)}}};
	5'd15:	IndexMask={4'hF,{15{~(|IndexReg)}}};
	5'd16:	IndexMask={3'h7,{16{~(|IndexReg)}}};
	5'd17:	IndexMask={2'h3,{17{~(|IndexReg)}}};
	5'd18:	IndexMask={1'h1,{18{~(|IndexReg)}}};
	5'd19:	IndexMask={19{~(|IndexReg)}};
	default: IndexMask=19'h7FFFF;
	endcase
	
ScaleOffsetBus=ScaleLengthBus >> StageReg;
	
// Forming offset for even value
ReadEvenOffset={15'd0, OffsetReg};
ReadOddOffset={14'd0, {1'b0,OffsetReg}+{1'd0,GroupCntBus}+20'd1};
WriteEvenOffset={15'd0, ABus[18:0]};
WriteOddOffset={14'd0,ABus[38:19]};

// Load New sel flag
LoadNewDSC=CheckACT & ~CheckAR[3] & ~CheckNetwork;
// load lower segment
LoadLowerDSC=CheckACT & CheckAR[3] & ~CheckAR[2] & (CheckOffset[33:2]<CheckLL) & ~CheckNetwork;
// load upper segment
LoadUpperDSC=CheckACT & CheckAR[3] & ~CheckAR[2] & (CheckOffset[33:2]>=CheckUL) & ~CheckNetwork;
// access error by type
AccessError=CheckACT & CheckAR[3] & ~CheckNetwork & ((~CheckCMD & ~CheckAR[1]) | (CheckCMD & ~CheckAR[0]));

EndFlag=(StageReg==ParReg) & (GroupReg==0) & (GroupCnt==0);

end

always_ff @(posedge CLK or negedge RESET)
begin
if (~RESET)
	begin
	BusyFlag<=0;
	FFTMach<=RS;
	WRMach<=WMRS;
	DLMachine<=DLWS;
	IntACTReg<=1'b0;
	CheckACT<=0;
	TAGiReg<=0;
	MemACT<=0;
	NetACT<=0;
	DTRWriteFlag<=0;
	DTR[0]<=0;
	DTR[1]<=0;
	ErrorFlag<=0;
	DRDYFlag<=0;
	IndexReg<=0;
	end
else begin

BusyFlag<=(BusyFlag | ACT) & (~CheckACT | CheckCMD | ~CheckTag[1] | ~CheckTag[0]) & (FFTMach!=FES);

// Tag delay
TAGiReg<=MemTI[1:0];
// DRDY delay
DRDYFlag<=MemDRDY & (MemTI[7:2]==6'b111000);

// DTR selection nodes
for (i2=0; i2<2; i2=i2+1) DTRWriteFlag[i2]<=MemDRDY & MemTI[7] & MemTI[6] & MemTI[5] & MemTI[4] & MemTI[3] & (MemTI[2]==i2);
DTRWriteFlag[2]<=MemDRDY & MemTI[7] & MemTI[6] & MemTI[5] & MemTI[4] & MemTI[3];


// FFT stage register
if (FFTMach==WS)
	begin
	ParReg<=PAR;
	IndexReg<=INDEX;
	end
if (FFTMach==WS) StageReg<=5'd0;
	else if ((GroupReg==0) & (GroupCnt==0) & (FFTMach==PPS) & AFIFOEmpty) StageReg<=StageReg+5'd1;
	
// group counter
if ((FFTMach==SPS)|((FFTMach==PPS) & (GroupReg==0) & (GroupCnt==0) & AFIFOEmpty))
	begin
	GroupReg<=GroupRegBus;
	GroupCntr<=18'd0;
	end
	else if ((FFTMach==PPS) & (GroupReg!=0) & (GroupCnt==0))
			begin
			GroupReg<=GroupReg-19'd1;
			GroupCntr<=GroupCntr+19'd1;
			end
	
// in-group butterfly counter
if (FFTMach==SPS) GroupCnt<=GroupCntBus & {19{&IndexMask}};
	else if ((FFTMach==PPS) & (AFIFOEmpty | (|GroupReg) | (|GroupCnt))) GroupCnt<=(GroupCnt==0) ? GroupCntBus & {19{&IndexMask}}:GroupCnt-19'd1;

if (~BusyFlag)
	begin
	// selector registers
	ADR[0]<=DataObject;
	ADR[1]<=ControlObject;
	// Data Offset register
	DataOffsetReg<=DataOffset;
	ControlOffsetReg<=ControlOffset;
	// CPL
	CPLReg<=CPL;
	// TASKID
	TASKIDReg<=TASKID;
	// CPSR
	CPSRReg<=CPSR;
	end

// Even value offset register
if ((FFTMach==SPS)|((FFTMach==PPS) & (GroupReg==0) & (GroupCnt==0) & AFIFOEmpty)) OffsetReg<=IndexReg & ~IndexMask;
	else if ((FFTMach==PPS) & (AFIFOEmpty | (|GroupReg) | (|GroupCnt)))
				begin
				if (GroupCnt==0) OffsetReg<=(((GroupCntr+19'd1) << (StageReg+5'd1)) & IndexMask) | (IndexReg & ~IndexMask);
					else OffsetReg<=OffsetReg+19'd1;
				end

// scale offset generation
if ((FFTMach==SPS)|((FFTMach==PPS) & (GroupCnt==0))) ScaleOffset<=(IndexReg & ~IndexMask) << (ParReg-(StageReg+{4'd0,(GroupReg==0) & (FFTMach==PPS)}));
	else if ((FFTMach==PPS) & (AFIFOEmpty | (|GroupReg) | (|GroupCnt))) ScaleOffset<=ScaleOffset+ScaleOffsetBus;

				
// state machine
case (FFTMach)
	RS: FFTMach<=WS;
	WS: if (~ACT) FFTMach<=WS;
			else FFTMach<=SPS;
	// first set pointers state
	SPS: FFTMach<=RSS;
	// read scale state
	RSS: if (ErrorFlag) FFTMach<=FES;
			else if (IntNext & (WRMach==WMWS)) FFTMach<=RYS;
					else FFTMach<=RSS;
	// read Y value state
	RYS: if (ErrorFlag) FFTMach<=FES;
			else if (IntNext & (WRMach==WMWS)) FFTMach<=RXS;
					else FFTMach<=RYS;
	// read X value state
	RXS: if (ErrorFlag) FFTMach<=FES;
			else if (IntNext & (WRMach==WMWS)) FFTMach<=PPS;
					else FFTMach<=RXS;
	// promote pointers
	PPS: if (ErrorFlag) FFTMach<=FES;
			else if ((~AFIFOEmpty & (GroupReg==0) & (GroupCnt==0))|(Ausedw>=4'd14)) FFTMach<=PPS;
					else begin
					if (EndFlag) FFTMach<=ES;
						else FFTMach<=RSS;
					end
	// end of processing
	ES: FFTMach<=WS;
	// error processing
	FES: if (AFIFOEmpty) FFTMach<=ES;
			else FFTMach<=FES;
	endcase

// write machine
case (WRMach)
	WMRS: WRMach<=WMWS;
	WMWS: if (RFIFOEmpty | ErrorFlag) WRMach<=WMWS;
				else WRMach<=WMWES;
	// write even value
	WMWES: if (IntNext) WRMach<=WMWOS;
				else WRMach<=WMWES;
	// write odd value
	WMWOS: if (IntNext) WRMach<=WMWS;
				else WRMach<=WMWOS;
	endcase

// internal access register
IntACTReg<=((IntACTReg & ~FCacheINEXT)|(FFTMach==RSS)|(FFTMach==RYS)|(FFTMach==RXS)|(WRMach==WMWES)|(WRMach==WMWOS)) & ~ErrorFlag;
if (IntNext)
	begin
	// write data register
	IntDataReg<=(RBus[63:0] & {64{(WRMach==WMWES)}})|((ABus[39] ? 64'h544646464F444E45 : RBus[127:64]) & {64{(WRMach==WMWOS)}});
	// offset register
	IntOffsetReg<=((DataOffsetReg & {34{(FFTMach!=RSS)|(WRMach!=WMWS)}})+((ReadEvenOffset & {34{(FFTMach==RXS)&(WRMach==WMWS)}})|
																		(ReadOddOffset & {34{(FFTMach==RYS)&(WRMach==WMWS)}})|
																		(WriteEvenOffset & {34{WRMach==WMWES}})|
																		(WriteOddOffset & {34{WRMach==WMWOS}})) |
					(({15'd0,ScaleOffset}+ControlOffsetReg) & {34{(FFTMach==RSS)&(WRMach==WMWS)}}));
	// command
	IntCMDReg<=WRMach==WMWS;
	// Tag
	InTagReg[0]<=(WRMach!=WMWS) ? ABus[39] & (WRMach==WMWOS) : (FFTMach==RXS);
	InTagReg[1]<=(WRMach!=WMWS) ? ABus[39] & (WRMach==WMWOS) : (FFTMach==RYS);
	// selector selection
	IntSelReg<=(FFTMach==RSS)&(WRMach==WMWS);
	end

//
// Conversion logical address to physical with access verification
//
CheckACT<=((CheckACT  & ~OutNEXT) | FCacheEACT) & ~ErrorFlag;
	
// check offset and access type stage
if (CheckNext)
	begin
	// selector index
	CheckSEL<=FCacheESEL;
	// Command
	CheckCMD<=FCacheECMD;
	// Forming OFFSET 
	CheckOffset<=FCacheEOFFSET;
	// Base address limits and access rights
	CheckNetwork<=(CPU!=ADR[FCacheESEL][31:24]) & (|ADR[FCacheESEL][31:24]);
	CheckSelector<=ADR[FCacheESEL][31:0];
	// forming data to memory
	CheckData<=FCacheEDATo;
	// tag
	CheckTag<=FCacheETAGo;
	end
// reloading descriptor when he loaded from table
if ((DTRLoadedFlag) | (CheckNext))
	begin
	CheckBase<=DTRLoadedFlag ? DTR[CheckSEL][39:0] : DTR[FCacheESEL][39:0];
	CheckLL<=DTRLoadedFlag ? DTR[CheckSEL][71:40] : DTR[FCacheESEL][71:40];
	CheckUL<=DTRLoadedFlag ? DTR[CheckSEL][103:72] : DTR[FCacheESEL][103:72];
	CheckLowerSel<=DTRLoadedFlag ? DTR[CheckSEL][127:104] : DTR[FCacheESEL][127:104];
	CheckUpperSel<=DTRLoadedFlag ? DTR[CheckSEL][151:128] : DTR[FCacheESEL][151:128];
	CheckAR<=DTRLoadedFlag ? DTR[CheckSEL][155:152] : DTR[FCacheESEL][155:152];
	end

//
// output stage
//
// local memory access activation
MemACT<=(MemACT & ~MemNEXT) | (CheckACT & CheckAR[3] & ~CheckNetwork & ~CheckAR[2] & (CheckOffset[33:2]>=CheckLL) & (CheckOffset[33:2]<CheckUL) & (CheckCMD | CheckAR[1]) & 
							(~CheckCMD | CheckAR[0])) | (DLMachine==LBS) | (DLMachine==LLSS) | (DLMachine==LSLS);
	
// network access activation
NetACT<=(NetACT & ~NetNEXT) | (CheckACT & CheckNetwork);
	
if ((~MemACT | MemNEXT) & (~NetACT | NetNEXT))
	begin
	// command
	MemCMD<=CheckCMD | DescriptorLoadState;
	// forming physical address
	MemADDR[0]<=DescriptorLoadState ? (DLMachine==LLSS) : CheckOffset[0];
	MemADDR[1]<=DescriptorLoadState ? (DLMachine==LSLS) : CheckOffset[1];
	MemADDR[41:2]<=DescriptorLoadState ? DTBASE + {16'd0, DLSelector} : (CheckNetwork ? {8'd0,CheckOffset[33:2]} : CheckBase+{8'd0,(CheckOffset[33:2]-CheckLL)});
	// offset
	MemOFFSET<=CheckOffset;
	// selector (for network accesses)
	MemSEL<=CheckSelector;
	// data to memory
	MemDO<=CheckData;
	// tag	
	MemTO[0]<=DescriptorLoadState ? (DLMachine==LLSS) : CheckTag[0];
	MemTO[1]<=DescriptorLoadState ? (DLMachine==LSLS) : CheckTag[1];
	MemTO[2]<=DescriptorLoadState & CheckSEL;
	MemTO[3]<=DescriptorLoadState;
	MemTO[4]<=DescriptorLoadState;
	MemTO[7:5]<=3'b111;
	end

//
// Loading DT registers
//
DescriptorLoadState<=(DLMachine==CSS)|(DLMachine==LBS)|(DLMachine==LLSS)|(DLMachine==LSLS);
// state machine for descriptor loading
case (DLMachine)
	// wait state
	DLWS: if ((LoadNewDSC | LoadLowerDSC | LoadUpperDSC) & ~ErrorFlag) DLMachine<=CSS;
			else if (AccessError) DLMachine<=AERSS;
					else DLMachine<=DLWS;
	// checking selector value
	CSS: if (DLSelector==24'd0) DLMachine<=ZSS;
			else if (DLSelector>=DTLIMIT) DLMachine<=INVSS;
					else DLMachine<=LBS;
	// load base address and access rights
	LBS: if (~MemACT | MemNEXT) DLMachine<=LLSS;
			else DLMachine<=LBS;
	// load link selectors and check descriptor type
	LLSS: if (~MemACT | MemNEXT) DLMachine<=LSLS;
			else DLMachine<=LLSS;
	// load segment limits
	LSLS: if (~MemACT | MemNEXT) DLMachine<=RWS;
			else DLMachine<=LSLS;
	// waiting for transaction retry condition
	RWS: if (~RetryTransactionFlag) DLMachine<=RWS;
			else if (~ValidDescriptor) DLMachine<=INVOBS;
					else DLMachine<=DLWS;
	// zero selector reporting
	ZSS: DLMachine<=STS;
	// invalid selector reporting
	INVSS: DLMachine<=STS;
	// invalid object reporting (CPL or type)
	INVOBS: DLMachine<=STS;
	// access error
	AERSS: DLMachine<=STS;
	// skip transaction after error reporting
	STS: DLMachine<=DLWS;
	endcase
// temporary selector register
if (DLMachine==DLWS) for (i0=0; i0<24; i0=i0+1) DLSelector[i0]<=(LoadNewDSC & CheckSelector[i0])|
															(LoadLowerDSC & CheckLowerSel[i0])|
															(LoadUpperDSC & CheckUpperSel[i0]);
				
// 0 - base and AR, 1 - link selectors, 2 - limits
for (i1=0; i1<2; i1=i1+1)
	if (ACT & ~BusyFlag) DTR[i1][155]<=1'b0;
		else begin
		if (DTRWriteFlag[i1])
		case (TAGiReg[1:0])
			2'b00 : begin
					DTR[i1][39:0]<=MemDI[39:0];
					DTR[i1][153:152]<=MemDI[61:60];
					DTR[i1][155:154]<=MemDI[57:56];
					end
			2'b01 : begin
					DTR[i1][127:104]<=MemDI[23:0];
					DTR[i1][151:128]<=MemDI[55:32];
					end
			2'b10 :	begin
					DTR[i1][103:40]<=MemDI;
					end
			2'b11 :	DTR[i1]<=156'hB000000000000FFFFFFFF000000000000000000;
			endcase
		end
DTRLoadedFlag<=DTRWriteFlag[2] & (TAGiReg[1:0]==2'b10);
// flag to enable check transaction again
RetryTransactionFlag<=DTRLoadedFlag;

// valid descriptor flag
if (DTRWriteFlag[2] & (TAGiReg[1:0]==2'b00))
	begin
	ValidDescriptor<=MemDI[57] & (CPLReg<=MemDI[59:58]) & ((TASKIDReg==MemDI[55:40]) | (~(|TASKIDReg)) | (~(|MemDI[55:40])));
	InvalidType<=~MemDI[57] | MemDI[56];
	InvalidCPL<=(CPLReg>MemDI[59:58]);
	InvalidTaskID<=(TASKIDReg!=MemDI[55:40]) & (|TASKIDReg) & (|MemDI[55:40]);
	InvalidSelector<=CheckSelector[23:0];
	end

// System error
ErrorFlag<=(ErrorFlag & (FFTMach!=SPS)) | (DLMachine==STS);

end
end

endmodule
