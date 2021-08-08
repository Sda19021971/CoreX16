module RTE (
	// global control
	input wire CLK, RESET,
	input wire [7:0] CPUNUM,
	// external interfaces
	output reg [3:0] EXTSTBO,
	output reg [32:0]EXTDO[3:0],
	input wire [3:0] EXTSTBI,
	input wire [32:0]EXTDI[3:0],
	// core interface
	output reg STONEXT,
	input wire STONEW,
	//input wire STOVAL,
	input wire [63:0] STODT,
	input wire STINEXT,
	output wire STINEW,
	output wire STIVAL,
	output wire [63:0] STIDT
);

logic [32:0] CoreFifoDataBus;
logic [65:0] CoreFifoQBus;
logic CoreEmptyWire, CoreFifoReadWire;

// read/write wires for fifo's
logic [3:0] FifoEmptyWire;
logic [32:0] FifoBus [3:0];

// north channel resources:
// 0 - core, 1 - east, 2 - south, 3 - west
reg [3:0] NorthReqFlag;
logic [3:0] NorthACKWire;
reg NorthFirstFlag, NorthBusyFlag, NorthEndFlag;
reg [2:0] NorthCntReg, NorthStopReg;
reg [3:0] NorthSelReg;

// requests to east channel :
// 0 - north, 1 - Core, 2 - south, 3 - west
reg [3:0] EastReqFlag;
logic [3:0] EastACKWire;
reg EastFirstFlag, EastBusyFlag, EastEndFlag;
reg [2:0] EastCntReg, EastStopReg;
reg [3:0] EastSelReg;

// requests to south channel
// 0 - north, 1 - east, 2 - Core, 3 - west
reg [3:0] SouthReqFlag;
logic [3:0] SouthACKWire;
reg SouthFirstFlag, SouthBusyFlag, SouthEndFlag;
reg [2:0] SouthCntReg, SouthStopReg;
reg [3:0] SouthSelReg;

// requests to west channel
// 0 - north, 1 - east, 2 - south, 3 - Core
reg [3:0] WestReqFlag;
logic [3:0] WestACKWire;
reg WestFirstFlag, WestBusyFlag, WestEndFlag;
reg [2:0] WestCntReg, WestStopReg;
reg [3:0] WestSelReg;

// requests to the core
reg [3:0] CoreReqFlag;
logic [3:0] CoreACKWire;
reg CoreFirstFlag, CoreBusyFlag, CoreEndFlag, CoreReadEnaFlag;
reg [2:0] CoreCntReg, CoreStopReg, CoreLimReg;
reg [3:0] CoreSelReg;

logic [3:0] VerticalWire;
logic CoreVerticalWire;

logic [3:0] CoreHDistance, CoreVDistance, NorthHDistance, NorthVDistance,
			EastHDistance, EastVDistance, SouthHDistance, SouthVDistance,
			WestHDistance, WestVDistance;

reg [2:0] CoreLengthReg, NorthLengthReg, EastLengthReg, SouthLengthReg, WestLengthReg;
reg CoreShortFlag, NorthShortFlag, EastShortFlag, SouthShortFlag, WestShortFlag;

logic [3:0] CoreHRange, CoreVRange, NorthHRange, NorthVRange, EastHRange, EastVRange, SouthHRange, SouthVRange, WestHRange, WestVRange;

reg [4:0] FullDistance [4:0];

// core fifo
dcfifo_mixed_widths	CoreFIFO(.wrclk(CLK), .rdreq(CoreFifoReadWire), .aclr(~RESET), .rdclk(CLK), .wrreq(CoreBusyFlag), .data(CoreFifoDataBus),
				.rdempty(CoreEmptyWire), .q(CoreFifoQBus), .rdfull(), .rdusedw(), .wrempty(), .wrfull(), .wrusedw());
	defparam
		CoreFIFO.lpm_hint = "MAXIMIZE_SPEED=10,",
		CoreFIFO.lpm_numwords = 128,
		CoreFIFO.lpm_showahead = "ON",
		CoreFIFO.lpm_type = "dcfifo",
		CoreFIFO.lpm_width = 33,
		CoreFIFO.lpm_widthu = 7,
		CoreFIFO.lpm_widthu_r = 6,
		CoreFIFO.lpm_width_r = 66,
		CoreFIFO.overflow_checking = "ON",
		CoreFIFO.rdsync_delaypipe = 1,
		CoreFIFO.underflow_checking = "ON",
		CoreFIFO.use_eab = "ON",
		CoreFIFO.write_aclr_synch = "ON",
		CoreFIFO.wrsync_delaypipe = 1;

// north channel
sc_fifo NorthFIFO(.rdreq((EastBusyFlag & EastSelReg[0]) | (SouthBusyFlag & SouthSelReg[0]) | (WestBusyFlag & WestSelReg[0]) | (CoreBusyFlag & CoreSelReg[0] & CoreReadEnaFlag)),
				.sclr(~RESET), .clock(CLK), .wrreq(EXTSTBI[0]), .data(EXTDI[0]), .empty(FifoEmptyWire[0]), .q(FifoBus[0]));
defparam NorthFIFO.LPM_NUMWORDS=128, NorthFIFO.LPM_WIDTH=33, NorthFIFO.LPM_WIDTHU=7;
		
// east channel
sc_fifo EastFIFO(.rdreq((NorthBusyFlag & NorthSelReg[1]) | (SouthBusyFlag & SouthSelReg[1]) | (WestBusyFlag & WestSelReg[1]) | (CoreBusyFlag & CoreSelReg[1] & CoreReadEnaFlag)),
				.sclr(~RESET), .clock(CLK), .wrreq(EXTSTBI[1]), .data(EXTDI[1]), .empty(FifoEmptyWire[1]), .q(FifoBus[1]));
defparam EastFIFO.LPM_NUMWORDS=128, EastFIFO.LPM_WIDTH=33, EastFIFO.LPM_WIDTHU=7;

// south channel
sc_fifo SouthFIFO(.rdreq((NorthBusyFlag & NorthSelReg[2]) | (EastBusyFlag & EastSelReg[2]) | (WestBusyFlag & WestSelReg[2]) | (CoreBusyFlag & CoreSelReg[2] & CoreReadEnaFlag)),
				.sclr(~RESET), .clock(CLK), .wrreq(EXTSTBI[2]), .data(EXTDI[2]), .empty(FifoEmptyWire[2]), .q(FifoBus[2]));
defparam SouthFIFO.LPM_NUMWORDS=128, SouthFIFO.LPM_WIDTH=33, SouthFIFO.LPM_WIDTHU=7;
 
// west channel
sc_fifo WestFIFO(.rdreq((NorthBusyFlag & NorthSelReg[3]) | (EastBusyFlag & EastSelReg[3]) | (SouthBusyFlag & SouthSelReg[3]) | (CoreBusyFlag & CoreSelReg[3] & CoreReadEnaFlag)),
				.sclr(~RESET), .clock(CLK), .wrreq(EXTSTBI[3]), .data(EXTDI[3]), .empty(FifoEmptyWire[3]), .q(FifoBus[3]));
defparam WestFIFO.LPM_NUMWORDS=128, WestFIFO.LPM_WIDTH=33, WestFIFO.LPM_WIDTHU=7;

//=================================================================================================
//		Assignments
//=================================================================================================

assign STIDT={CoreFifoQBus[64:33],CoreFifoQBus[31:0]};
assign STINEW=CoreFifoQBus[32] & ~CoreEmptyWire;
assign STIVAL=~CoreEmptyWire;

//=================================================================================================
//		Asynchronous part
//=================================================================================================
always_comb
begin

// core fifo reading
CoreFifoReadWire=STINEXT | (~CoreEmptyWire & ~CoreFifoQBus[32]);

// vertical wires turned ON when vertical range greater than horizontal
CoreHDistance=STODT[3:0]-CPUNUM[3:0];
CoreVDistance=STODT[7:4]-CPUNUM[7:4];
CoreVerticalWire=((CoreHDistance ^ {4{CoreHDistance[3]}})+{3'd0,CoreHDistance[3]})<((CoreVDistance ^ {4{CoreVDistance[3]}})+{3'd0,CoreVDistance[3]});

NorthHDistance=FifoBus[0][3:0]-CPUNUM[3:0];
NorthVDistance=FifoBus[0][7:4]-CPUNUM[7:4];
VerticalWire[0]=((NorthHDistance ^ {4{NorthHDistance[3]}})+{3'd0,NorthHDistance[3]})<((NorthVDistance ^ {4{NorthVDistance[3]}})+{3'd0,NorthVDistance[3]});

EastHDistance=FifoBus[1][3:0]-CPUNUM[3:0];
EastVDistance=FifoBus[1][7:4]-CPUNUM[7:4];
VerticalWire[1]=((EastHDistance ^ {4{EastHDistance[3]}})+{3'd0,EastHDistance[3]})<((EastVDistance ^ {4{EastVDistance[3]}})+{3'd0,EastVDistance[3]});

SouthHDistance=FifoBus[2][3:0]-CPUNUM[3:0];
SouthVDistance=FifoBus[2][7:4]-CPUNUM[7:4];
VerticalWire[2]=((SouthHDistance ^ {4{SouthHDistance[3]}})+{3'd0,SouthHDistance[3]})<((SouthVDistance ^ {4{SouthVDistance[3]}})+{3'd0,SouthVDistance[3]});

WestHDistance=FifoBus[3][3:0]-CPUNUM[3:0];
WestVDistance=FifoBus[3][7:4]-CPUNUM[7:4];
VerticalWire[3]=((WestHDistance ^ {4{WestHDistance[3]}})+{3'd0,WestHDistance[3]})<((WestVDistance ^ {4{WestVDistance[3]}})+{3'd0,WestVDistance[3]});

// absolute lengths
NorthHRange=FifoBus[0][3:0]-FifoBus[0][11:8];
NorthVRange=FifoBus[0][7:4]-FifoBus[0][15:12];
EastHRange=FifoBus[1][3:0]-FifoBus[1][11:8];
EastVRange=FifoBus[1][7:4]-FifoBus[1][15:12];
SouthHRange=FifoBus[2][3:0]-FifoBus[2][11:8];
SouthVRange=FifoBus[2][7:4]-FifoBus[2][15:12];
WestHRange=FifoBus[3][3:0]-FifoBus[3][11:8];
WestVRange=FifoBus[3][7:4]-FifoBus[3][15:12];
CoreHRange=STODT[3:0]-CPUNUM[3:0];
CoreVRange=STODT[7:4]-CPUNUM[7:4];

// priority for north bridge
NorthACKWire[0]=NorthReqFlag[0] & (~NorthFirstFlag | ~NorthSelReg[0]) & 
									((FullDistance[4]>FullDistance[1]) | ~NorthReqFlag[1]) & ((FullDistance[4]>FullDistance[2]) | ~NorthReqFlag[2]) & 
									((FullDistance[4]>FullDistance[3]) | ~NorthReqFlag[3]);
NorthACKWire[1]=NorthReqFlag[1] & (~NorthFirstFlag | ~NorthSelReg[1]) & 
									((FullDistance[1]>=FullDistance[4]) | ~NorthReqFlag[0]) & ((FullDistance[1]>=FullDistance[2]) | ~NorthReqFlag[2]) & 
									((FullDistance[1]>=FullDistance[3]) | ~NorthReqFlag[3]);
NorthACKWire[2]=NorthReqFlag[2] & (~NorthFirstFlag | ~NorthSelReg[2]) &
									((FullDistance[2]>=FullDistance[4]) | ~NorthReqFlag[0]) & ((FullDistance[2]>FullDistance[1]) | ~NorthReqFlag[1]) &
									((FullDistance[2]>=FullDistance[3]) | ~NorthReqFlag[3]);
NorthACKWire[3]=NorthReqFlag[3] & (~NorthFirstFlag | ~NorthSelReg[3]) &
									((FullDistance[3]>=FullDistance[4]) | ~NorthReqFlag[0]) & ((FullDistance[3]>FullDistance[1]) | ~NorthReqFlag[1]) &
									((FullDistance[3]>FullDistance[2]) | ~NorthReqFlag[2]);
// priority for east bridge
EastACKWire[0]=EastReqFlag[0] & (~EastFirstFlag | ~EastSelReg[0]) &
									((FullDistance[0]>=FullDistance[4]) | ~EastReqFlag[1]) & ((FullDistance[0]>=FullDistance[2]) | ~EastReqFlag[2]) &
									((FullDistance[0]>=FullDistance[3]) | ~EastReqFlag[3]);
EastACKWire[1]=EastReqFlag[1] & (~EastFirstFlag | ~EastSelReg[1]) &
									((FullDistance[4]>FullDistance[0]) | ~EastReqFlag[0]) & ((FullDistance[4]>FullDistance[2]) | ~EastReqFlag[2]) & 
									((FullDistance[4]>FullDistance[3]) | ~EastReqFlag[3]);
EastACKWire[2]=EastReqFlag[2] & (~EastFirstFlag | ~EastSelReg[2]) &
									((FullDistance[2]>=FullDistance[4]) | ~EastReqFlag[1]) & ((FullDistance[2]>FullDistance[0]) | ~EastReqFlag[0]) &
									((FullDistance[2]>=FullDistance[3]) | ~EastReqFlag[3]);
EastACKWire[3]=EastReqFlag[3] & (~EastFirstFlag | ~EastSelReg[3]) &
									((FullDistance[3]>=FullDistance[4]) | ~EastReqFlag[1]) & ((FullDistance[3]>FullDistance[0]) | ~EastReqFlag[0]) & 
									((FullDistance[3]>FullDistance[2]) | ~EastReqFlag[2]);
// priority for south bridge
SouthACKWire[0]=SouthReqFlag[0] & (~SouthFirstFlag | ~SouthSelReg[0]) &
									((FullDistance[0]>=FullDistance[4]) | ~SouthReqFlag[2]) & ((FullDistance[0]>=FullDistance[1]) | ~SouthReqFlag[1]) &
									((FullDistance[0]>=FullDistance[3]) | ~SouthReqFlag[3]);
SouthACKWire[1]=SouthReqFlag[1] & (~SouthFirstFlag | ~SouthSelReg[1]) &
									((FullDistance[1]>=FullDistance[4]) | ~SouthReqFlag[2]) & ((FullDistance[1]>FullDistance[0]) | ~SouthReqFlag[0]) &
									((FullDistance[1]>=FullDistance[3]) | ~SouthReqFlag[3]);
SouthACKWire[2]=SouthReqFlag[2] & (~SouthFirstFlag | ~SouthSelReg[2]) &
									((FullDistance[4]>FullDistance[0]) | ~SouthReqFlag[0]) & ((FullDistance[4]>FullDistance[1]) | ~SouthReqFlag[1]) &
									((FullDistance[4]>FullDistance[3]) | ~SouthReqFlag[3]);
SouthACKWire[3]=SouthReqFlag[3] & (~SouthFirstFlag | ~SouthSelReg[3]) &
									((FullDistance[3]>=FullDistance[4]) | ~SouthReqFlag[2]) & ((FullDistance[3]>FullDistance[0]) | ~SouthReqFlag[0]) &
									((FullDistance[3]>FullDistance[1]) | ~SouthReqFlag[1]);
// priority for west bridge
WestACKWire[0]=WestReqFlag[0] & (~WestFirstFlag | ~WestSelReg[0]) &
									((FullDistance[0]>=FullDistance[4]) | ~WestReqFlag[3]) & ((FullDistance[0]>=FullDistance[1]) | ~WestReqFlag[1]) &
									((FullDistance[0]>=FullDistance[2]) | ~WestReqFlag[2]);
WestACKWire[1]=WestReqFlag[1] & (~WestFirstFlag | ~WestSelReg[1]) &
									((FullDistance[1]>=FullDistance[4]) | ~WestReqFlag[3]) & ((FullDistance[1]>FullDistance[0]) | ~WestReqFlag[0]) &
									((FullDistance[1]>=FullDistance[2]) | ~WestReqFlag[2]);
WestACKWire[2]=WestReqFlag[2] & (~WestFirstFlag | ~WestSelReg[2]) &
									((FullDistance[2]>=FullDistance[4]) | ~WestReqFlag[3]) & ((FullDistance[2]>FullDistance[0]) | ~WestReqFlag[0]) &
									((FullDistance[2]>FullDistance[1]) | ~WestReqFlag[1]);
WestACKWire[3]=WestReqFlag[3] & (~WestFirstFlag | ~WestSelReg[3]) &
									((FullDistance[4]>FullDistance[0]) | ~WestReqFlag[0]) & ((FullDistance[4]>FullDistance[1]) | ~WestReqFlag[1]) &
									((FullDistance[4]>FullDistance[2]) | ~WestReqFlag[2]);
// priority for core channel
CoreACKWire[0]=CoreReqFlag[0] & (~CoreFirstFlag | ~CoreSelReg[0]) & ((FullDistance[0]>=FullDistance[1]) | ~CoreReqFlag[1]) &
									((FullDistance[0]>=FullDistance[2]) | ~CoreReqFlag[2]) & ((FullDistance[0]>=FullDistance[3]) | ~CoreReqFlag[3]);
CoreACKWire[1]=CoreReqFlag[1] & (~CoreFirstFlag | ~CoreSelReg[1]) & ((FullDistance[1]>FullDistance[0]) | ~CoreReqFlag[0]) &
									((FullDistance[1]>=FullDistance[2]) | ~CoreReqFlag[2]) & ((FullDistance[1]>=FullDistance[3]) | ~CoreReqFlag[3]);
CoreACKWire[2]=CoreReqFlag[2] & (~CoreFirstFlag | ~CoreSelReg[2]) & ((FullDistance[2]>FullDistance[0]) | ~CoreReqFlag[0]) &
									((FullDistance[2]>FullDistance[1]) | ~CoreReqFlag[1]) & ((FullDistance[2]>=FullDistance[3]) | ~CoreReqFlag[3]);
CoreACKWire[3]=CoreReqFlag[3] & (~CoreFirstFlag | ~CoreSelReg[3]) & ((FullDistance[3]>FullDistance[0]) | ~CoreReqFlag[0]) &
									((FullDistance[3]>FullDistance[1]) | ~CoreReqFlag[1]) & ((FullDistance[3]>FullDistance[2]) | ~CoreReqFlag[2]);
// Data output for core channel
CoreFifoDataBus[32]=CoreFirstFlag;
CoreFifoDataBus[31:0]=(FifoBus[0][31:0] & {32{CoreSelReg[0]}}) |
						(FifoBus[1][31:0] & {32{CoreSelReg[1]}}) |
						(FifoBus[2][31:0] & {32{CoreSelReg[2]}}) |
						(FifoBus[3][31:0] & {32{CoreSelReg[3]}});

end

//=================================================================================================
//			Synchronous part
//=================================================================================================
always_ff @(posedge CLK)
begin

// requests to the north bridge (current CPUNUM[7:4] must be greater than CPUDST[7:4]
NorthReqFlag[0]<=|((STODT[7:4]-CPUNUM[7:4]) & 4'd8) & STONEW & ~STONEXT & CoreVerticalWire &
					(~NorthFirstFlag | ~NorthSelReg[0]);
NorthReqFlag[1]<=|((FifoBus[1][7:4]-CPUNUM[7:4]) & 4'd8) & FifoBus[1][32] & ~FifoEmptyWire[1] & VerticalWire[1] &
					(~NorthFirstFlag | ~NorthSelReg[1]);
NorthReqFlag[2]<=|((FifoBus[2][7:4]-CPUNUM[7:4]) & 4'd8) & FifoBus[2][32] & ~FifoEmptyWire[2] & VerticalWire[2] &
					(~NorthFirstFlag | ~NorthSelReg[2]);
NorthReqFlag[3]<=|((FifoBus[3][7:4]-CPUNUM[7:4]) & 4'd8) & FifoBus[3][32] & ~FifoEmptyWire[3] & VerticalWire[3] &
					(~NorthFirstFlag | ~NorthSelReg[3]);
// requests to the east bridge (current CPUNUM[3:0] must be less than CPUDST[3:0])
EastReqFlag[0]<=~(|((FifoBus[0][3:0]-CPUNUM[3:0]) & 4'd8)) & |(FifoBus[0][3:0]-CPUNUM[3:0]) & FifoBus[0][32] & ~FifoEmptyWire[0] & ~VerticalWire[0] &
					(~EastFirstFlag | ~EastSelReg[0]);
EastReqFlag[1]<=~(|((STODT[3:0]-CPUNUM[3:0]) & 4'd8)) & |(STODT[3:0] ^ CPUNUM[3:0]) & STONEW & ~STONEXT & ~CoreVerticalWire &
					(~EastFirstFlag | ~EastSelReg[1]);
EastReqFlag[2]<=~(|((FifoBus[2][3:0]-CPUNUM[3:0]) & 4'd8)) & |(FifoBus[2][3:0]-CPUNUM[3:0]) & FifoBus[2][32] & ~FifoEmptyWire[2] & ~VerticalWire[2] &
					(~EastFirstFlag | ~EastSelReg[2]);
EastReqFlag[3]<=~(|((FifoBus[3][3:0]-CPUNUM[3:0]) & 4'd8)) & |(FifoBus[3][3:0]-CPUNUM[3:0]) & FifoBus[3][32] & ~FifoEmptyWire[3] & ~VerticalWire[3] &
					(~EastFirstFlag | ~EastSelReg[3]);
// requests to the south bridge (current CPUNUM[7:4] must be less than CPUDST[7:4])
SouthReqFlag[0]<=~(|((FifoBus[0][7:4]-CPUNUM[7:4]) & 4'd8)) & |(FifoBus[0][7:4]^CPUNUM[7:4]) & FifoBus[0][32] & ~FifoEmptyWire[0] & VerticalWire[0] &
					(~SouthFirstFlag | ~SouthSelReg[0]);
SouthReqFlag[1]<=~(|((FifoBus[1][7:4]-CPUNUM[7:4]) & 4'd8)) & |(FifoBus[1][7:4]^CPUNUM[7:4]) & FifoBus[1][32] & ~FifoEmptyWire[1] & VerticalWire[1] &
					(~SouthFirstFlag | ~SouthSelReg[1]);
SouthReqFlag[2]<=~(|((STODT[7:4]-CPUNUM[7:4]) & 4'd8)) & |(STODT[7:4] ^ CPUNUM[7:4]) & STONEW & ~STONEXT & CoreVerticalWire &
					(~SouthFirstFlag | ~SouthSelReg[2]);
SouthReqFlag[3]<=~(|((FifoBus[3][7:4]-CPUNUM[7:4]) & 4'd8)) & |(FifoBus[3][7:4]^CPUNUM[7:4]) & FifoBus[3][32] & ~FifoEmptyWire[3] & VerticalWire[3] &
					(~SouthFirstFlag | ~SouthSelReg[3]);
// requests to the west bridge (current CPUNUM[3:0] must be greater than CPUDST[3:0])
WestReqFlag[0]<=|((FifoBus[0][3:0]-CPUNUM[3:0]) & 4'd8) & FifoBus[0][32] & ~FifoEmptyWire[0] & ~VerticalWire[0] &
					(~WestFirstFlag | ~WestSelReg[0]);
WestReqFlag[1]<=|((FifoBus[1][3:0]-CPUNUM[3:0]) & 4'd8) & FifoBus[1][32] & ~FifoEmptyWire[1] & ~VerticalWire[1] &
					(~WestFirstFlag | ~WestSelReg[1]);
WestReqFlag[2]<=|((FifoBus[2][3:0]-CPUNUM[3:0]) & 4'd8) & FifoBus[2][32] & ~FifoEmptyWire[2] & ~VerticalWire[2] &
					(~WestFirstFlag | ~WestSelReg[2]);
WestReqFlag[3]<=|((STODT[3:0]-CPUNUM[3:0]) & 4'd8) & STONEW & ~STONEXT & ~CoreVerticalWire &
					(~WestFirstFlag | ~WestSelReg[3]);
// requests to the core
CoreReqFlag[0]<=(FifoBus[0][7:0]==CPUNUM) & FifoBus[0][32] & ~FifoEmptyWire[0] & (~CoreFirstFlag | ~CoreSelReg[0]);
CoreReqFlag[1]<=(FifoBus[1][7:0]==CPUNUM) & FifoBus[1][32] & ~FifoEmptyWire[1] & (~CoreFirstFlag | ~CoreSelReg[1]);
CoreReqFlag[2]<=(FifoBus[2][7:0]==CPUNUM) & FifoBus[2][32] & ~FifoEmptyWire[2] & (~CoreFirstFlag | ~CoreSelReg[2]);
CoreReqFlag[3]<=(FifoBus[3][7:0]==CPUNUM) & FifoBus[3][32] & ~FifoEmptyWire[3] & (~CoreFirstFlag | ~CoreSelReg[3]);


//----------------------------------------------------------
// packet length from the core
casez (STODT[18:16])
	// new packet, write
	3'd0:	case (STODT[23:22])
				2'd0:	CoreLengthReg<=3'd3;
				2'd1:	CoreLengthReg<=3'd3;
				2'd2:	CoreLengthReg<=3'd4;
				2'd3:	CoreLengthReg<=3'd5;
				endcase
	// new packet, read
	3'd1:	CoreLengthReg<=3'd3;
	// short packet, write
	3'd2:	case (STODT[23:22])
				2'd0:	CoreLengthReg<=3'd1;
				2'd1:	CoreLengthReg<=3'd1;
				2'd2:	CoreLengthReg<=3'd2;
				2'd3:	CoreLengthReg<=3'd3;
				endcase
	// short packet, read
	3'd3:	CoreLengthReg<=3'd1;
	// message packet
	3'd4:	CoreLengthReg<=3'd4;
	// message processing termination
	3'd5:	CoreLengthReg<=3'd1;
	// data retry
	3'd6:	case (STODT[23:22])
				2'd0:	CoreLengthReg<={2'd0,~STODT[19]};
				2'd1:	CoreLengthReg<={2'd0,~STODT[19]};
				2'd2:	CoreLengthReg<={2'd0,~STODT[19]};
				2'd3:	CoreLengthReg<={1'd0,~STODT[19],1'd0};
				endcase
	// reserved frame type
	3'd7:	CoreLengthReg<=3'd0;
	endcase
CoreShortFlag<=STODT[19] & STODT[18] & STODT[17] & ~STODT[16];

//----------------------------------------------------------
// packet length from the north bridge
casez (FifoBus[0][18:16])
	// new packet, write
	3'd0:	case (FifoBus[0][23:22])
				2'd0:	NorthLengthReg<=3'd3;
				2'd1:	NorthLengthReg<=3'd3;
				2'd2:	NorthLengthReg<=3'd4;
				2'd3:	NorthLengthReg<=3'd5;
				endcase
	// new packet, read
	3'd1:	NorthLengthReg<=3'd3;
	// short packet, write
	3'd2:	case (FifoBus[0][23:22])
				2'd0:	NorthLengthReg<=3'd1;
				2'd1:	NorthLengthReg<=3'd1;
				2'd2:	NorthLengthReg<=3'd2;
				2'd3:	NorthLengthReg<=3'd3;
				endcase
	// short packet, read
	3'd3:	NorthLengthReg<=3'd1;
	// message packet
	3'd4:	NorthLengthReg<=3'd4;
	// message processing termination
	3'd5:	NorthLengthReg<=3'd1;
	// data retry
	3'd6:	case (FifoBus[0][23:22])
				2'd0:	NorthLengthReg<={2'd0,~FifoBus[0][19]};
				2'd1:	NorthLengthReg<={2'd0,~FifoBus[0][19]};
				2'd2:	NorthLengthReg<={2'd0,~FifoBus[0][19]};
				2'd3:	NorthLengthReg<={1'd0,~FifoBus[0][19],1'd0};
				endcase
	// reserved frame type
	3'd7:	NorthLengthReg<=3'd0;
	endcase
NorthShortFlag<=FifoBus[0][19] & FifoBus[0][18] & FifoBus[0][17] & ~FifoBus[0][16];

//----------------------------------------------------------
// packet length from the east bridge
casez (FifoBus[1][18:16])
	// new packet, write
	3'd0:	case (FifoBus[1][23:22])
				2'd0:	EastLengthReg<=3'd3;
				2'd1:	EastLengthReg<=3'd3;
				2'd2:	EastLengthReg<=3'd4;
				2'd3:	EastLengthReg<=3'd5;
				endcase
	// new packet, read
	3'd1:	EastLengthReg<=3'd3;
	// short packet, write
	3'd2:	case (FifoBus[1][23:22])
				2'd0:	EastLengthReg<=3'd1;
				2'd1:	EastLengthReg<=3'd1;
				2'd2:	EastLengthReg<=3'd2;
				2'd3:	EastLengthReg<=3'd3;
				endcase
	// short packet, read
	3'd3:	EastLengthReg<=3'd1;
	// message packet
	3'd4:	EastLengthReg<=3'd4;
	// message processing termination
	3'd5:	EastLengthReg<=3'd1;
	// data retry
	3'd6:	case (FifoBus[1][23:22])
				2'd0:	EastLengthReg<={2'd0,~FifoBus[1][19]};
				2'd1:	EastLengthReg<={2'd0,~FifoBus[1][19]};
				2'd2:	EastLengthReg<={2'd0,~FifoBus[1][19]};
				2'd3:	EastLengthReg<={1'd0,~FifoBus[1][19],1'd0};
				endcase
	// reserved frame type
	3'd7:	EastLengthReg<=3'd0;
	endcase
EastShortFlag<=FifoBus[1][19] & FifoBus[1][18] & FifoBus[1][17] & ~FifoBus[1][16];

//----------------------------------------------------------
// packet length from the south bridge
casez (FifoBus[2][18:16])
	// new packet, write
	3'd0:	case (FifoBus[2][23:22])
				2'd0:	SouthLengthReg<=3'd3;
				2'd1:	SouthLengthReg<=3'd3;
				2'd2:	SouthLengthReg<=3'd4;
				2'd3:	SouthLengthReg<=3'd5;
				endcase
	// new packet, read
	3'd1:	SouthLengthReg<=3'd3;
	// short packet, write
	3'd2:	case (FifoBus[2][23:22])
				2'd0:	SouthLengthReg<=3'd1;
				2'd1:	SouthLengthReg<=3'd1;
				2'd2:	SouthLengthReg<=3'd2;
				2'd3:	SouthLengthReg<=3'd3;
				endcase
	// short packet, read
	3'd3:	SouthLengthReg<=3'd1;
	// message packet
	3'd4:	SouthLengthReg<=3'd4;
	// message processing termination
	3'd5:	SouthLengthReg<=3'd1;
	// data retry
	3'd6:	case (FifoBus[2][23:22])
				2'd0:	SouthLengthReg<={2'd0,~FifoBus[2][19]};
				2'd1:	SouthLengthReg<={2'd0,~FifoBus[2][19]};
				2'd2:	SouthLengthReg<={2'd0,~FifoBus[2][19]};
				2'd3:	SouthLengthReg<={1'd0,~FifoBus[2][19],1'd0};
				endcase
	// reserved frame type
	3'd7:	SouthLengthReg<=3'd0;
	endcase
SouthShortFlag<=FifoBus[2][19] & FifoBus[2][18] & FifoBus[2][17] & ~FifoBus[2][16];

//----------------------------------------------------------
// packet length from the west bridge
casez (FifoBus[3][18:16])
	// new packet, write
	3'd0:	case (FifoBus[3][23:22])
				2'd0:	WestLengthReg<=3'd3;
				2'd1:	WestLengthReg<=3'd3;
				2'd2:	WestLengthReg<=3'd4;
				2'd3:	WestLengthReg<=3'd5;
				endcase
	// new packet, read
	3'd1:	WestLengthReg<=3'd3;
	// short packet, write
	3'd2:	case (FifoBus[3][23:22])
				2'd0:	WestLengthReg<=3'd1;
				2'd1:	WestLengthReg<=3'd1;
				2'd2:	WestLengthReg<=3'd2;
				2'd3:	WestLengthReg<=3'd3;
				endcase
	// short packet, read
	3'd3:	WestLengthReg<=3'd1;
	// message packet
	3'd4:	WestLengthReg<=3'd4;
	// message processing termination
	3'd5:	WestLengthReg<=3'd1;
	// data retry
	3'd6:	case (FifoBus[3][23:22])
				2'd0:	WestLengthReg<={2'd0,~FifoBus[3][19]};
				2'd1:	WestLengthReg<={2'd0,~FifoBus[3][19]};
				2'd2:	WestLengthReg<={2'd0,~FifoBus[3][19]};
				2'd3:	WestLengthReg<={1'd0,~FifoBus[3][19],1'd0};
				endcase
	// reserved frame type
	3'd7:	WestLengthReg<=3'd0;
	endcase
WestShortFlag<=FifoBus[3][19] & FifoBus[3][18] & FifoBus[3][17] & ~FifoBus[3][16];


// distances calculation
FullDistance[0]<={5{FifoBus[0][32]}} & 
				({1'b0,((NorthHRange ^ {4{NorthHRange[3]}})+{3'd0,NorthHRange[3]})}+{1'b0,((NorthVRange ^ {4{NorthVRange[3]}})+{3'd0,NorthVRange[3]})});
FullDistance[1]<={5{FifoBus[1][32]}} &
				({1'b0,((EastHRange ^ {4{EastHRange[3]}})+{3'd0,EastHRange[3]})}+{1'b0,((EastVRange ^ {4{EastVRange[3]}})+{3'd0,EastVRange[3]})});
FullDistance[2]<={5{FifoBus[2][32]}} &
				({1'b0,((SouthHRange ^ {4{SouthHRange[3]}})+{3'd0,SouthHRange[3]})}+{1'b0,((SouthVRange ^ {4{SouthVRange[3]}})+{3'd0,SouthVRange[3]})});
FullDistance[3]<={5{FifoBus[3][32]}} &
				({1'b0,((WestHRange ^ {4{WestHRange[3]}})+{3'd0,WestHRange[3]})}+{1'b0,((WestVRange ^ {4{WestVRange[3]}})+{3'd0,WestVRange[3]})});
FullDistance[4]<={5{STONEW}} &
				({1'b0,((CoreHRange ^ {4{CoreHRange[3]}})+{3'd0,CoreHRange[3]})}+{1'b0,((CoreVRange ^ {4{CoreVRange[3]}})+{3'd0,CoreVRange[3]})});


/////////////////////////////////////////////////
//			NORTH CHANNEL
/////////////////////////////////////////////////
// first dword flag
NorthFirstFlag<=|NorthACKWire & ((~NorthCntReg[2] & ~NorthCntReg[1] & ~NorthCntReg[0]) | NorthEndFlag);
// channel selection
if (NorthEndFlag | (~NorthCntReg[2] & ~NorthCntReg[1] & ~NorthCntReg[0])) 
	begin
	NorthSelReg<=NorthACKWire;
	NorthStopReg<=(CoreLengthReg & {3{NorthACKWire[0]}}) | (EastLengthReg & {3{NorthACKWire[1]}}) |
				(SouthLengthReg & {3{NorthACKWire[2]}}) | (WestLengthReg & {3{NorthACKWire[3]}});
	end
// Data output for north channel
EXTSTBO[0]<=NorthBusyFlag & RESET;
EXTDO[0][32]<=NorthFirstFlag;
EXTDO[0][31:0]<=(STODT[63:32] & {32{NorthSelReg[0] & ~NorthCntReg[0]}}) |
				(STODT[31:0] & {32{NorthSelReg[0] & NorthCntReg[0]}}) |
				(FifoBus[1][31:0] & {32{NorthSelReg[1]}}) |
				(FifoBus[2][31:0] & {32{NorthSelReg[2]}}) |
				(FifoBus[3][31:0] & {32{NorthSelReg[3]}});
// cycle counter
if (~RESET) NorthCntReg<=3'd0;
	else if ((|NorthCntReg) | (|NorthACKWire)) NorthCntReg<=((NorthCntReg+3'd1) & {3{~NorthEndFlag}}) | ({2'd0,NorthEndFlag & (|NorthACKWire)});
// north bridge busy flag
if (~RESET)	NorthBusyFlag<=1'b0;
	else NorthBusyFlag<=(NorthBusyFlag  & ~NorthEndFlag) | (|NorthACKWire);
// End of transmission flag
NorthEndFlag<=((NorthEndFlag | (~NorthCntReg[2] & ~NorthCntReg[1] & ~NorthCntReg[0])) & 
					((CoreShortFlag & NorthACKWire[0]) | (EastShortFlag & NorthACKWire[1]) | (SouthShortFlag & NorthACKWire[2]) | (WestShortFlag & NorthACKWire[3]))) | 
			((NorthCntReg==NorthStopReg) & NorthBusyFlag);


/////////////////////////////////////////////////
//			EAST CHANNEL
/////////////////////////////////////////////////
// first dword flag
EastFirstFlag<=|EastACKWire & ((~EastCntReg[2] & ~EastCntReg[1] & ~EastCntReg[0]) | EastEndFlag);
// channel selection
if (EastEndFlag | (~EastCntReg[2] & ~EastCntReg[1] & ~EastCntReg[0])) 
	begin
	EastSelReg<=EastACKWire;
	EastStopReg<=(NorthLengthReg & {3{EastACKWire[0]}}) | (CoreLengthReg & {3{EastACKWire[1]}}) |
				(SouthLengthReg & {3{EastACKWire[2]}}) | (WestLengthReg & {3{EastACKWire[3]}});
	end
// Data output for east channel
EXTSTBO[1]<=EastBusyFlag & RESET;
EXTDO[1][32]<=EastFirstFlag;
EXTDO[1][31:0]<=(FifoBus[0][31:0] & {32{EastSelReg[0]}}) |
				(STODT[63:32] & {32{EastSelReg[1] & ~EastCntReg[0]}}) |
				(STODT[31:0] & {32{EastSelReg[1] & EastCntReg[0]}}) |
				(FifoBus[2][31:0] & {32{EastSelReg[2]}}) |
				(FifoBus[3][31:0] & {32{EastSelReg[3]}});
// cycle counter
if (~RESET) EastCntReg<=3'd0;
	else if ((|EastCntReg) | (|EastACKWire)) EastCntReg<=((EastCntReg+3'd1) & {3{~EastEndFlag}}) | ({2'd0,EastEndFlag & (|EastACKWire)});
// north bridge busy flag
if (~RESET)	EastBusyFlag<=1'b0;
	else EastBusyFlag<=(EastBusyFlag  & ~EastEndFlag) | (|EastACKWire);
// End of transmission flag
EastEndFlag<=((EastEndFlag | (~EastCntReg[2] & ~EastCntReg[1] & ~EastCntReg[0])) & 
					((NorthShortFlag & EastACKWire[0]) | (CoreShortFlag & EastACKWire[1]) | (SouthShortFlag & EastACKWire[2]) | (WestShortFlag & EastACKWire[3]))) | 
			((EastCntReg==EastStopReg) & EastBusyFlag);


/////////////////////////////////////////////////
//			SOUTH CHANNEL
/////////////////////////////////////////////////
// first dword flag
SouthFirstFlag<=|SouthACKWire & ((~SouthCntReg[2] & ~SouthCntReg[1] & ~SouthCntReg[0]) | SouthEndFlag);
// channel selection
if (SouthEndFlag | (~SouthCntReg[2] & ~SouthCntReg[1] & ~SouthCntReg[0])) 
	begin
	SouthSelReg<=SouthACKWire;
	SouthStopReg<=(NorthLengthReg & {3{SouthACKWire[0]}}) | (EastLengthReg & {3{SouthACKWire[1]}}) |
				(CoreLengthReg & {3{SouthACKWire[2]}}) | (WestLengthReg & {3{SouthACKWire[3]}});
	end
// Data output for south channel
EXTSTBO[2]<=SouthBusyFlag & RESET;
EXTDO[2][32]<=SouthFirstFlag;
EXTDO[2][31:0]<=(FifoBus[0][31:0] & {32{SouthSelReg[0]}}) |
				(FifoBus[1][31:0] & {32{SouthSelReg[1]}}) |
				(STODT[63:32] & {32{SouthSelReg[2] & ~SouthCntReg[0]}}) |
				(STODT[31:0] & {32{SouthSelReg[2] & SouthCntReg[0]}}) |
				(FifoBus[3][31:0] & {32{SouthSelReg[3]}});
// cycle counter
if (~RESET) SouthCntReg<=3'd0;
	else if ((|SouthCntReg) | (|SouthACKWire)) SouthCntReg<=((SouthCntReg+3'd1) & {3{~SouthEndFlag}}) | ({2'd0,SouthEndFlag & (|SouthACKWire)});
// north bridge busy flag
if (~RESET)	SouthBusyFlag<=1'b0;
	else SouthBusyFlag<=(SouthBusyFlag  & ~SouthEndFlag) | (|SouthACKWire);
// End of transmission flag
SouthEndFlag<=((SouthEndFlag | (~SouthCntReg[2] & ~SouthCntReg[1] & ~SouthCntReg[0])) & 
					((NorthShortFlag & SouthACKWire[0]) | (EastShortFlag & SouthACKWire[1]) | (CoreShortFlag & SouthACKWire[2]) | (WestShortFlag & SouthACKWire[3]))) | 
			((SouthCntReg==SouthStopReg) & SouthBusyFlag);


/////////////////////////////////////////////////
//			WEST CHANNEL
/////////////////////////////////////////////////
// first dword flag
WestFirstFlag<=|WestACKWire & ((~WestCntReg[2] & ~WestCntReg[1] & ~WestCntReg[0]) | WestEndFlag);
// channel selection
if (WestEndFlag | (~WestCntReg[2] & ~WestCntReg[1] & ~WestCntReg[0])) 
	begin
	WestSelReg<=WestACKWire;
	WestStopReg<=(NorthLengthReg & {3{WestACKWire[0]}}) | (EastLengthReg & {3{WestACKWire[1]}}) |
				(SouthLengthReg & {3{EastACKWire[2]}}) | (CoreLengthReg & {3{WestACKWire[3]}});
	end
// Data output for west channel
EXTSTBO[3]<=WestBusyFlag & RESET;
EXTDO[3][32]<=WestFirstFlag;
EXTDO[3][31:0]<=(FifoBus[0][31:0] & {32{WestSelReg[0]}}) |
				(FifoBus[1][31:0] & {32{WestSelReg[1]}}) |
				(FifoBus[2][31:0] & {32{WestSelReg[2]}}) |
				(STODT[63:32] & {32{WestSelReg[3] & ~WestCntReg[0]}}) |
				(STODT[31:0] & {32{WestSelReg[3] & WestCntReg[0]}});
// cycle counter
if (~RESET) WestCntReg<=3'd0;
	else if ((|WestCntReg) | (|WestACKWire)) WestCntReg<=((WestCntReg+3'd1) & {3{~WestEndFlag}}) | ({2'd0,WestEndFlag & (|WestACKWire)});
// north bridge busy flag
if (~RESET)	WestBusyFlag<=1'b0;
	else WestBusyFlag<=(WestBusyFlag  & ~WestEndFlag) | (|WestACKWire);
// End of transmission flag
WestEndFlag<=((WestEndFlag | (~WestCntReg[2] & ~WestCntReg[1] & ~WestCntReg[0])) & 
					((NorthShortFlag & WestACKWire[0]) | (EastShortFlag & WestACKWire[1]) | (SouthShortFlag & WestACKWire[2]) | (CoreShortFlag & WestACKWire[3]))) | 
			((WestCntReg==WestStopReg) & WestBusyFlag);


/////////////////////////////////////////////////
//			CORE CHANNEL
/////////////////////////////////////////////////
// first dword flag
CoreFirstFlag<=|CoreACKWire & ((~CoreCntReg[2] & ~CoreCntReg[1] & ~CoreCntReg[0]) | CoreEndFlag);
// channel selection
if (CoreEndFlag | (~CoreCntReg[2] & ~CoreCntReg[1] & ~CoreCntReg[0])) 
	begin
	CoreSelReg<=CoreACKWire;
	CoreStopReg<=((NorthLengthReg+{2'd0,~NorthLengthReg[0]}) & {3{CoreACKWire[0]}}) | ((EastLengthReg+{2'd0,~EastLengthReg[0]}) & {3{CoreACKWire[1]}}) |
				((SouthLengthReg + {2'd0,~SouthLengthReg[0]}) & {3{CoreACKWire[2]}}) | ((WestLengthReg+{2'd0,~WestLengthReg[0]}) & {3{CoreACKWire[3]}});
	CoreLimReg<=((NorthLengthReg+3'd1) & {3{CoreACKWire[0]}}) | ((EastLengthReg+3'd1) & {3{CoreACKWire[1]}}) |
				((SouthLengthReg+3'd1) & {3{CoreACKWire[2]}}) | ((WestLengthReg+3'd1) & {3{CoreACKWire[3]}});
	end
// cycle counter
if (~RESET) CoreCntReg<=3'd0;
	else if ((|CoreCntReg) | (|CoreACKWire)) CoreCntReg<=((CoreCntReg+3'd1) & {3{~CoreEndFlag}}) | ({2'd0,CoreEndFlag & (|CoreACKWire)});
// core bridge busy flag
if (~RESET)	CoreBusyFlag<=1'b0;
	else CoreBusyFlag<=(CoreBusyFlag  & ~CoreEndFlag) | (|CoreACKWire);
// End of transmission flag
CoreEndFlag<=((CoreCntReg==CoreStopReg) & CoreBusyFlag);
// end of read fifo flag
if (~RESET) CoreReadEnaFlag<=1'b0;
	else CoreReadEnaFlag<=(CoreReadEnaFlag & (~(CoreLimReg==CoreCntReg) | ~CoreBusyFlag)) | ((|CoreACKWire) & (CoreEndFlag | ~CoreBusyFlag));


// next qword for core channel
STONEXT<=(NorthBusyFlag & NorthCntReg[0] & NorthSelReg[0]) | (EastBusyFlag & EastCntReg[0] & EastSelReg[1]) |
		(SouthBusyFlag & SouthCntReg[0] & SouthSelReg[2]) | (WestBusyFlag & WestCntReg[0] & WestSelReg[3]);
end

endmodule
