module StreamController (
	input wire CLK, RESET, TE,
	input wire [39:0] DTBASE,
	output wire INIT,
// Core interface
	output wire CoreNEXT,
	input wire CoreACT, CoreCMD, CoreADDR,
	input wire [23:0] CoreSEL,
	input wire [63:0] CoreDTI,
	input wire [7:0] CoreTAGI,
	output reg CoreDRDY,
	output reg [63:0] CoreDTO,
	output reg [7:0] CoreTAGO,
	output reg [2:0] CoreSIZEO,
// network interface
	output wire NetNEXT,
	input wire NetACT,
	input wire [23:0] NetSEL,
	input wire [63:0] NetDATA,
// memory interface
	input wire NEXT,
	output reg ACT, CMD,
	output reg [1:0] SIZEO,
	output reg [44:0] ADDR,
	output reg [63:0] DTO,
	output reg [4:0] TAGO,
	input wire DRDY,
	input wire [63:0] DTI,
	input wire [4:0] TAGI
);

integer i,ii, i1, i2, i3, i4, i5, i6, i7, i8, i9, ia, ib;


//-----------------------------------------------
//		read channel resources
logic ReadEna;
reg ReadACTReg, ReadLockFlag, ReadAddrReg;
reg [3:0] ReadMatchFlag;
reg [7:0] ReadTagReg;
reg [23:0] ReadSelReg;
reg [15:0] TypeReg [3:0];

//-----------------------------------------------
//		write channel resources
logic WriteEna;
reg WriteACTReg;
reg [3:0] WriteMatchFlag;
reg [23:0] WriteSelReg;
reg [63:0] WriteDataReg;
logic [23:0] SelBus;
logic [63:0] DIBus;

//-----------------------------------------------
//		data output resources
reg CoreDRDYFlag;
reg [3:0] DataReadyReg, DataAbortReg, DataSelReg;
reg [7:0] CoreSizeReg;

//-----------------------------------------------
//		uncached transaction resources
reg IntACTReg, SelMissFlag;
reg [23:0] IntSelReg;
reg [63:0] IntDataReg;
reg [125:0] IntDescReg, BypassReg;
logic MemEna, IntEna;
logic [28:0] IntOffsetBus;

//-----------------------------------------------
//		memory channel
reg MemACTReg, MemCmdReg;
reg [28:0] MemOffsetReg;
reg [39:0] MemBaseReg;
reg [63:0] MemDataReg;
reg [1:0] MemSizeReg;
reg [4:0] MemTagReg;

//-----------------------------------------------
// 		cahnnel resources
reg [23:0] ChannelSelReg [3:0];
reg [125:0] ChannelDescReg [3:0];
reg [3:0] ChannelCntReg [3:0];		// unstored data counters
reg [3:0] WriteToFifoFlag, CntReloadFlag, TMOFlag;			// enable write data into fifo
reg [3:0] DFWPtr [3:0];				// data fifo write pointers
reg [3:0] DFRPtr [3:0];				// data fifo read  pointers
reg [3:0] TFWPtr [3:0];				// tag fifo write pointers
reg [3:0] TFRPtr [3:0];				// tag fifo read pointers
reg [3:0] AbortCntr [3:0];			// aborted transaction counters
reg [3:0] ChannelLRUReg [3:0];
reg [3:0] DataReady, DataAbort, CntrRead, DisableCntReg;
reg [7:0] TMOReg [3:0];
logic [1:0] DataChannel;
logic FifoLoadFromMem;				// loading from memory data to the fifo
logic [3:0] LoadWABus;
logic [5:0] DRAMWABus, DRAMRABus, TRAMWABus, TRAMRABus;

//-----------------------------------------------
//		control resources
reg WriteLockFlag, DSetFlag, BypassFlag, CtrlCmdReg, MachineWaitFlag, CCFlag, CtrlACTReg, DLoadFlag, ReadDescFlag, OneCountFlag,
	StrobeDescFlag;
reg [8:0] ClearReg;
reg [69:0] DescReg;
reg [25:0] CtrlOffsetReg;
reg [63:0] CtrlDataReg;
reg [1:0] CtrlSizeReg;
reg [3:0] CtrlSelFlag, CtrlStoreCntReg, LoadReqFlag, LockLoadReqFlag;
reg [2:0] CtrlLoadCntReg;
reg [3:0] SCntReg [3:0];
reg [2:0] LCntReg [3:0];
reg [4:0] CtrlTagReg;
reg [19:0] LoadOffsetReg, StoreOffsetReg, CtrlMaskReg, CtrlWPtrReg, StoreMaskReg;
logic [28:0] CtrlOffsetBus;
logic [39:0] CtrlBaseBus;
logic ChannelChange, MustWrite, ChannelValid, EndRead, EndWrite, CtrlEna, LoadReq, MustWait;
logic [1:0] CtrlSelBus;
logic [4:0] LRUBus [1:0];
logic [3:0] SCntBus [3:0];
logic [19:0] LCntBus [3:0];
reg [19:0] ChannelCountReg [3:0];
logic PreventSetUpWriteToFIFO;
reg [3:0] ModifyWhileLoadFlag;


reg [3:0] Machine, StackReg;
parameter IDLEState=4'd0, DLPrepareState=4'd1, DLFirstState=4'd2, DLSecondState=4'd3, DLWaitState=4'd4, ResetState=4'd5, InitChannelState=4'd6,
			CheckChannelState=4'd7, WriteState=4'd8, StoreDescState=4'd9, GetDescState=4'd10, CheckNewState=4'd11, LoadState=4'd12, 
			LoadPrepareState=4'd13, EndLoadState=4'd14, FetchState=4'd15;

// address ram
// base address			40 bit 	[39:0]
// selector upper bits	16 bit	[55:40]
// cycle mask			20 bit	[75:56]
// read pointer			20 bit	[95:76]
// write pointer		20 bit	[115:96]
// channel timeout		8 bit	[123:116]
// channel size			2 bit	[125:124]

// Channel A - access from core controller
// Channel B - access from network and core write transactions

logic [125:0] ARAMOutBus, ARAMInBus;
reg [125:0] ARAMInReg;
logic ARAMwren;

altsyncram	ARAM(.clocken0(1'b1), .clocken1(WriteEna | ReadDescFlag | (StrobeDescFlag & WriteACTReg)), .wren_a(ARAMwren), .clock0(CLK), .clock1(CLK),
				.address_a(ClearReg[8] ? ((Machine==StoreDescState) ? (ChannelSelReg[0][7:0] & {8{CtrlSelFlag[0]}}) | (ChannelSelReg[1][7:0] & {8{CtrlSelFlag[1]}}) |
																		(ChannelSelReg[2][7:0] & {8{CtrlSelFlag[2]}}) | (ChannelSelReg[3][7:0] & {8{CtrlSelFlag[3]}}) : 
																		IntSelReg[7:0]) : ClearReg[7:0]),
				.address_b(ReadDescFlag ? ReadSelReg[7:0] : ((StrobeDescFlag & WriteACTReg) ? WriteSelReg[7:0] : ((~CoreACT | CoreCMD) ? NetSEL[7:0] : CoreSEL[7:0]))),
				.data_a(ARAMInBus), .q_b(ARAMOutBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clocken2(1'b1),
				.clocken3(1'b1), .data_b({126{1'b1}}), .eccstatus(), .q_a(), .rden_a(1'b1), .rden_b(1'b1), .wren_b(1'b0));
defparam
		ARAM.address_reg_b = "CLOCK1", ARAM.clock_enable_input_a = "NORMAL", ARAM.clock_enable_input_b = "NORMAL", ARAM.clock_enable_output_a = "BYPASS",
		ARAM.clock_enable_output_b = "BYPASS", ARAM.lpm_type = "altsyncram", ARAM.numwords_a = 256,
		ARAM.numwords_b = 256, ARAM.operation_mode = "DUAL_PORT", ARAM.outdata_aclr_b = "NONE", ARAM.outdata_reg_b = "UNREGISTERED",
		ARAM.power_up_uninitialized = "FALSE", ARAM.widthad_a = 8, ARAM.widthad_b = 8, ARAM.width_a = 126, ARAM.width_b = 126, ARAM.width_byteena_a = 1;

logic [63:0] DRAMOutBus;
logic DRAMwren;

altsyncram	DRAM(.wren_a(DRAMwren), .clock0(CLK), .address_a(FifoLoadFromMem ? {TAGI[3:2],LoadWABus} : DRAMWABus), .address_b(DRAMRABus), 
				.data_a(FifoLoadFromMem ? DTI : WriteDataReg), .q_b(DRAMOutBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1), .clocken0(1'b1),
				.clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1), .data_b({64{1'b1}}), .eccstatus(), .q_a(), .rden_a(1'b1), .rden_b(1'b1), .wren_b(1'b0));
defparam
		DRAM.address_reg_b = "CLOCK0", DRAM.clock_enable_input_a = "BYPASS", DRAM.clock_enable_input_b = "BYPASS", DRAM.clock_enable_output_a = "BYPASS",
		DRAM.clock_enable_output_b = "BYPASS", DRAM.lpm_type = "altsyncram", DRAM.numwords_a = 64, DRAM.numwords_b = 64,
		DRAM.operation_mode = "DUAL_PORT", DRAM.outdata_aclr_b = "NONE", DRAM.outdata_reg_b = "UNREGISTERED", DRAM.power_up_uninitialized = "FALSE",
		DRAM.read_during_write_mode_mixed_ports = "DONT_CARE", DRAM.widthad_a = 6, DRAM.widthad_b = 6, DRAM.width_a = 64, DRAM.width_b = 64, DRAM.width_byteena_a = 1;


// TAG's FIFO system
logic [7:0] TRAMOutBus;

altsyncram	TRAM(.wren_a((ReadACTReg & (|ReadMatchFlag))), .clock0(CLK), .address_a(TRAMWABus), .address_b(TRAMRABus), 
				.data_a(ReadTagReg), .q_b(TRAMOutBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1), .clocken0(1'b1),
				.clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1), .data_b({8{1'b1}}), .eccstatus(), .q_a(), .rden_a(1'b1), .rden_b(1'b1), .wren_b(1'b0));
defparam
		TRAM.address_reg_b = "CLOCK0", TRAM.clock_enable_input_a = "BYPASS", TRAM.clock_enable_input_b = "BYPASS", TRAM.clock_enable_output_a = "BYPASS",
		TRAM.clock_enable_output_b = "BYPASS", TRAM.lpm_type = "altsyncram", TRAM.numwords_a = 64,
		TRAM.numwords_b = 64, TRAM.operation_mode = "DUAL_PORT", TRAM.outdata_aclr_b = "NONE", TRAM.outdata_reg_b = "UNREGISTERED", TRAM.power_up_uninitialized = "FALSE",
		TRAM.read_during_write_mode_mixed_ports = "DONT_CARE", TRAM.widthad_a = 6, TRAM.widthad_b = 6, TRAM.width_a = 8, TRAM.width_b = 8, TRAM.width_byteena_a = 1;


//=================================================================================================
//				output pin assignments
//=================================================================================================

assign CoreNEXT=CoreCMD ? ~ReadLockFlag & ReadEna : ~WriteLockFlag & WriteEna;
assign NetNEXT=~WriteLockFlag & WriteEna & (~CoreACT | CoreCMD);
assign DRAMwren=(WriteACTReg & ((WriteMatchFlag[0] & WriteToFifoFlag[0]) | (WriteMatchFlag[1] & WriteToFifoFlag[1]) |
						(WriteMatchFlag[2] & WriteToFifoFlag[2]) | (WriteMatchFlag[3] & WriteToFifoFlag[3]))) | FifoLoadFromMem;
assign INIT=ClearReg[8];

//=================================================================================================
//				Logic
//=================================================================================================
always_comb
begin


// muxing selector
SelBus=(~CoreACT | CoreCMD) ? NetSEL : CoreSEL;
DIBus=(~CoreACT | CoreCMD) ? NetDATA : CoreDTI;

// enable read transaction
ReadEna=~ReadACTReg;
// enable write transaction
WriteEna=~WriteACTReg;
// enable memory transaction
MemEna=~MemACTReg | ~ACT | NEXT;

// internal stage 
IntEna=~IntACTReg;

// enable control transactions
CtrlEna=~CtrlACTReg | MemEna;

casez (DataReady)
	4'b0000:	DataChannel=2'd0;
	4'b???1:	DataChannel=2'd0;
	4'b??10:	DataChannel=2'd1;
	4'b?100:	DataChannel=2'd2;
	4'b1000:	DataChannel=2'd3;
	endcase

// write address bus for TRAM
TRAMWABus[5]=ReadMatchFlag[2] | ReadMatchFlag[3];
TRAMWABus[4]=ReadMatchFlag[1] | ReadMatchFlag[3];
TRAMWABus[3:0]=(TFWPtr[0] & {4{ReadMatchFlag[0]}}) | (TFWPtr[1] & {4{ReadMatchFlag[1]}}) |
				(TFWPtr[2] & {4{ReadMatchFlag[2]}}) | (TFWPtr[3] & {4{ReadMatchFlag[3]}});
// read address bus for TRAM
TRAMRABus[5:4]=DataChannel;
TRAMRABus[3:0]=(TFRPtr[0] & {4{DataReady[0]}}) | (TFRPtr[1] & {4{DataReady[1] & ~DataReady[0]}}) |
				(TFRPtr[2] & {4{DataReady[2] & ~DataReady[1] & ~DataReady[0]}}) |
				(TFRPtr[3] & {4{DataReady[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]}});

// low bits of write address for data fifo, when data loaded from memory
case (TAGI[3:2])
	2'd0:	LoadWABus=DFWPtr[0];
	2'd1:	LoadWABus=DFWPtr[1];
	2'd2:	LoadWABus=DFWPtr[2];
	2'd3:	LoadWABus=DFWPtr[3];
	endcase

// write address for data fifo when data loaded from network or core
DRAMWABus[5]=WriteMatchFlag[2] | WriteMatchFlag[3];
DRAMWABus[4]=WriteMatchFlag[1] | WriteMatchFlag[3];
DRAMWABus[3:0]=(DFWPtr[0] & {4{WriteMatchFlag[0]}}) | (DFWPtr[1] & {4{WriteMatchFlag[1]}}) |
				(DFWPtr[2] & {4{WriteMatchFlag[2]}}) | (DFWPtr[3] & {4{WriteMatchFlag[3]}});
// read address for data fifo
DRAMRABus[5:4]=(Machine==FetchState) | (Machine==WriteState) ? {CtrlSelFlag[2] | CtrlSelFlag[3],CtrlSelFlag[1] | CtrlSelFlag[3]} : DataChannel;
DRAMRABus[3:0]=(Machine==FetchState) | (Machine==WriteState) ? (DFRPtr[0] & {4{CtrlSelFlag[0]}}) | (DFRPtr[1] & {4{CtrlSelFlag[1]}}) |
										(DFRPtr[2] & {4{CtrlSelFlag[2]}}) | (DFRPtr[3] & {4{CtrlSelFlag[3]}}) : 
										(DFRPtr[0] & {4{DataReady[0]}}) | (DFRPtr[1] & {4{DataReady[1] & ~DataReady[0]}}) |
										(DFRPtr[2] & {4{DataReady[2] & ~DataReady[1] & ~DataReady[0]}}) |
										(DFRPtr[3] & {4{DataReady[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]}});
// loading data from memory to the fifo
FifoLoadFromMem=DRDY & TAGI[1];

// write to the ARAM
ARAMwren=~ClearReg[8] | IntACTReg | (Machine==StoreDescState);
// data to ARAM
ARAMInBus=(Machine==StoreDescState) ? ARAMInReg : {IntDescReg[125:116],(IntDescReg[115:96]+20'd1) & IntDescReg[75:56],IntDescReg[95:0]} | {126{~ClearReg[8]}};

// data offset for write data to memory
case (IntDescReg[125:124])
	2'd0:	IntOffsetBus={9'd0,IntDescReg[115:96]};
	2'd1:	IntOffsetBus={8'd0,IntDescReg[115:96],1'd0};
	2'd2:	IntOffsetBus={7'd0,IntDescReg[115:96],2'd0};
	2'd3:	IntOffsetBus={6'd0,IntDescReg[115:96],3'd0};
	endcase

// request to change channel
ChannelChange=ReadACTReg & ~(|ReadMatchFlag);
// request to store data from FIFO
MustWrite=(CtrlSelFlag[0] & (|ChannelCntReg[0])) | (CtrlSelFlag[1] & (|ChannelCntReg[1])) |
			(CtrlSelFlag[2] & (|ChannelCntReg[2])) | (CtrlSelFlag[3] & (|ChannelCntReg[3]));
// request to waiting uncompleted read transactions
MustWait=(CtrlSelFlag[0] & ((TFWPtr[0]!=TFRPtr[0]) | LockLoadReqFlag[0])) | (CtrlSelFlag[1] & ((TFWPtr[1]!=TFRPtr[1]) | LockLoadReqFlag[1])) |
		(CtrlSelFlag[2] & ((TFWPtr[2]!=TFRPtr[2]) | LockLoadReqFlag[2])) | (CtrlSelFlag[3] & ((TFWPtr[3]!=TFRPtr[3]) | LockLoadReqFlag[3]));
// valid descriptor in the selected channel
ChannelValid=(CtrlSelFlag[0] & ~(&ChannelDescReg[0][55:40])) | (CtrlSelFlag[1] & ~(&ChannelDescReg[1][55:40])) |
			(CtrlSelFlag[2] & ~(&ChannelDescReg[2][55:40])) | (CtrlSelFlag[3] & ~(&ChannelDescReg[3][55:40]));

// selection least used channel for replacing descriptor
LRUBus[0][4]=(ChannelLRUReg[1]<ChannelLRUReg[0]);
if (ChannelLRUReg[1]<ChannelLRUReg[0]) LRUBus[0][3:0]=ChannelLRUReg[1];
	else LRUBus[0][3:0]=ChannelLRUReg[0];
LRUBus[1][4]=(ChannelLRUReg[3]<ChannelLRUReg[2]);
if (ChannelLRUReg[3]<ChannelLRUReg[2]) LRUBus[1][3:0]=ChannelLRUReg[3];
	else LRUBus[1][3:0]=ChannelLRUReg[2];
CtrlSelBus[1]=(LRUBus[1][3:0]<LRUBus[0][3:0]);
if (LRUBus[1][3:0]<LRUBus[0][3:0]) CtrlSelBus[0]=LRUBus[1][4];
	else CtrlSelBus[0]=LRUBus[0][4];

// setup counter values
SCntBus[0]=DFWPtr[0]-DFRPtr[0]+{3'd0,WriteACTReg & WriteMatchFlag[0] & WriteToFifoFlag[0]};
SCntBus[1]=DFWPtr[1]-DFRPtr[1]+{3'd0,WriteACTReg & WriteMatchFlag[1] & WriteToFifoFlag[1]};
SCntBus[2]=DFWPtr[2]-DFRPtr[2]+{3'd0,WriteACTReg & WriteMatchFlag[2] & WriteToFifoFlag[2]};
SCntBus[3]=DFWPtr[3]-DFRPtr[3]+{3'd0,WriteACTReg & WriteMatchFlag[3] & WriteToFifoFlag[3]};
for (i=0; i<4; i=i+1) LCntBus[i]=((ChannelDescReg[i][115:96]-ChannelDescReg[i][95:76]) & ChannelDescReg[i][75:56])-{16'd0,DFWPtr[i]-DFRPtr[i]};

// offset bus from control block
case (CtrlSizeReg)
	2'd0: CtrlOffsetBus={3'd0,CtrlOffsetReg};
	2'd1: CtrlOffsetBus={2'd0,CtrlOffsetReg,1'd0};
	2'd2: CtrlOffsetBus={1'd0,CtrlOffsetReg,2'd0};
	2'd3: CtrlOffsetBus={CtrlOffsetReg,3'd0};
	endcase
// base bus for CTRL access
CtrlBaseBus=(ChannelDescReg[0][39:0] & {40{~CtrlTagReg[3] & ~CtrlTagReg[2] & ~DLoadFlag}}) | 
			(ChannelDescReg[1][39:0] & {40{~CtrlTagReg[3] & CtrlTagReg[2] & ~DLoadFlag}}) |
			(ChannelDescReg[2][39:0] & {40{CtrlTagReg[3] & ~CtrlTagReg[2] & ~DLoadFlag}}) |
			(ChannelDescReg[3][39:0] & {40{CtrlTagReg[3] & CtrlTagReg[2] & ~DLoadFlag}}) |
			(DTBASE & {40{DLoadFlag}});

// load data to fifo request
LoadReq=|LoadReqFlag;

// end of read or write operation
EndWrite=(CtrlStoreCntReg==4'd1) & CtrlEna;
// end of read operation
EndRead=(CtrlLoadCntReg==3'd1) & CtrlEna;

// prevent setup write to fifo flag
PreventSetUpWriteToFIFO=|(ModifyWhileLoadFlag & CtrlSelFlag);

end

//=================================================================================================
//				Syncronous logic by CLK
//=================================================================================================
always_ff @(negedge RESET or posedge CLK)
begin

if (!RESET) begin
				ModifyWhileLoadFlag<=0;
				ChannelSelReg[0]<={24{1'b1}};
				ChannelSelReg[1]<={24{1'b1}};
				ChannelSelReg[2]<={24{1'b1}};
				ChannelSelReg[3]<={24{1'b1}};
				ChannelCntReg[0]<=0;
				ChannelCntReg[1]<=0;
				LockLoadReqFlag<=0;
				ChannelCntReg[2]<=0;
				ChannelCntReg[3]<=0;
				WriteToFifoFlag[0]<=0;
				WriteToFifoFlag[1]<=0;
				WriteToFifoFlag[2]<=0;
				WriteToFifoFlag[3]<=0;
				ChannelLRUReg[0]<=4'd0;
				ChannelLRUReg[1]<=4'd0;
				ChannelLRUReg[2]<=4'd0;
				ChannelLRUReg[3]<=4'd0;
				IntACTReg<=1'b0;
				MachineWaitFlag<=1'b0;
				DLoadFlag<=0;
				MemACTReg<=1'b0;
				ACT<=1'b0;
				ClearReg<=9'd0;
				CtrlACTReg<=1'b0;				
				WriteLockFlag<=0;
				ReadLockFlag<=0;
				CCFlag<=0;
				SCntReg[0]<=0;
				SCntReg[1]<=0;
				SCntReg[2]<=0;
				SCntReg[3]<=0;
				ReadACTReg<=1'b0;
				ReadMatchFlag<=0;
				CoreDRDYFlag<=0;
				DataReady<=0;
				WriteACTReg<=1'b0;
				WriteMatchFlag<=0;
				DFWPtr[0]<=0;
				DFWPtr[1]<=0;
				DFWPtr[2]<=0;
				DFWPtr[3]<=0;
				AbortCntr[0]<=0;
				AbortCntr[1]<=0;
				AbortCntr[2]<=0;
				AbortCntr[3]<=0;
				DFRPtr[0]<=0;
				DFRPtr[1]<=0;
				DFRPtr[2]<=0;
				DFRPtr[3]<=0;
				TFWPtr[0]<=0;
				TFWPtr[1]<=0;
				TFWPtr[2]<=0;
				TFWPtr[3]<=0;
				TFRPtr[0]<=0;
				TFRPtr[1]<=0;
				TFRPtr[2]<=0;
				TFRPtr[3]<=0;
				TypeReg[0]<=0;
				TypeReg[1]<=0;
				TypeReg[2]<=0;
				TypeReg[3]<=0;
				TMOReg[0]<=0;
				TMOReg[1]<=0;
				TMOReg[2]<=0;
				TMOReg[3]<=0;
				TMOFlag[0]<=0;
				TMOFlag[1]<=0;
				TMOFlag[2]<=0;
				TMOFlag[3]<=0;
				ChannelDescReg[0]<={50'd0, {76{1'b1}}};
				ChannelDescReg[1]<={50'd0, {76{1'b1}}};
				ChannelDescReg[2]<={50'd0, {76{1'b1}}};
				ChannelDescReg[3]<={50'd0, {76{1'b1}}};
				Machine<=IDLEState;
				SelMissFlag<=0;
				CoreDRDY<=0;
				LoadReqFlag<=0;
				CtrlLoadCntReg<=0;
				CtrlStoreCntReg<=0;
				end

else begin
//-----------------------------------------------
// clear system
if (~ClearReg[8]) ClearReg<=ClearReg+9'd1;

//-----------------------------------------------
// 		Read channel input
ReadACTReg<=(ReadACTReg | (CoreACT & CoreCMD & ~ReadLockFlag)) & (~ReadACTReg | ((~(|ReadMatchFlag)) & ((Machine!=ResetState) | ~CCFlag)));
if (ReadEna)
	begin
	// selector
	ReadSelReg<=CoreSEL;
	// tag
	ReadTagReg<=CoreTAGI;
	// address
	ReadAddrReg<=CoreADDR;
	end
// channel match flags	
if (ReadEna | (Machine==InitChannelState))
	for (ii=0; ii<4; ii=ii+1)
		ReadMatchFlag[ii]<=CCFlag ? CtrlSelFlag[ii] : (CoreSEL==ChannelSelReg[ii]);


//-----------------------------------------------
// 		Read channel output
// Data ready signal
CoreDRDY<=CoreDRDYFlag;
CoreDRDYFlag<=|DataReady;
// data bus to core
CoreDTO<=(DRAMOutBus & {64{~DataSelReg[0] & ~DataSelReg[1] & ~DataSelReg[2] & ~DataSelReg[3]}})|
			({44'd0, ChannelCountReg[0]} & {64{DataSelReg[0]}}) | ({44'd0, ChannelCountReg[1]} & {64{DataSelReg[1]}}) |
			({44'd0, ChannelCountReg[2]} & {64{DataSelReg[2]}}) | ({44'd0, ChannelCountReg[3]} & {64{DataSelReg[3]}});
// tag to core
CoreTAGO<=TRAMOutBus;
// size to core
CoreSIZEO<=({DataAbortReg[0], CoreSizeReg[1:0] | {2{DataAbortReg[0]}}} & {3{DataReadyReg[0]}}) |
			({DataAbortReg[1], CoreSizeReg[3:2] | {2{DataAbortReg[1]}}} & {3{DataReadyReg[1] & ~DataReadyReg[0]}}) |
			({DataAbortReg[2], CoreSizeReg[5:4] | {2{DataAbortReg[2]}}} & {3{DataReadyReg[2] & ~DataReadyReg[1] & ~DataReadyReg[0]}}) |
			({DataAbortReg[3], CoreSizeReg[7:6] | {2{DataAbortReg[3]}}} & {3{DataReadyReg[3] & ~DataReadyReg[2] & ~DataReadyReg[1] & ~DataReadyReg[0]}});
DataReadyReg<=DataReady;
DataAbortReg<=DataAbort;
DataSelReg[0]<=DataReady[0] & CntrRead[0];
DataSelReg[1]<=DataReady[1] & CntrRead[1] & ~DataReady[0];
DataSelReg[2]<=DataReady[2] & CntrRead[2] & ~DataReady[1] & ~DataReady[0];
DataSelReg[3]<=DataReady[3] & CntrRead[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0];
CoreSizeReg<={(ChannelDescReg[3][125:124] | {CntrRead[3],1'b0}) & {1'b1, ~CntrRead[3]},
				(ChannelDescReg[2][125:124] | {CntrRead[2],1'b0}) & {1'b1, ~CntrRead[2]},
				(ChannelDescReg[1][125:124] | {CntrRead[1],1'b0}) & {1'b1, ~CntrRead[1]},
				(ChannelDescReg[0][125:124] | {CntrRead[0],1'b0}) & {1'b1, ~CntrRead[0]}};
// counter values
ChannelCountReg[0]<=(ChannelDescReg[0][115:96]-ChannelDescReg[0][95:76]) & ChannelDescReg[0][75:56];
ChannelCountReg[1]<=(ChannelDescReg[1][115:96]-ChannelDescReg[1][95:76]) & ChannelDescReg[1][75:56];
ChannelCountReg[2]<=(ChannelDescReg[2][115:96]-ChannelDescReg[2][95:76]) & ChannelDescReg[2][75:56];
ChannelCountReg[3]<=(ChannelDescReg[3][115:96]-ChannelDescReg[3][95:76]) & ChannelDescReg[3][75:56];


//-----------------------------------------------
//			Write channel input
WriteACTReg<=(WriteACTReg | (~WriteLockFlag & ((CoreACT & ~CoreCMD) | NetACT))) & (~WriteACTReg | (~IntEna & ~(|(WriteMatchFlag & WriteToFifoFlag))));

if (WriteEna)
	begin
	BypassFlag<=(IntSelReg[7:0]==SelBus[7:0]) & ((CoreACT & ~CoreCMD) | NetACT) & IntACTReg;
	// input data
	WriteDataReg<=DIBus;
	// input selector
	WriteSelReg<=SelBus;
	for (i1=0; i1<4; i1=i1+1)
		begin
		// channels match flags
		WriteMatchFlag[i1]<=(~CoreACT | CoreCMD) ? (NetSEL==ChannelSelReg[i1]) : (CoreSEL==ChannelSelReg[i1]);
		end
	end

//-----------------------------------------------
//		Cached streams system
// 
// write data pointers
for (i2=0; i2<4; i2=i2+1)
	if (CtrlSelFlag[i2] & (Machine==InitChannelState)) DFWPtr[i2]<=4'd0;
		else if ((WriteACTReg & WriteMatchFlag[i2] & WriteToFifoFlag[i2]) | (FifoLoadFromMem & (TAGI[3:2]==i2)) | TMOFlag[i2]) DFWPtr[i2]<=DFWPtr[i2]+4'd1;

// abort counters
if (CtrlSelFlag[0] & (Machine==InitChannelState)) AbortCntr[0]<=4'd0;
	else AbortCntr[0]<=AbortCntr[0]+{3'd0,TMOFlag[0] & (~WriteACTReg | ~WriteMatchFlag[0] | ~WriteToFifoFlag[0])}-{3'd0,DataReady[0] & (|AbortCntr[0])};
if (CtrlSelFlag[1] & (Machine==InitChannelState)) AbortCntr[1]<=4'd0;
	else AbortCntr[1]<=AbortCntr[1]+{3'd0,TMOFlag[1] & (~WriteACTReg | ~WriteMatchFlag[1] | ~WriteToFifoFlag[1])}-{3'd0,DataReady[1] & (|AbortCntr[1]) & ~DataReady[0]};
if (CtrlSelFlag[2] & (Machine==InitChannelState)) AbortCntr[2]<=4'd0;
	else AbortCntr[2]<=AbortCntr[2]+{3'd0,TMOFlag[2] & (~WriteACTReg | ~WriteMatchFlag[2] | ~WriteToFifoFlag[2])}-{3'd0,DataReady[2] & (|AbortCntr[2]) & ~DataReady[1] & ~DataReady[0]};
if (CtrlSelFlag[3] & (Machine==InitChannelState)) AbortCntr[3]<=4'd0;
	else AbortCntr[3]<=AbortCntr[3]+{3'd0,TMOFlag[3] & (~WriteACTReg | ~WriteMatchFlag[3] | ~WriteToFifoFlag[3])}-{3'd0,DataReady[3] & (|AbortCntr[3]) & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]};

		
// read data pointers
if (CtrlSelFlag[0] & (Machine==InitChannelState)) DFRPtr[0]<=4'd0;
	else DFRPtr[0]<=DFRPtr[0]+{3'd0,(Machine==WriteState) ? CtrlSelFlag[0] & CtrlEna : DataReady[0] & ~CntrRead[0]};
if (CtrlSelFlag[1] & (Machine==InitChannelState)) DFRPtr[1]<=4'd0;
	else DFRPtr[1]<=DFRPtr[1]+{3'd0,(Machine==WriteState) ? CtrlSelFlag[1] & CtrlEna : DataReady[1] & ~CntrRead[1] & ~DataReady[0]};
if (CtrlSelFlag[2] & (Machine==InitChannelState)) DFRPtr[2]<=4'd0;
	else DFRPtr[2]<=DFRPtr[2]+{3'd0,(Machine==WriteState) ? CtrlSelFlag[2] & CtrlEna : DataReady[2] & ~CntrRead[2] & ~DataReady[1] & ~DataReady[0]};
if (CtrlSelFlag[3] & (Machine==InitChannelState)) DFRPtr[3]<=4'd0;
	else DFRPtr[3]<=DFRPtr[3]+{3'd0,(Machine==WriteState) ? CtrlSelFlag[3] & CtrlEna : DataReady[3] & ~CntrRead[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]};

// write TAG fifo pointers
for (i3=0; i3<4; i3=i3+1)
	if (CtrlSelFlag[i3] & (Machine==InitChannelState)) TFWPtr[i3]<=4'd0;
		else TFWPtr[i3]<=TFWPtr[i3]+{3'd0,ReadACTReg & ReadMatchFlag[i3]};

// read TAG fifo pointers
if (CtrlSelFlag[0] & (Machine==InitChannelState)) TFRPtr[0]<=4'd0;
	else TFRPtr[0]<=TFRPtr[0]+{3'd0,DataReady[0]};
if (CtrlSelFlag[1] & (Machine==InitChannelState)) TFRPtr[1]<=4'd0;
	else TFRPtr[1]<=TFRPtr[1]+{3'd0,DataReady[1] & ~DataReady[0]};
if (CtrlSelFlag[2] & (Machine==InitChannelState)) TFRPtr[2]<=4'd0;
	else TFRPtr[2]<=TFRPtr[2]+{3'd0,DataReady[2] & ~DataReady[1] & ~DataReady[0]};
if (CtrlSelFlag[3] & (Machine==InitChannelState)) TFRPtr[3]<=4'd0;
	else TFRPtr[3]<=TFRPtr[3]+{3'd0,DataReady[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]};

// Access type registers
for (ib=0; ib<16; ib=ib+1)
	begin
	if (ReadACTReg & ReadMatchFlag[0] & (TFWPtr[0]==ib)) TypeReg[0][ib]<=ReadAddrReg;
	if (ReadACTReg & ReadMatchFlag[1] & (TFWPtr[1]==ib)) TypeReg[1][ib]<=ReadAddrReg;
	if (ReadACTReg & ReadMatchFlag[2] & (TFWPtr[2]==ib)) TypeReg[2][ib]<=ReadAddrReg;
	if (ReadACTReg & ReadMatchFlag[3] & (TFWPtr[3]==ib)) TypeReg[3][ib]<=ReadAddrReg;
	end

// TimeoutRegisters and flags
for (i4=0; i4<4; i4=i4+1)
	begin
	TMOReg[i4]<=(((TMOReg[i4]==8'd0) & (TFWPtr[i4] != TFRPtr[i4]) & TE & ~TypeReg[i4][TFRPtr[i4]]) ? ChannelDescReg[i4][123:116] : (TMOReg[i4]-{7'd0,|TMOReg[i4] & TE}) | {8{WriteACTReg & WriteMatchFlag[i4] & WriteToFifoFlag[i4] & (|TMOReg[i4])}}) & {8{~DataReady[i4]}};
	TMOFlag[i4]<=(TMOReg[i4]==8'd1) & TE & (~WriteACTReg | ~WriteMatchFlag[i4] | ~WriteToFifoFlag[i4]) &
					(~(WriteACTReg & WriteMatchFlag[i4] & (IntEna | (WriteMatchFlag[i4] & WriteToFifoFlag[i4]))));
	end

// data ready signals by channels
DataReady[0]<=(TFWPtr[0] != TFRPtr[0]) & ((DFWPtr[0] != DFRPtr[0]) | TypeReg[0][TFRPtr[0]]) & ~DataReady[0];
DataReady[1]<=(TFWPtr[1] != TFRPtr[1]) & ((DFWPtr[1] != DFRPtr[1]) | TypeReg[1][TFRPtr[1]]) & (~DataReady[1] | DataReady[0]);
DataReady[2]<=(TFWPtr[2] != TFRPtr[2]) & ((DFWPtr[2] != DFRPtr[2]) | TypeReg[2][TFRPtr[2]]) & (~DataReady[2] | DataReady[1] | DataReady[0]);
DataReady[3]<=(TFWPtr[3] != TFRPtr[3]) & ((DFWPtr[3] != DFRPtr[3]) | TypeReg[3][TFRPtr[3]]) & (~DataReady[3] | DataReady[2] | DataReady[1] | DataReady[0]);
// counter read flags
CntrRead[0]<=TypeReg[0][TFRPtr[0]];
CntrRead[1]<=TypeReg[1][TFRPtr[1]];
CntrRead[2]<=TypeReg[2][TFRPtr[2]];
CntrRead[3]<=TypeReg[3][TFRPtr[3]];
// data abort register
DataAbort[0]<=|AbortCntr[0] & ~TypeReg[0][TFRPtr[0]];
DataAbort[1]<=|AbortCntr[1] & ~TypeReg[1][TFRPtr[1]];
DataAbort[2]<=|AbortCntr[2] & ~TypeReg[2][TFRPtr[2]];
DataAbort[3]<=|AbortCntr[3] & ~TypeReg[3][TFRPtr[3]];
// disable data counters
DisableCntReg[0]<=(|AbortCntr[0]) | TypeReg[0][TFRPtr[0]];
DisableCntReg[1]<=(|AbortCntr[1]) | TypeReg[1][TFRPtr[1]];
DisableCntReg[2]<=(|AbortCntr[2]) | TypeReg[2][TFRPtr[2]];
DisableCntReg[3]<=(|AbortCntr[3]) | TypeReg[3][TFRPtr[3]];

// channel descriptor registers
// base address			40 bit 	[39:0]
// selector upper bits	16 bit	[55:40]
// cycle mask			20 bit	[75:56]
// read pointer			20 bit	[95:76]
// write pointer		20 bit	[115:96]
// channel timeout		8 bit	[123:116]
// channel size			2 bit	[125:124]
for (i5=0; i5<4; i5=i5+1)
	begin
	// [75:0]
	if (CtrlSelFlag[i5] & (StrobeDescFlag | (DSetFlag & CCFlag))) 
				ChannelDescReg[i5][75:0]<=DSetFlag ? {DescReg[59:40], ReadSelReg[23:8], DescReg[39:0]} : ARAMOutBus[75:0];
	// write pointer [115:96]
	if (CtrlSelFlag[i5] & StrobeDescFlag) ChannelDescReg[i5][115:96]<=ARAMOutBus[115:96];
		else ChannelDescReg[i5][115:96]<=(ChannelDescReg[i5][115:96]+{19'd0,WriteACTReg & WriteMatchFlag[i5] & (IntEna | (WriteMatchFlag[i5] & WriteToFifoFlag[i5]))}) &
										{20{~(DSetFlag & CCFlag & CtrlSelFlag[i5])}} & ChannelDescReg[i5][75:56];
	//[125:116]
	if (CtrlSelFlag[i5] & (StrobeDescFlag | (DSetFlag & CCFlag))) ChannelDescReg[i5][125:116]<=DSetFlag ? DescReg[69:60] : ARAMOutBus[125:116];
	end
// read pointer [95:76]
if (CtrlSelFlag[0] & StrobeDescFlag) ChannelDescReg[0][95:76]<=ARAMOutBus[95:76];
		else ChannelDescReg[0][95:76]<=(ChannelDescReg[0][95:76]+{19'd0,DataReady[0] & ~DisableCntReg[0]}) & {20{~(DSetFlag & CCFlag & CtrlSelFlag[0])}} & ChannelDescReg[0][75:56];
if (CtrlSelFlag[1] & StrobeDescFlag) ChannelDescReg[1][95:76]<=ARAMOutBus[95:76];
		else ChannelDescReg[1][95:76]<=(ChannelDescReg[1][95:76]+{19'd0,DataReady[1] & ~DisableCntReg[1] & ~DataReady[0]}) & {20{~(DSetFlag & CCFlag & CtrlSelFlag[1])}} & ChannelDescReg[1][75:56];
if (CtrlSelFlag[2] & StrobeDescFlag) ChannelDescReg[2][95:76]<=ARAMOutBus[95:76];
		else ChannelDescReg[2][95:76]<=(ChannelDescReg[2][95:76]+{19'd0,DataReady[2] & ~DisableCntReg[2] & ~DataReady[1] & ~DataReady[0]}) & 
										{20{~(DSetFlag & CCFlag & CtrlSelFlag[2])}} & ChannelDescReg[2][75:56];
if (CtrlSelFlag[3] & StrobeDescFlag) ChannelDescReg[3][95:76]<=ARAMOutBus[95:76];
		else ChannelDescReg[3][95:76]<=(ChannelDescReg[3][95:76]+{19'd0,DataReady[3] & ~DisableCntReg[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]}) & 
										{20{~(DSetFlag & CCFlag & CtrlSelFlag[3])}} & ChannelDescReg[3][75:56];
			
				
// load data to fifo request flags
LoadReqFlag[0]<=~WriteToFifoFlag[0] & ~LockLoadReqFlag[0] & ((DFWPtr[0]-DFRPtr[0])<4'd7) & (ChannelDescReg[0][95:76]!=ChannelDescReg[0][115:96]) & (|LCntBus[0]);
LoadReqFlag[1]<=~WriteToFifoFlag[1] & ~LockLoadReqFlag[1] & ((DFWPtr[1]-DFRPtr[1])<4'd7) & (ChannelDescReg[1][95:76]!=ChannelDescReg[1][115:96]) & (|LCntBus[1]);
LoadReqFlag[2]<=~WriteToFifoFlag[2] & ~LockLoadReqFlag[2] & ((DFWPtr[2]-DFRPtr[2])<4'd7) & (ChannelDescReg[2][95:76]!=ChannelDescReg[2][115:96]) & (|LCntBus[2]);
LoadReqFlag[3]<=~WriteToFifoFlag[3] & ~LockLoadReqFlag[3] & ((DFWPtr[3]-DFRPtr[3])<4'd7) & (ChannelDescReg[3][95:76]!=ChannelDescReg[3][115:96]) & (|LCntBus[3]);

// selector registers
for (i6=0; i6<4; i6=i6+1)
	if (CtrlSelFlag[i6] & (Machine==InitChannelState)) ChannelSelReg[i6]<=ReadSelReg;

// calculate unstored data in the fifo
if (CtrlSelFlag[0] & (Machine==InitChannelState)) ChannelCntReg[0]<=4'd0;
	else if (CntReloadFlag[0]) ChannelCntReg[0]<=DFWPtr[0]-DFRPtr[0]-{3'd0,DataReady[0] & ~CntrRead[0]};
		else ChannelCntReg[0]<=ChannelCntReg[0]+{3'd0,WriteACTReg & WriteToFifoFlag[0] & WriteMatchFlag[0]} - {3'd0,|ChannelCntReg[0] & DataReady[0] & ~CntrRead[0]};
if (CtrlSelFlag[1] & (Machine==InitChannelState)) ChannelCntReg[1]<=4'd0;
	else if (CntReloadFlag[1]) ChannelCntReg[1]<=DFWPtr[1]-DFRPtr[1]-{3'd0,DataReady[1] & ~CntrRead[1] & ~DataReady[0]};
		else ChannelCntReg[1]<=ChannelCntReg[1]+{3'd0,WriteACTReg & WriteToFifoFlag[1] & WriteMatchFlag[1]} - {3'd0,|ChannelCntReg[1] & DataReady[1] & ~CntrRead[1] & ~DataReady[0]};
if (CtrlSelFlag[2] & (Machine==InitChannelState)) ChannelCntReg[2]<=4'd0;
	else if (CntReloadFlag[2]) ChannelCntReg[2]<=DFWPtr[2]-DFRPtr[2]-{3'd0,DataReady[2] & ~CntrRead[2] & ~DataReady[1] & ~DataReady[0]};
		else ChannelCntReg[2]<=ChannelCntReg[2]+{3'd0,WriteACTReg & WriteToFifoFlag[2] & WriteMatchFlag[2]} - {3'd0,|ChannelCntReg[2] & DataReady[2] & ~CntrRead[2] & ~DataReady[1] & ~DataReady[0]};
if (CtrlSelFlag[3] & (Machine==InitChannelState)) ChannelCntReg[3]<=4'd0;
	else if (CntReloadFlag[3]) ChannelCntReg[3]<=DFWPtr[3]-DFRPtr[3]-{3'd0,DataReady[3] & ~CntrRead[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]};
		else ChannelCntReg[3]<=ChannelCntReg[3]+{3'd0,WriteACTReg & WriteToFifoFlag[3] & WriteMatchFlag[3]} - {3'd0,|ChannelCntReg[3] & DataReady[3] & ~CntrRead[3] & ~DataReady[2] & ~DataReady[1] & ~DataReady[0]};

// reload data counters
CntReloadFlag[0]<=DRDY & TAGI[4] & ~TAGI[3] & ~TAGI[2] & TAGI[1] & TAGI[0];
CntReloadFlag[1]<=DRDY & TAGI[4] & ~TAGI[3] & TAGI[2] & TAGI[1] & TAGI[0];
CntReloadFlag[2]<=DRDY & TAGI[4] & TAGI[3] & ~TAGI[2] & TAGI[1] & TAGI[0];
CntReloadFlag[3]<=DRDY & TAGI[4] & TAGI[3] & TAGI[2] & TAGI[1] & TAGI[0];

// enable write to fifo
for (i7=0; i7<4; i7=i7+1)
	WriteToFifoFlag[i7]<=(WriteToFifoFlag[i7] & ~((Machine==InitChannelState) & CtrlSelFlag[i7]) & ~(WriteACTReg & WriteToFifoFlag[i7] & WriteMatchFlag[i7] & ((DFWPtr[i7]-DFRPtr[i7])==4'hE))) |
						((ChannelDescReg[i7][115:96]==ChannelDescReg[i7][95:76]) & (Machine==InitChannelState) & CtrlSelFlag[i7]) | 
						(DRDY & TAGI[4] & (TAGI[3:2]==i7) & TAGI[1] & TAGI[0]);

// LRU registers
for (i8=0; i8<4; i8=i8+1)
		ChannelLRUReg[i8]<=(ChannelLRUReg[i8]-{3'd0,ReadACTReg & |ChannelLRUReg[i8] & ~ReadLockFlag}) |
								{4{ReadACTReg & ReadMatchFlag[i8]}} | {4{CtrlSelFlag[i8] & (Machine==InitChannelState)}};

//-----------------------------------------------
//		uncached write data channel
IntACTReg<=(IntACTReg  | (WriteACTReg & ~(|(WriteMatchFlag & WriteToFifoFlag)))) & (~MemEna | SelMissFlag | ~IntACTReg);

if (IntEna)
	begin
	// data register
	IntDataReg<=WriteDataReg;
	// selector
	IntSelReg<=WriteSelReg;
	end
// descriptor register
if (IntEna | DSetFlag)
	begin
	IntDescReg<=DSetFlag ? {DescReg[69:60], 40'd0, DescReg[59:40], IntSelReg[23:8], DescReg[39:0]} : (((BypassFlag ? BypassReg : ARAMOutBus) & {126{~(|WriteMatchFlag)}}) | 
				(ChannelDescReg[0] & {126{WriteMatchFlag[0]}}) | (ChannelDescReg[1] & {126{WriteMatchFlag[1]}}) |
				(ChannelDescReg[2] & {126{WriteMatchFlag[2]}}) | (ChannelDescReg[3] & {126{WriteMatchFlag[3]}}));
	// sel missmatch flag
	SelMissFlag<=(WriteSelReg[23:8]!=ARAMOutBus[55:40]) & ~DSetFlag & ~BypassFlag & ~(|WriteMatchFlag);
	end
if (IntACTReg) BypassReg<=ARAMInBus;

//-----------------------------------------------
//		control machine
if (~ClearReg[8]) Machine<=IDLEState;
	else if (~MachineWaitFlag)
			case (Machine)
			IDLEState: if (IntACTReg & SelMissFlag) Machine<=DLPrepareState;
							else if (ChannelChange) Machine<=CheckChannelState;
								else if (LoadReq) Machine<=LoadPrepareState;
									else Machine<=IDLEState;
			// load descriptor
			DLPrepareState: Machine<=DLFirstState;				// prepare pointers

			// first descriptor qword
			DLFirstState: if (CtrlEna) Machine<=DLSecondState;
								else Machine<=DLFirstState;
			// second qword
			DLSecondState: if (CtrlEna) Machine<=DLWaitState;
								else Machine<=DLSecondState;
			// waiting for descriptor completely readed
			DLWaitState: if (DSetFlag) Machine<=StackReg;
							else Machine<=DLWaitState;
			//========== setting channel
			// checking current state 
			CheckChannelState: if (MustWait) Machine<=CheckChannelState;
									else if (MustWrite) Machine<=FetchState;
										else if (ChannelValid) Machine<=StoreDescState;
											else Machine<=GetDescState;
			// if data must be downloaded from FIFO
			FetchState: Machine<=WriteState;
			WriteState: if (~CtrlEna) Machine<=WriteState;
							else if (EndWrite) Machine<=StoreDescState;
								else Machine<=FetchState;
			// store descriptor into ARAM
			StoreDescState: Machine<=GetDescState;
			// loading from the ARAM new descriptor
			GetDescState: Machine<=CheckNewState;
			// checking new channel descriptor
			CheckNewState: if (~ChannelValid) Machine<=DLFirstState;
								else Machine<=InitChannelState;
			// initialize channel
			InitChannelState: Machine<=ResetState;
			// clear all locker flags
			ResetState: Machine<=IDLEState;
			//========== uploading data into FIFO
			// prepare load operation
			LoadPrepareState: Machine<=LoadState;
			// load operation
			LoadState: if (EndRead) Machine<=EndLoadState;
							else Machine<=LoadState;
			// end of load operation
			EndLoadState: Machine<=IDLEState;
			endcase

// register for muxing descriptor from channel
ARAMInReg<=(ChannelDescReg[0] & {126{CtrlSelFlag[0]}}) | (ChannelDescReg[1] & {126{CtrlSelFlag[1]}}) |
			(ChannelDescReg[2] & {126{CtrlSelFlag[2]}}) | (ChannelDescReg[3] & {126{CtrlSelFlag[3]}});

// read descriptor form ARAM buffser
ReadDescFlag<=(Machine==StoreDescState) | (Machine==CheckChannelState);
StrobeDescFlag<=(Machine==GetDescState);

// machine stack register
if (Machine==DLPrepareState) StackReg<=ResetState;
	else if (Machine==CheckNewState) StackReg<=InitChannelState;

// machine wait flag, used for delay machine by one CLK
MachineWaitFlag<=~MachineWaitFlag & ((Machine==GetDescState) | (LoadReq & (Machine==IDLEState)));

// temporary descriptor register
if (DRDY & ~TAGI[1] & ~TAGI[0]) DescReg[39:0]<=DTI[39:0];
if (DRDY & ~TAGI[1] & TAGI[0]) DescReg[69:40]<={DTI[63:62],DTI[39:32],DTI[19:0]};
// descriptor set flag
DSetFlag<=DRDY & ~TAGI[1] & TAGI[0];

// channel selection flags/ channel will be used for allocate new stream
if ((Machine==IDLEState) | ((Machine==LoadPrepareState) & MachineWaitFlag))
	begin
	CtrlSelFlag[0]<=(Machine==LoadPrepareState) ? LoadReqFlag[0] : ~CtrlSelBus[1] & ~CtrlSelBus[0];
	CtrlSelFlag[1]<=(Machine==LoadPrepareState) ? LoadReqFlag[1] & ~LoadReqFlag[0] : ~CtrlSelBus[1] & CtrlSelBus[0];
	CtrlSelFlag[2]<=(Machine==LoadPrepareState) ? LoadReqFlag[2] & ~LoadReqFlag[1] & ~LoadReqFlag[0] : CtrlSelBus[1] & ~CtrlSelBus[0];
	CtrlSelFlag[3]<=(Machine==LoadPrepareState) ? LoadReqFlag[3] & ~LoadReqFlag[2] & ~LoadReqFlag[1] & ~LoadReqFlag[0] : CtrlSelBus[1] & CtrlSelBus[0];
	end
// channel change flag
CCFlag<=(CCFlag | (Machine==CheckChannelState)) & (Machine!=ResetState);

// data counter for store operations
CtrlStoreCntReg<=(Machine==CheckChannelState) ? 
		((SCntReg[0] & {4{CtrlSelFlag[0]}}) | (SCntReg[1] & {4{CtrlSelFlag[1]}}) | (SCntReg[2] & {4{CtrlSelFlag[2]}}) | (SCntReg[3] & {4{CtrlSelFlag[3]}})) : 
		CtrlStoreCntReg-{3'd0,|CtrlStoreCntReg & (Machine==WriteState) & CtrlEna};
for (i9=0; i9<4; i9=i9+1) SCntReg[i9]<=SCntBus[i9][3:0];
// counter for load operations
CtrlLoadCntReg<=(Machine==LoadPrepareState) ? (LCntReg[0] & {3{CtrlSelFlag[0]}}) | (LCntReg[1] & {3{CtrlSelFlag[1]}}) |
					(LCntReg[2] & {3{CtrlSelFlag[2]}}) | (LCntReg[3] & {3{CtrlSelFlag[3]}}) : CtrlLoadCntReg-{2'd0,|CtrlLoadCntReg & (Machine==LoadState) & CtrlEna};
// values for read operation
for (ia=0; ia<4; ia=ia+1) LCntReg[ia]<=LCntBus[ia][2:0] | {3{|LCntBus[ia][19:3]}};

// descriptor load flag
DLoadFlag<=(DLoadFlag | (Machine==DLPrepareState) | (Machine==CheckNewState)) & (Machine!=ResetState);

// locking load requests
LockLoadReqFlag[0]<=(LockLoadReqFlag[0] | ((Machine==LoadState) & CtrlSelFlag[0])) & (~DRDY | ~TAGI[4] | TAGI[3] | TAGI[2]);
LockLoadReqFlag[1]<=(LockLoadReqFlag[1] | ((Machine==LoadState) & CtrlSelFlag[1])) & (~DRDY | ~TAGI[4] | TAGI[3] | ~TAGI[2]);
LockLoadReqFlag[2]<=(LockLoadReqFlag[2] | ((Machine==LoadState) & CtrlSelFlag[2])) & (~DRDY | ~TAGI[4] | ~TAGI[3] | TAGI[2]);
LockLoadReqFlag[3]<=(LockLoadReqFlag[3] | ((Machine==LoadState) & CtrlSelFlag[3])) & (~DRDY | ~TAGI[4] | ~TAGI[3] | ~TAGI[2]);

// locking read transactions
ReadLockFlag<=(ReadLockFlag | (ReadEna & CoreACT & CoreCMD & (CoreSEL != ChannelSelReg[0]) & (CoreSEL != ChannelSelReg[1]) &
								(CoreSEL != ChannelSelReg[2]) & (CoreSEL != ChannelSelReg[3]))) & (Machine != ResetState);
// locking write requests
WriteLockFlag<=(WriteLockFlag | ((Machine==IDLEState) & LoadReq) | 
								((Machine==CheckChannelState) & ~MustWait) |
								(ReadEna & CoreACT & CoreCMD & (CoreSEL != ChannelSelReg[0]) & (CoreSEL != ChannelSelReg[1]) &
								(CoreSEL != ChannelSelReg[2]) & (CoreSEL != ChannelSelReg[3]))) & (Machine!=ResetState) & 
				(~DRDY | ~TAGI[4] | (Machine!=IDLEState));

// command for memory access
CtrlCmdReg<=(CtrlCmdReg | (Machine==DLPrepareState) | (Machine==LoadPrepareState) | (Machine==DLFirstState)) & (Machine!=CheckChannelState);

// offset register for load data operations
if ((Machine==IDLEState) | (Machine==CheckChannelState))
	begin
	LoadOffsetReg<=(ChannelDescReg[0][95:76]+{16'd0,DFWPtr[0]-DFRPtr[0]} & {20{LoadReqFlag[0]}} & ChannelDescReg[0][75:56]) |
					(ChannelDescReg[1][95:76]+{16'd0,DFWPtr[1]-DFRPtr[1]} & {20{LoadReqFlag[1] & ~LoadReqFlag[0]}} & ChannelDescReg[1][75:56]) |
					(ChannelDescReg[2][95:76]+{16'd0,DFWPtr[2]-DFRPtr[2]} & {20{LoadReqFlag[2] & ~LoadReqFlag[1] & ~LoadReqFlag[0]}} & ChannelDescReg[2][75:56]) |
					(ChannelDescReg[3][95:76]+{16'd0,DFWPtr[3]-DFRPtr[3]} & {20{LoadReqFlag[3] & ~LoadReqFlag[2] & ~LoadReqFlag[1] & ~LoadReqFlag[0]}} & ChannelDescReg[3][75:56]);
	CtrlMaskReg<=(ChannelDescReg[0][75:56] & {20{LoadReqFlag[0]}}) | 
				({20{LoadReqFlag[1] & ~LoadReqFlag[0]}} & ChannelDescReg[1][75:56]) |
				({20{LoadReqFlag[2] & ~LoadReqFlag[1] & ~LoadReqFlag[0]}} & ChannelDescReg[2][75:56]) | 
				({20{LoadReqFlag[3] & ~LoadReqFlag[2] & ~LoadReqFlag[1] & ~LoadReqFlag[0]}} & ChannelDescReg[3][75:56]) | {20{Machine==CheckChannelState}};
	StoreMaskReg<=(ChannelDescReg[0][75:56] & {20{CtrlSelFlag[0]}}) | (ChannelDescReg[1][75:56] & {20{CtrlSelFlag[1]}}) |
				(ChannelDescReg[2][75:56] & {20{CtrlSelFlag[2]}}) | (ChannelDescReg[3][75:56] & {20{CtrlSelFlag[3]}}) | {20{Machine==IDLEState}};
	end

// offset register for store operations
StoreOffsetReg<=(ChannelDescReg[0][95:76] & {20{CtrlSelFlag[0]}}) | (ChannelDescReg[1][95:76] & {20{CtrlSelFlag[1]}}) |
				(ChannelDescReg[2][95:76] & {20{CtrlSelFlag[2]}}) | (ChannelDescReg[3][95:76] & {20{CtrlSelFlag[3]}});

// offset register
if (Machine==DLPrepareState) CtrlOffsetReg<={IntSelReg,2'd0};
	else if (Machine==CheckChannelState) CtrlOffsetReg<={6'd0,StoreOffsetReg};
		else if (Machine==CheckNewState) CtrlOffsetReg<={ReadSelReg,2'd0};
			else if (Machine==LoadPrepareState) CtrlOffsetReg<={6'd0,LoadOffsetReg};
				else if (CtrlACTReg & MemEna) CtrlOffsetReg<=(CtrlOffsetReg+26'd1) & ({6'd0,CtrlMaskReg & StoreMaskReg} | {26{DLoadFlag}});

// write pointer
if (Machine==LoadPrepareState)
	CtrlWPtrReg<=((ChannelDescReg[0][115:96] & {20{CtrlSelFlag[0]}}) | (ChannelDescReg[1][115:96] & {20{CtrlSelFlag[1]}}) |
				(ChannelDescReg[2][115:96] & {20{CtrlSelFlag[2]}}) | (ChannelDescReg[3][115:96] & {20{CtrlSelFlag[3]}}))-20'd2;

// transaction to memory
CtrlACTReg<=(CtrlACTReg & !MemEna) | (Machine==DLFirstState) | (Machine==DLSecondState) | (Machine==WriteState) | (Machine==LoadState);
if (CtrlEna)
	begin
	CtrlDataReg<=DRAMOutBus;
	CtrlSizeReg<=(ChannelDescReg[0][125:124] & {2{CtrlSelFlag[0]}}) | (ChannelDescReg[1][125:124] & {2{CtrlSelFlag[1]}}) |
				(ChannelDescReg[2][125:124] & {2{CtrlSelFlag[2]}}) | (ChannelDescReg[3][125:124] & {2{CtrlSelFlag[3]}}) | 
				{2{(Machine==DLFirstState) | (Machine==DLSecondState)}};
	CtrlTagReg[0]<=(Machine==DLSecondState) | (((CtrlWPtrReg==CtrlOffsetReg[19:0]) | OneCountFlag) & (CtrlLoadCntReg==3'd1) & ~PreventSetUpWriteToFIFO);
	CtrlTagReg[1]<=(Machine==LoadState);
	CtrlTagReg[2]<=CtrlSelFlag[1] | CtrlSelFlag[3];
	CtrlTagReg[3]<=CtrlSelFlag[2] | CtrlSelFlag[3];
	CtrlTagReg[4]<=CtrlCmdReg ? (CtrlLoadCntReg==3'd1) : (CtrlStoreCntReg==4'd1);
	end
OneCountFlag<=((ChannelCountReg[0]==20'd1) & CtrlSelFlag[0]) | ((ChannelCountReg[1]==20'd1) & CtrlSelFlag[1]) |
			((ChannelCountReg[2]==20'd1) & CtrlSelFlag[2]) | ((ChannelCountReg[3]==20'd1) & CtrlSelFlag[3]);
// flags set if data was wrote to the menory when controller performs load operation
ModifyWhileLoadFlag[0]<=(ModifyWhileLoadFlag[0] | (WriteACTReg & WriteMatchFlag[0] & IntEna & !WriteToFifoFlag[0])) & (Machine!=IDLEState);
ModifyWhileLoadFlag[1]<=(ModifyWhileLoadFlag[1] | (WriteACTReg & WriteMatchFlag[1] & IntEna & !WriteToFifoFlag[1])) & (Machine!=IDLEState);
ModifyWhileLoadFlag[2]<=(ModifyWhileLoadFlag[2] | (WriteACTReg & WriteMatchFlag[2] & IntEna & !WriteToFifoFlag[2])) & (Machine!=IDLEState);
ModifyWhileLoadFlag[3]<=(ModifyWhileLoadFlag[3] | (WriteACTReg & WriteMatchFlag[3] & IntEna & !WriteToFifoFlag[3])) & (Machine!=IDLEState);

//-----------------------------------------------
// Memory interface
MemACTReg<=(MemACTReg & ACT & ~NEXT) | (IntACTReg & ~SelMissFlag) | CtrlACTReg;
if (MemEna)
	begin
	// offset
	MemOffsetReg<=CtrlACTReg ? CtrlOffsetBus : IntOffsetBus;
	// base address
	MemBaseReg<=CtrlACTReg ? CtrlBaseBus : IntDescReg[39:0];
	// data
	MemDataReg<=CtrlACTReg ? CtrlDataReg : IntDataReg;
	// command
	MemCmdReg<=CtrlACTReg & CtrlCmdReg;
	// size
	MemSizeReg<=CtrlACTReg ? CtrlSizeReg : IntDescReg[125:124];
	// tag
	MemTagReg<=CtrlTagReg;
	end

// output memory stage
ACT<=(ACT & ~NEXT) | MemACTReg;
if (~ACT | NEXT)
	begin
	// command
	CMD<=MemCmdReg;
	// address
	ADDR<={MemBaseReg+{16'd0,MemOffsetReg[28:5]},MemOffsetReg[4:0]};
	// operand size
	SIZEO<=MemSizeReg;
	// data to memory
	DTO<=MemDataReg;
	// transaction tag
	TAGO<=MemTagReg;
	end
end
end

endmodule
