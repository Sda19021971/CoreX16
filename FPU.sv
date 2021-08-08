module FPU (
	// global
	input wire CLK, RESETn, TCK,
	input wire [39:0] DTBASE,
	input wire [23:0] DTLIMIT,
	input wire [7:0] CPUNUM,
	input wire [15:0] TASKID,
	input wire [23:0] CPSR,
	output wire INIT,
	// interface from core
	output wire CORENEXT,
	input wire COREACT, CORECMD,
	input wire [1:0] CORESIZE, CORECPL,
	input wire [36:0] COREOFFSET,
	input wire [31:0] CORESEL,
	input wire [63:0] COREDTO,
	input wire [7:0] CORETAGO,
	output reg COREDRDY,
	output reg [63:0] COREDTI,
	output reg [7:0] CORETAGI,
	output reg [2:0] CORESIZEI,
	// local memory interface
	input wire NEXT, SNEXT,
	output reg ACT, SACT, CMD,
	output reg [1:0] SIZEO,
	output reg [44:0] ADDR,
	output reg [63:0] DTO,
	output reg [12:0] TAGO,
	output reg [23:0] SEL,
	input wire DRDY,
	input wire [63:0] DTI,
	input wire [1:0] SIZEI,
	input wire [12:0] TAGI,
	// interface to the routing system
	input wire STONEXT,
	output reg STONEW,
	output reg STOVAL,
	output reg [63:0] STODT,
	output wire STINEXT,
	input wire STIVAL,
	input wire STINEW,
	input wire [63:0] STIDT,
	// message system interface (request channel)
	output reg MSGREQ,
	input wire MSGRD,
	output reg [121:0] MSGDATA,
	// messages from Messenger
	output reg MSGNEXT,
	input wire MSGSEND,
	input wire MSGTYPE,
	input wire [79:0] MSGPARAM,
	input wire [4:0] MSGSTAT,
	// error report interface
	output reg ESTB,
	output reg [12:0] ECD
	);

integer i;

reg [63:0] STODTReg;
// resources for access error processing
logic ErrorFifoEmptyWire;
logic [15:0] ErrorFifoBus;
logic [63:0] ErrorValue;
reg ErrorACTReg, ReturnErrorChannelFlag;
// resources for output message and message acknowledge to another processor
reg [79:0] MsgOutParamReg;			// {ParamSelector, ProcIndex, PSO}
reg [4:0] MsgOutStatReg;
reg [1:0] MsgOutCntReg;
reg MsgOutTypeFlag, MessageChannelFlag;
logic MsgOutEndWire;
logic [63:0] MessageOutValue [2:0];

logic [77:0] DataFifoBus;
logic [63:0] ReadDataValue [1:0];
logic DataFifoEmptyWire;
reg ReadACTReg, ReadSecondFlag, ReturnDataChannelFlag;

logic [31:0] MsgFifoBus, MSGDATABus;
reg [1:0] MsgCntReg;
reg [2:0] MsgOutCReg;
reg MsgACTReg;
logic MessageWire, MSGRDWire, MSGREQWire;
logic [7:0] MsgFIFOUsedBus;

// core transaction input
logic CoreInEnaWire;
reg	CoreInACTReg, CoreInCmdReg;
reg [1:0] CoreInSizeReg, CoreInCPLReg;
reg [31:0] CoreInSelReg;
reg [36:0] CoreInOffsetReg;
reg [63:0] CoreInDataReg;
reg [7:0] CoreInTagReg;
reg [15:0] CoreInTaskReg;

// core cache stage
logic CoreRAMEnaWire;
reg [87:0] CoreRAMReg;
reg CoreRAMACTReg, CoreRAMCmdReg;
reg [1:0] CoreRAMSizeReg, CoreRAMCPLReg;
reg	[36:0] CoreRAMOffsetReg;
reg [31:0] CoreRAMSelReg;
reg [63:0] CoreRAMDataReg;
reg [15:0] CoreRAMTaskReg;

// core transaction output stage
logic CoreOutEndWire, CoreOutEnaWire;
reg CoreOutACTReg, CoreOutMatchReg, CoreOutCmdReg;
reg [1:0] CoreOutCntReg, CoreOutCntBus, CoreOutSizeReg, CoreOutPLReg, MsgOutCntBus;
reg [31:0] CoreOutSelReg;
reg [15:0] CoreOutDispReg, CoreOutTaskReg;
reg [63:0] CoreOutDataReg;
reg [36:0] CoreOutOffsetReg;
reg [87:0] CoreInOutBypassReg;
logic [63:0] CoreOutValue [2:0];

// channel selection resources
reg DataChannelSelWire;

logic [87:0] MasterRAMBus;
logic CoreMatchFlag, CoreOffsetFlag;
logic [36:0] CoreDisplacement;

reg FifoWriteFlag, DrdyACTReg;
logic [7:0] ReadFifoBus;
logic DataRetryWire, ErrorRetryWire;
		
logic [11:0] LRUBus, LRUNewValue;
logic [7:0] LRUTemp;
logic [1:0] LRUWriteIndex;
reg [7:0] ClearCacheReg;

// TRAM input stage resources
logic InEndWire, InDataWire, InEnaWire;
reg InACTReg, InEndFlag;
reg [1:0] InCntReg;

// TRAM output stage resources
logic TRamEnaWire;
reg TRamACTReg, TRamOutCmdReg;
reg [36:0] TRamOutOffsetReg;
reg [23:0] TRamOutSelReg;
reg [1:0] TRamOutSizeReg, TRamOutCPLReg;
reg [63:0] TRamOutDataReg;
reg [11:0] TRamOutTagReg;
reg [15:0] TRamOutDispReg, TRamOutTaskReg;

reg [175:0] TRamInReg;
logic [79:0] TRAMBus;

// selector cache input stage registers
logic SCInEnaWire;
reg SCInACTReg, SCInCMDReg;
reg [36:0] SCInOffsetReg;
reg [23:0] SCInSelReg;
reg [63:0] SCInDataReg;
reg [15:0] SCInTaskReg;
reg [1:0] SCInPLReg, SCInSizeReg;
reg [11:0] SCInTagReg;

// selector cache output stage resources
logic SCRamEnaWire;
reg SCRamACTReg, SCRamCMDReg;
reg [36:0] SCRamOffsetReg;
reg [23:0] SCRamSelReg;
reg [63:0] SCRamDataReg;
reg [15:0] SCRamTaskReg;
reg [1:0] SCRamPLReg, SCRamSizeReg;
reg [11:0] SCRamTagReg, SCRamLRUReg;
reg [3:0] SCRamMatchReg;

reg [175:0] DescReg;		// temporary register
logic [175:0] DRAMBus;		// descriptor buffer output
reg DescLoadFlag, RestartFlag, DescCountFlag;
reg [2:0] DescCntReg;
reg [23:0] SelectorReg;
reg [6:0] DescIndexReg;

// descriptor RAM input stage resources
logic StartEnaWire;
reg StartACTReg, StartCmdReg, StartNewSelFlag, StartReloadFlag;
reg [36:0] StartOffsetReg;
reg [1:0] StartSizeReg, StartCPLReg;
reg [63:0] StartDataReg;
reg [11:0] StartTagReg;
logic [6:0] StartIndexBus;
reg [6:0] StartIndexReg;
reg [23:0] StartSelReg;
reg [15:0] StartTaskReg;

// Descriptor RAM output stage resources
logic RamEnaWire;
reg [39:0] RamBaseReg;
reg [15:0] RamTaskIDReg, RamTaskReg;
reg [7:0] RamCtrlReg;
reg [23:0] RamLLSReg, RamULSReg, RamSelReg;
reg	[31:0] RamLLReg, RamULReg;
reg [36:0] RamOffsetReg;
reg RamACTReg, RamCmdReg, RamStreamReg, RamNewSelFlag, RamReloadFlag;
reg [63:0] RamDataReg;
reg [11:0] RamTagReg;
reg [1:0] RamSizeReg, RamCPLReg;
reg [6:0] RamIndexReg;

// physical address adder stage registers
logic AdderEnaWire;
reg AdderACTReg, AdderCMDReg, AdderSrcCMDReg;
reg [36:0] AdderOffsetReg, AdderSourceOffsetReg;
reg [39:0] AdderBaseReg;
reg [63:0] AdderDataReg;
reg [12:0] AdderTagReg;
reg [15:0] AdderTaskReg;
reg [11:0] AdderSrcTagReg;
reg [23:0] AdderSelReg, AdderLLSReg, AdderULSReg;
reg LLErrorFlag, ULErrorFlag, ARErrorFlag, LLSelFlag, ULSelFlag, AdderNewSelFlag, AdderStreamFlag, SelErrorFlag, PLErrorFlag;
reg TIDErrorFlag, INVDErrorFlag, NetErrorFlag;
reg [6:0] AdderIndexReg;
reg [1:0] AdderSizeReg, AdderSrcSizeReg, AdderPLReg;

// transaction output stage resources
logic OutEnaWire;

logic [79:0] SCDataBus;

logic [3:0] SkipTag;
logic SkipFlag;
reg SkipReg;


//=================================================================================================
//		Modules
//=================================================================================================

sc_fifo	ErrorFIFO(.rdreq(ErrorACTReg & ReturnErrorChannelFlag & STONEXT), .clock(CLK), 
				.wrreq((AdderACTReg & ((LLErrorFlag & ~LLSelFlag) | (ULErrorFlag & ~ULSelFlag) | ARErrorFlag | PLErrorFlag | TIDErrorFlag | INVDErrorFlag | NetErrorFlag)) | SelErrorFlag), 
				.data({AdderSrcCMDReg,TIDErrorFlag | INVDErrorFlag | NetErrorFlag,ARErrorFlag | PLErrorFlag | NetErrorFlag,INVDErrorFlag | (LLErrorFlag & ~LLSelFlag) | (ULErrorFlag & ~ULSelFlag) | PLErrorFlag,AdderTagReg[3:0],AdderTagReg[11:4]}),
				.empty(ErrorFifoEmptyWire), .q(ErrorFifoBus), .sclr(~RESETn));
defparam ErrorFIFO.LPM_WIDTH=16, ErrorFIFO.LPM_NUMWORDS=32, ErrorFIFO.LPM_WIDTHU=5;

// fifo for readed from local memory data
sc_fifo	DataFIFO(.rdreq(ReturnDataChannelFlag & ((ReadACTReg & (~DataFifoBus[77] | ~DataFifoBus[76])) | ReadSecondFlag) & STONEXT), .clock(CLK), 
				.wrreq(DRDY & ~TAGI[12]), .data({SIZEI,TAGI[11:0],DTI}), .empty(DataFifoEmptyWire), .q(DataFifoBus), .sclr(~RESETn));
defparam DataFIFO.LPM_WIDTH=78, DataFIFO.LPM_WIDTHU=5, DataFIFO.LPM_NUMWORDS=32;

// fifo for message system
sc_fifo	MsgFIFO(.rdreq(MSGRDWire), .sclr(~RESETn), .clock(CLK), .wrreq(MsgACTReg), .data(MsgFifoBus), .q(MSGDATABus), .usedw(MsgFIFOUsedBus));
defparam MsgFIFO.LPM_NUMWORDS=256, MsgFIFO.LPM_WIDTH=32, MsgFIFO.LPM_WIDTHU=8;


// master transaction cache system
// 39:0 - offset
// 55:40 - TASK ID
// 87:56 - SEL
FpuMasterRAM MasterRAM (.CLK(CLK), .RDEN(CoreInEnaWire), .WREN((CoreInACTReg & CoreRAMEnaWire) | ~ClearCacheReg[7]), .RADDR(CORESEL[3:0]),
						.WADDR(ClearCacheReg[7] ? CoreInSelReg[3:0] : ClearCacheReg[3:0]),
						.DI({CoreInSelReg,CoreInTaskReg,3'd0,CoreInOffsetReg} & {88{ClearCacheReg[7]}}),
						.DO(MasterRAMBus));
defparam MasterRAM.Mode=0, MasterRAM.Width=88;


// multichannel fifo for read data transactions
MultichannelFIFO ReadFifo(.CLK(CLK), .RESET(RESETn), .TCK(TCK), .WCS(CoreInSelReg[3:0]), .WFULL(), .WEMPTY(), .WR(FifoWriteFlag), .DI(CoreInTagReg),
							.RCS((SkipReg | SkipFlag) ? SkipTag : TRamInReg[27:24]), .RFULL(), .REMPTY(), .RD(DrdyACTReg), .DO(ReadFifoBus), .STT(SkipTag), .SKIP(SkipFlag));

defparam ReadFifo.Channels=16, ReadFifo.Width=8, ReadFifo.Depth=16;

// slave transaction cache system
// 39:0 - offset
// 55:40 - TASKID
// 79:56 - selector
altsyncram	TRAM(.clocken0(1'b1), .wren_a(InACTReg & (TRamInReg[18:17]==2'b0)), .clock0(CLK), .wren_b(TRamACTReg), .byteena_b(10'b0000011111),
					.address_a({TRamInReg[15:8],TRamInReg[27:24]}), .address_b(TRamOutTagReg), .data_a({TRamInReg[71:48],TRamInReg[47:32],3'd0,TRamInReg[108:72]}),
					.data_b({43'd0,TRamOutOffsetReg+{{21{TRamOutDispReg[15]}},TRamOutDispReg}}), .q_a(TRAMBus),
					.q_b(), .aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .byteena_a(1'b1), .clock1(1'b1), .clocken1(1'b1), .clocken2(1'b1),
					.clocken3(1'b1), .eccstatus(), .rden_a(1'b1), .rden_b(1'b1));
	defparam
		TRAM.address_reg_b = "CLOCK0",
		TRAM.byteena_reg_b = "CLOCK0",
		TRAM.byte_size = 8,
		TRAM.clock_enable_input_a = "NORMAL",
		TRAM.clock_enable_input_b = "NORMAL",
		TRAM.clock_enable_output_a = "BYPASS",
		TRAM.clock_enable_output_b = "BYPASS",
		TRAM.init_file = "TRAM.mif",
		TRAM.indata_reg_b = "CLOCK0",
		TRAM.lpm_type = "altsyncram",
		TRAM.numwords_a = 4096,
		TRAM.numwords_b = 4096,
		TRAM.operation_mode = "BIDIR_DUAL_PORT",
		TRAM.outdata_aclr_a = "NONE",
		TRAM.outdata_aclr_b = "NONE",
		TRAM.outdata_reg_a = "UNREGISTERED",
		TRAM.outdata_reg_b = "UNREGISTERED",
		TRAM.power_up_uninitialized = "FALSE",
		TRAM.read_during_write_mode_mixed_ports = "DONT_CARE",
		TRAM.widthad_a = 12,
		TRAM.widthad_b = 12,
		TRAM.width_a = 80,
		TRAM.width_b = 80,
		TRAM.width_byteena_a = 1,
		TRAM.width_byteena_b = 10,
		TRAM.wrcontrol_wraddress_reg_b = "CLOCK0";

// selector cache RAM
altsyncram	SCDataRAM(.wren_a((~(|SCRamMatchReg) & SCRamACTReg) | ~ClearCacheReg[7]), .clock0(CLK), .address_a(ClearCacheReg[7] ? {SCRamSelReg[4:0], LRUWriteIndex} : ClearCacheReg[6:0]), 
				.address_b(TRamOutSelReg[4:0]), .rden_b(SCInEnaWire), .data_a({1'b1,SCRamSelReg[23:5]} & {20{ClearCacheReg[7]}}), .q_b(SCDataBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1), .clocken0(1'b1),
				.clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1), .data_b({80{1'b1}}), .eccstatus(), .q_a(), .rden_a(1'b1), .wren_b(1'b0));
	defparam
		SCDataRAM.address_reg_b = "CLOCK0",
		SCDataRAM.clock_enable_input_a = "BYPASS",
		SCDataRAM.clock_enable_input_b = "BYPASS",
		SCDataRAM.clock_enable_output_a = "BYPASS",
		SCDataRAM.clock_enable_output_b = "BYPASS",
		SCDataRAM.init_file = "Cache.mif",
		SCDataRAM.lpm_type = "altsyncram",
		SCDataRAM.numwords_a = 128,
		SCDataRAM.numwords_b = 32,
		SCDataRAM.operation_mode = "DUAL_PORT",
		SCDataRAM.outdata_aclr_b = "NONE",
		SCDataRAM.outdata_reg_b = "UNREGISTERED",
		SCDataRAM.power_up_uninitialized = "FALSE",
		SCDataRAM.rdcontrol_reg_b = "CLOCK0",
		SCDataRAM.read_during_write_mode_mixed_ports = "DONT_CARE",
		SCDataRAM.widthad_a = 7,
		SCDataRAM.widthad_b = 5,
		SCDataRAM.width_a = 20,
		SCDataRAM.width_b = 80,
		SCDataRAM.width_byteena_a = 1;

// cache LRU system
altsyncram	LRURam(.wren_a(~ClearCacheReg[7] | SCRamACTReg), .clock0 (CLK), .address_a(ClearCacheReg[7] ? SCRamSelReg[4:0] : ClearCacheReg[4:0]),
				.address_b(TRamOutSelReg[4:0]), .rden_b(SCInEnaWire), .data_a(LRUNewValue & {12{ClearCacheReg[7]}}), .q_b(LRUBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1), .clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1),
				.clocken3 (1'b1), .data_b({12{1'b1}}), .eccstatus(), .q_a(), .rden_a(1'b1), .wren_b(1'b0));
	defparam
		LRURam.address_reg_b = "CLOCK0",
		LRURam.clock_enable_input_a = "BYPASS",
		LRURam.clock_enable_input_b = "BYPASS",
		LRURam.clock_enable_output_a = "BYPASS",
		LRURam.clock_enable_output_b = "BYPASS",
		LRURam.init_file = "LRU.mif",
		LRURam.lpm_type = "altsyncram",
		LRURam.numwords_a = 32,
		LRURam.numwords_b = 32,
		LRURam.operation_mode = "DUAL_PORT",
		LRURam.outdata_aclr_b = "NONE",
		LRURam.outdata_reg_b = "UNREGISTERED",
		LRURam.power_up_uninitialized = "FALSE",
		LRURam.rdcontrol_reg_b = "CLOCK0",
		LRURam.read_during_write_mode_mixed_ports = "DONT_CARE",
		LRURam.widthad_a = 5,
		LRURam.widthad_b = 5,
		LRURam.width_a = 12,
		LRURam.width_b = 12,
		LRURam.width_byteena_a = 1;

// descriptor RAM
altsyncram	DRAM (.wren_a((TAGI[12] & TAGI[1] & ~TAGI[0] & DRDY)), .clock0(CLK), .address_a(TAGI[10:4]), 
				.address_b(StartReloadFlag ? StartIndexReg:StartIndexBus), 
				.rden_b(StartEnaWire | StartReloadFlag), .data_a({DTI,DescReg[111:0]}), .q_b(DRAMBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1),
				.clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1), .data_b({176{1'b1}}), .eccstatus(), .q_a(), .rden_a(1'b1), .wren_b(1'b0));
	defparam
		DRAM.address_reg_b = "CLOCK0",
		DRAM.clock_enable_input_a = "BYPASS",
		DRAM.clock_enable_input_b = "BYPASS",
		DRAM.clock_enable_output_a = "BYPASS",
		DRAM.clock_enable_output_b = "BYPASS",
		DRAM.init_file = "DRAM.mif",
		DRAM.lpm_type = "altsyncram",
		DRAM.numwords_a = 128,
		DRAM.numwords_b = 128,
		DRAM.operation_mode = "DUAL_PORT",
		DRAM.outdata_aclr_b = "NONE",
		DRAM.outdata_reg_b = "UNREGISTERED",
		DRAM.power_up_uninitialized = "FALSE",
		DRAM.rdcontrol_reg_b = "CLOCK0",
		DRAM.read_during_write_mode_mixed_ports = "DONT_CARE",
		DRAM.widthad_a = 7,
		DRAM.widthad_b = 7,
		DRAM.width_a = 176,
		DRAM.width_b = 176,
		DRAM.width_byteena_a = 1;


//=================================================================================================
//		Assignments
//=================================================================================================

assign STINEXT=~InACTReg & ~InCntReg[1] & ~InCntReg[0] & ~MsgACTReg;
assign CORENEXT=CoreInEnaWire;
assign INIT=ClearCacheReg[7];
assign STODT=STODTReg;
assign MSGREQWire=(MsgFIFOUsedBus>=8'd4);

//=================================================================================================
//		Asynchronous
//=================================================================================================
always_comb
begin

// reading message FIFO
MSGRDWire=(MsgOutCReg!=5) & (MsgOutCReg!=3'd0);

// multiplexing message data on the message FIFO input
// 0 - CPL and target PSO selector
// 1 - Task ID and Proc Indx
// 2 - Message parameter
// 3 - source PSO selector
case (MsgCntReg)
	2'd0: MsgFifoBus={TRamInReg[21:20],6'd0,TRamInReg[71:48]};
	2'd1: MsgFifoBus={TRamInReg[47:32],TRamInReg[87:72]};
	2'd2: MsgFifoBus=TRamInReg[119:88];
	2'd3: MsgFifoBus={TRamInReg[15:8],TRamInReg[143:120]};
	endcase

// checking offset range, selector and task ID
// 39:0 - offset
// 55:40 - TASK ID
// 87:56 - SEL
CoreDisplacement=CoreRAMOffsetReg - CoreRAMReg[36:0];
CoreOffsetFlag=~(|(CoreDisplacement[36:15] ^ {22{CoreDisplacement[36]}}));
CoreMatchFlag=(CoreRAMTaskReg==CoreRAMReg[55:40]) & (CoreRAMSelReg==CoreRAMReg[87:56]) & CoreOffsetFlag;

// end of transaction flag
CoreOutEndWire=((CoreOutCntReg==2'd0) & CoreOutMatchReg & (CoreOutCmdReg | ~CoreOutSizeReg[1])) |
				((CoreOutCntReg==2'd1) & (CoreOutMatchReg ? ~CoreOutCmdReg & CoreOutSizeReg[1]:CoreOutCmdReg | (~CoreOutCmdReg & ~CoreOutSizeReg[1]))) |
				(CoreOutCntReg==2'd2);

// end of message transfer
MsgOutEndWire=~MsgOutCntReg[1] & ~MsgOutCntReg[0] & MsgOutTypeFlag;

// values for data out transactions
CoreOutValue[0][7:0]=CoreOutSelReg[31:24];
CoreOutValue[0][15:8]=CPUNUM;
CoreOutValue[0][23:16]={CoreOutSizeReg,CoreOutPLReg,2'b00,CoreOutMatchReg,CoreOutCmdReg};
CoreOutValue[0][31:24]={4'd0,CoreOutSelReg[3:0]};
CoreOutValue[0][47:32]=CoreOutMatchReg ? CoreOutDispReg : CoreOutTaskReg;
CoreOutValue[0][63:48]=CoreOutMatchReg ? CoreOutDataReg[15:0] : CoreOutSelReg[15:0];
CoreOutValue[1]=CoreOutMatchReg ? {16'd0,CoreOutDataReg[63:16]} : {CoreOutDataReg[15:0],3'd0,CoreOutOffsetReg,CoreOutSelReg[23:16]};
CoreOutValue[2]={16'd0,CoreOutDataReg[63:16]};

ReadDataValue[0]={DataFifoBus[31:0],4'd0,DataFifoBus[67:64],DataFifoBus[77:76],6'b000110,CPUNUM,DataFifoBus[75:68]};
ReadDataValue[1]={32'd0,DataFifoBus[63:32]};

MessageOutValue[0]=MsgOutTypeFlag ? {8'd0,MsgOutParamReg[23:0],8'd0,MsgOutStatReg,3'd5,CPUNUM,MsgOutParamReg[31:24]} : {MsgOutParamReg[15:0],TASKID,10'd0,CORECPL,4'd4,CPUNUM,MsgOutParamReg[31:24]};
MessageOutValue[1]={CPSR[7:0],MsgOutParamReg[79:32],MsgOutParamReg[23:16]};
MessageOutValue[2]={48'd0,CPSR[23:8]};
ErrorValue={32'd0,4'd0,ErrorFifoBus[11:8],ErrorFifoBus[15:12],4'b1110,CPUNUM,ErrorFifoBus[7:0]};

// master input enable
CoreInEnaWire=~CoreInACTReg;
// master out stage enable 
CoreOutEnaWire=&CoreOutCntReg;
// master cache stage enable
CoreRAMEnaWire=~CoreRAMACTReg | CoreOutEnaWire;

// input packet length control
InEndWire=((InCntReg==2'd1) & ((TRamInReg[18] & TRamInReg[16]) | 
								((TRamInReg[18:16]==3'd6) & (TRamInReg[21:19]==3'd0) & (TRamInReg[23:22]<2'd3)) |
								((TRamInReg[18:16]==3'b110) & (TRamInReg[21:19]!=3'd0)) |
								(TRamInReg[18:16]==3'd3) |
								(TRamInReg[18:16]==3'd5) |
								((TRamInReg[18:16]==3'd2) & ~TRamInReg[23]))) |
		((InCntReg==2'd2) & ((TRamInReg[18:16]==3'd1) | ((TRamInReg[18:16]==3'd0) & ~TRamInReg[23]) | ((TRamInReg[18:16]==3'd2) & TRamInReg[23]) | 
								((TRamInReg[18:16]==3'd6) & (TRamInReg[21:19]==3'd0) & (TRamInReg[23:22]==2'd3)))) |
		(InCntReg==2'd3);
// data transaction wire
InDataWire=~TRamInReg[18];
// data retry wire
DataRetryWire=(TRamInReg[18:16]==3'b110) & (!TRamInReg[19] | TRamInReg[23]);
// error retry wire
ErrorRetryWire=(TRamInReg[18:16]==3'b110) & (TRamInReg[21:19]!=3'd0);
// message detect wire
MessageWire=(TRamInReg[18:16]==3'b100);

// write index, if cache not have selector
if (SCRamLRUReg[2:0]<SCRamLRUReg[5:3])	LRUTemp[3:0]={1'b0,SCRamLRUReg[2:0]};
	else LRUTemp[3:0]={1'b1,SCRamLRUReg[5:3]};
if (SCRamLRUReg[8:6]<SCRamLRUReg[11:9]) LRUTemp[7:4]={1'b0, SCRamLRUReg[8:6]};
	else LRUTemp[7:4]={1'b1, SCRamLRUReg[11:9]};
if (LRUTemp[2:0]<LRUTemp[6:4]) LRUWriteIndex={1'b0, LRUTemp[3]};
	else LRUWriteIndex={1'b1, LRUTemp[7]};
// new value for LRURam
if (SCRamMatchReg[0] | (~SCRamMatchReg[3] & ~SCRamMatchReg[2] & ~SCRamMatchReg[1] & ~SCRamMatchReg[0] & ~LRUWriteIndex[1] & ~LRUWriteIndex[0])) LRUNewValue[2:0]=3'b111;
	else if (SCRamLRUReg[2:0] != 3'd0) LRUNewValue[2:0]=SCRamLRUReg[2:0] - 3'd1;
				else LRUNewValue[2:0]=3'd0;
if (SCRamMatchReg[1] | (~SCRamMatchReg[3] & ~SCRamMatchReg[2] & ~SCRamMatchReg[1] & ~SCRamMatchReg[0] & ~LRUWriteIndex[1] & LRUWriteIndex[0])) LRUNewValue[5:3]=3'b111;
	else if (SCRamLRUReg[5:3] != 3'd0) LRUNewValue[5:3]=SCRamLRUReg[5:3] - 3'd1;
				else LRUNewValue[5:3]=3'd0;
if (SCRamMatchReg[2] | (~SCRamMatchReg[3] & ~SCRamMatchReg[2] & ~SCRamMatchReg[1] & ~SCRamMatchReg[0] & LRUWriteIndex[1] & ~LRUWriteIndex[0])) LRUNewValue[8:6]=3'b111;
	else if (SCRamLRUReg[8:6] != 3'd0) LRUNewValue[8:6]=SCRamLRUReg[8:6] - 3'd1;
				else LRUNewValue[8:6]=3'd0;
if (SCRamMatchReg[3] | (~SCRamMatchReg[3] & ~SCRamMatchReg[2] & ~SCRamMatchReg[1] & ~SCRamMatchReg[0] & LRUWriteIndex[1] & LRUWriteIndex[0])) LRUNewValue[11:9]=3'b111;
	else if (SCRamLRUReg[11:9] != 3'd0) LRUNewValue[11:9]=SCRamLRUReg[11:9] - 3'd1;
				else LRUNewValue[11:9]=3'd0;
// calculated cache index
StartIndexBus=(|SCRamMatchReg) ? {SCRamSelReg[4:0], SCRamMatchReg[3] | SCRamMatchReg[2], SCRamMatchReg[3] | SCRamMatchReg[1]} : {SCRamSelReg[4:0],LRUWriteIndex};

// transaction input stage
InEnaWire=~InACTReg;
// selector cache input stage
SCInEnaWire=~SCRamACTReg;
// TRAM output stage
TRamEnaWire=~TRamACTReg | (SCInEnaWire & ClearCacheReg[7]);
// adder stage enable flag
AdderEnaWire=~AdderACTReg ;
// Ram output stage enable flag
RamEnaWire=~RamACTReg | (AdderEnaWire & ~DescLoadFlag);
// input stage enable flag
StartEnaWire=~StartACTReg | RamEnaWire;
// ram stage of selector cache
SCRamEnaWire=~SCRamACTReg | StartEnaWire;
// output enable flag
OutEnaWire=(NEXT | ~ACT) & (SNEXT | ~SACT);

// datapatch control bus
CoreOutCntBus=CoreOutCntReg + {1'b0,STONEXT};
// message control bus
MsgOutCntBus=MsgOutCntReg + {1'b0,STONEXT};

end

//=================================================================================================
//		Synchronous part
//=================================================================================================
always_ff @(negedge RESETn or posedge CLK)
begin
if (!RESETn) begin
				CoreOutMatchReg<=0;
				COREDRDY<=0;
				CoreOutCntReg<=2'd3;
				MsgOutCntReg<=2'd3;
				ErrorACTReg<=1'b0;
				ReadACTReg<=1'b0;
				ReadSecondFlag<=1'b0;
				InCntReg<=2'd0;
				MsgACTReg<=1'b0;
				MsgCntReg<=2'b00;
				InACTReg<=1'b0;
				TRamACTReg<=1'b0;
				ClearCacheReg<=8'd0;
				SCInACTReg<=1'b0;
				SCRamACTReg<=1'b0;
				StartACTReg<=1'b0;
				RamACTReg<=1'b0;
				AdderACTReg<=1'b0;
				ACT<=1'b0;
				SACT<=1'b0;
				DescLoadFlag<=1'b0;
				DescCountFlag<=1'b0;
				DescCntReg<=3'd0;
				CoreRAMACTReg<=1'b0;
				CoreInACTReg<=1'b0;
				MsgOutCReg<=3'd0;
				STONEW<=0;
				STOVAL<=0;
				AdderNewSelFlag<=0;
				RestartFlag<=0;
				LLErrorFlag<=0;
				ULErrorFlag<=0;
				ARErrorFlag<=0;
				PLErrorFlag<=0;
				DrdyACTReg<=0;
				MSGREQ<=0;
				ESTB<=0;
				SelErrorFlag<=0;
				TIDErrorFlag<=0;
				INVDErrorFlag<=0;
				NetErrorFlag<=0;
				SkipReg<=0;
				end
else begin

// output message end signal
MSGNEXT<=MsgOutEndWire;

// output message counter
if (((MsgOutCReg!=3'd5) | MSGRD) & ((MsgOutCReg!=0) | MSGREQWire)) MsgOutCReg<=(MsgOutCReg+3'd1) & {3{(MsgOutCReg!=3'd5)}};

// output to the messenger BUS(msb:lsb): CPL[121:120], Target PSO[119:96], TaskID[95:80], ProcINDX[79:64], Parameter[63:32], SourcePSO[31:0]
if (MsgOutCReg==3'd1)
	begin
	MSGDATA[121:120]<=MSGDATABus[31:30];				// CPL
	MSGDATA[119:96]<=MSGDATABus[23:0];					//Target PSO
	end
if (MsgOutCReg==3'd2) MSGDATA[95:64]<=MSGDATABus;	// Task ID and ProcIndex
if (MsgOutCReg==3'd3) MSGDATA[63:32]<=MSGDATABus;		// Parameter
if (MsgOutCReg==3'd4) MSGDATA[31:0]<=MSGDATABus;		// Source PSO

// output request to the messenger
MSGREQ<=(MsgOutCReg==3'd1);


//-----------------------------------------------
// processing transactions from core
CoreInACTReg<=(CoreInACTReg | COREACT) & (~CoreRAMEnaWire | ~CoreInACTReg);
if (CoreInEnaWire)
	begin
	CoreInCmdReg<=CORECMD;
	CoreInSizeReg<=CORESIZE;
	CoreInOffsetReg<=COREOFFSET;
	CoreInSelReg<=CORESEL;
	CoreInDataReg<=COREDTO;
	CoreInTagReg<=CORETAGO;
	CoreInCPLReg<=CORECPL;
	CoreInTaskReg<=TASKID;
	end

// tag FIFO
FifoWriteFlag<=CoreInACTReg & CoreRAMEnaWire & CoreInCmdReg;

//-----------------------------------------------
// MasterRAM output stage registers and flags
CoreRAMACTReg<=(CoreRAMACTReg & ~CoreOutEnaWire) | CoreInACTReg;
if (CoreRAMEnaWire)
	begin
	CoreRAMCmdReg<=CoreInCmdReg;
	CoreRAMSizeReg<=CoreInSizeReg;
	CoreRAMOffsetReg<=CoreInOffsetReg;
	CoreRAMSelReg<=CoreInSelReg;
	CoreRAMDataReg<=CoreInDataReg;
	CoreRAMCPLReg<=CoreInCPLReg;
	CoreRAMTaskReg<=CoreInTaskReg;
	CoreRAMReg<=MasterRAMBus;
	end

//-----------------------------------------------
// Frame forming stage registers
if (CoreOutEnaWire)
	begin
	CoreOutCmdReg<=CoreRAMCmdReg;
	CoreOutSizeReg<=CoreRAMSizeReg;
	CoreOutPLReg<=CoreRAMCPLReg;
	CoreOutTaskReg<=CoreRAMTaskReg;
	CoreOutOffsetReg<=CoreRAMOffsetReg;
	CoreOutDispReg<=CoreDisplacement[15:0];
	CoreOutDataReg<=CoreRAMDataReg;
	CoreOutMatchReg<=CoreMatchFlag;
	CoreOutSelReg<=CoreRAMSelReg;
	end
// qword counter
if ((CoreOutCntReg[1] & CoreOutCntReg[0] & CoreRAMACTReg) | 
			(~CoreOutCntReg[1] & ~CoreOutCntReg[0] & STONEXT & DataChannelSelWire) |
			((CoreOutCntReg[1] ^ CoreOutCntReg[0]) & STONEXT)) CoreOutCntReg<=(CoreOutCntReg+2'd1) | {2{CoreOutEndWire}};

///////////////////////////////////////////////////////////////////////////////////////////////////


// Output channel to routing subsystem
STONEW<=((~CoreOutCntReg[1] & ~CoreOutCntReg[0] & DataChannelSelWire) | (ReadACTReg & ReturnDataChannelFlag & ~ReadSecondFlag) | 
		(~MsgOutCntReg[1] & ~MsgOutCntReg[0] & MessageChannelFlag) | (ErrorACTReg & ReturnErrorChannelFlag)) & ~STONEXT;
STOVAL<=(~(&CoreOutCntBus) & DataChannelSelWire) | (ReadACTReg & ReturnDataChannelFlag) | (~(&MsgOutCntBus) & MessageChannelFlag) | 
		(ErrorACTReg & ReturnErrorChannelFlag & ~STONEXT);
STODTReg<=(CoreOutValue[0] & {64{~CoreOutCntBus[1] & ~CoreOutCntBus[0] & DataChannelSelWire}}) |
		(CoreOutValue[1] & {64{~CoreOutCntBus[1] & CoreOutCntBus[0]}}) |
		(CoreOutValue[2] & {64{CoreOutCntBus[1] & ~CoreOutCntBus[0]}}) |
		(ReadDataValue[0] & {64{~STONEXT & ~ReadSecondFlag & ReturnDataChannelFlag}}) |
		(ReadDataValue[1] & {64{(ReadSecondFlag | STONEXT) & ReturnDataChannelFlag}}) |
		(MessageOutValue[0] & {64{~MsgOutCntBus[1] & ~MsgOutCntBus[0] & MessageChannelFlag}}) |
		(MessageOutValue[1] & {64{~MsgOutCntBus[1] & MsgOutCntBus[0]}}) |
		(MessageOutValue[2] & {64{MsgOutCntBus[1] & ~MsgOutCntBus[0]}}) |
		(ErrorValue & {64{ErrorACTReg & ReturnErrorChannelFlag}});

//-----------------------------------------------
// Channel selection circuit
if ((&CoreOutCntReg | (~CoreOutCntReg[1] & ~CoreOutCntReg[0] & ~DataChannelSelWire)) & (~ReadACTReg | ~ReturnDataChannelFlag) &
	(&MsgOutCntReg | (~MsgOutCntReg[1] & ~MsgOutCntReg[0] & ~MessageChannelFlag)))
	begin
	ReturnDataChannelFlag<=~DataFifoEmptyWire;
	ReturnErrorChannelFlag<=DataFifoEmptyWire & ~ErrorFifoEmptyWire;
	DataChannelSelWire<=CoreRAMACTReg & DataFifoEmptyWire & ErrorFifoEmptyWire;
	MessageChannelFlag<=DataFifoEmptyWire & ErrorFifoEmptyWire & CoreOutCntReg[1] & CoreOutCntReg[0] & ~CoreRAMACTReg;
	end

///////////////////////////////////////////////////////////////////////////////////////////////////
//  Send message to another CPU
if (&MsgOutCntReg)
	begin
	MsgOutParamReg<=MSGPARAM;
	MsgOutTypeFlag<=MSGTYPE;					// if zero - forward message? if 1 - reverse
	MsgOutStatReg<=MSGSTAT;
	end
if ((MsgOutCntReg[1] & MsgOutCntReg[0] & MSGSEND) |
			(~MsgOutCntReg[1] & ~MsgOutCntReg[0] & STONEXT & MessageChannelFlag) |
			((MsgOutCntReg[1] ^ MsgOutCntReg[0]) & STONEXT)) MsgOutCntReg<=(MsgOutCntReg+2'd1) | {2{MsgOutEndWire}};

///////////////////////////////////////////////////////////////////////////////////////////////////
// 	Acess to local memory error 
ErrorACTReg<=(ErrorACTReg & (~ReturnErrorChannelFlag | ~STONEXT)) | (~ErrorFifoEmptyWire & ~ErrorACTReg);

///////////////////////////////////////////////////////////////////////////////////////////////////
//  Readed data from local memory
ReadACTReg<=(ReadACTReg & (~STONEXT | (DataFifoBus[77] & DataFifoBus[76] & ~ReadSecondFlag) | ~ReturnDataChannelFlag)) |  (~DataFifoEmptyWire & ~ReadACTReg);
// second cycle flag
if (ReadACTReg & ReturnDataChannelFlag & STONEXT) ReadSecondFlag<=ReadSecondFlag ^ (DataFifoBus[77] & DataFifoBus[76]);

///////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------
//	receive transaction from routing engine
// qword counter
if (InEnaWire & (InEndWire | (STIVAL & (STINEW | (InCntReg != 2'd0))))) InCntReg<=(InCntReg+2'd1) & {2{~InEndWire}};
if (InEnaWire & STIVAL & (InCntReg==2'b00)) TRamInReg[63:0]<=STIDT;
if (InEnaWire & STIVAL & (InCntReg==2'b01)) TRamInReg[127:64]<=STIDT;
if (InEnaWire & STIVAL & (InCntReg==2'b10)) TRamInReg[175:128]<=STIDT[47:0];

// activation data ready transfer
DrdyACTReg<=(InEndWire & DataRetryWire) | SkipFlag;
SkipReg<=SkipFlag;
// output read data result
COREDRDY<=DrdyACTReg;
COREDTI<=TRamInReg[95:32];
CORETAGI<=ReadFifoBus;
CORESIZEI<=({1'b0, TRamInReg[23:22]}) | {3{SkipReg}};

// output transaction error
ESTB<=(InEndWire & ErrorRetryWire) | (SkipReg & (|CORECPL));
case (TRamInReg[22:20] | {3{SkipReg & (|CORECPL)}})
	3'd0:	ECD<={5'd0,TRamInReg[15:8]};		// invalid selector
	3'd1:	ECD<={5'd1,TRamInReg[15:8]};		// Limit error
	3'd2:	ECD<={5'd2,TRamInReg[15:8]};		// Access type error
	3'd3:	ECD<={5'd4,TRamInReg[15:8]};		// PL Error
	3'd4:	ECD<={5'd8,TRamInReg[15:8]};		// Task ID Error
	3'd5:	ECD<={5'd16,TRamInReg[15:8]};		// invalid descriptor
	3'd6:	ECD<={5'd31,TRamInReg[15:8]};		// reserved
	3'd7:	ECD<={5'd31,TRamInReg[15:8]};		// no network accessible object
	endcase


//-----------------------------------------------
// 	output message system
MsgACTReg<=(MsgACTReg | (InEndWire & MessageWire)) & (~MsgCntReg[1] | ~MsgCntReg[0]);
// message dword counter
if (MsgACTReg) MsgCntReg<=MsgCntReg+2'd1;
	

// activation on the TRAM input stage
InACTReg<=(InACTReg | (InEndWire & InDataWire)) & (~TRamEnaWire | ~InACTReg);


//-----------------------------------------------
// TRAM output stage
TRamACTReg<=(TRamACTReg & ~SCInEnaWire) | InACTReg;
if (TRamEnaWire)
	begin
	TRamOutCmdReg<=TRamInReg[16];													// command
	TRamOutOffsetReg<=TRamInReg[17] ? TRAMBus[36:0] : TRamInReg[108:72] ;			// non-modified offet
	TRamOutSelReg<=TRamInReg[17] ? TRAMBus[79:56] : TRamInReg[71:48];				// selector
	TRamOutSizeReg<=TRamInReg[23:22];												// size
	TRamOutDataReg<=TRamInReg[17] ? TRamInReg[111:48] : TRamInReg[175:112];			// data register
	TRamOutTagReg<={TRamInReg[15:8],TRamInReg[27:24]};								// Tag
	TRamOutDispReg<=TRamInReg[47:32] & {16{TRamInReg[17]}};							// displacement
	TRamOutTaskReg<=TRamInReg[17] ? TRAMBus[55:40] : TRamInReg[47:32];				// TASKID
	TRamOutCPLReg<=TRamInReg[21:20];												// CPL
	end


//-----------------------------------------------
// Clear cache ram
if (~ClearCacheReg[7]) ClearCacheReg<=ClearCacheReg+8'd1;

//-----------------------------------------------
//	Selector cache input stage registers
SCInACTReg<=(SCInACTReg & ~SCRamEnaWire) | (TRamACTReg & ClearCacheReg[7] & ~SCRamACTReg);
if (SCInEnaWire)
	begin
	SCInCMDReg<=TRamOutCmdReg;
	SCInOffsetReg<=TRamOutOffsetReg + {{21{TRamOutDispReg[15]}},TRamOutDispReg};
	SCInSelReg<=TRamOutSelReg;
	SCInDataReg<=TRamOutDataReg;
	SCInTaskReg<=TRamOutTaskReg;
	SCInPLReg<=TRamOutCPLReg;
	SCInSizeReg<=TRamOutSizeReg;
	SCInTagReg<=TRamOutTagReg;
	end

//-----------------------------------------------
// Selector cache output stage registers
SCRamACTReg<=(SCRamACTReg & ~StartEnaWire) | SCInACTReg;
if (SCRamEnaWire)
	begin
	SCRamCMDReg<=SCInCMDReg;
	SCRamOffsetReg<=SCInOffsetReg;
	SCRamSelReg<=SCInSelReg;
	SCRamDataReg<=SCInDataReg;
	SCRamTaskReg<=SCInTaskReg;
	SCRamPLReg<=SCInPLReg;
	SCRamSizeReg<=SCInSizeReg;
	SCRamTagReg<=SCInTagReg;
	// comparatorts
	SCRamMatchReg[0]<=(SCDataBus[19] & (SCInSelReg[23:5] == SCDataBus[18:0])) | (SCInACTReg & SCRamACTReg & (SCRamSelReg==SCInSelReg) & (StartIndexBus[1:0]==2'd0));
	SCRamMatchReg[1]<=(SCDataBus[39] & (SCInSelReg[23:5] == SCDataBus[38:20])) | (SCInACTReg & SCRamACTReg & (SCRamSelReg==SCInSelReg) & (StartIndexBus[1:0]==2'd1));
	SCRamMatchReg[2]<=(SCDataBus[59] & (SCInSelReg[23:5] == SCDataBus[58:40])) | (SCInACTReg & SCRamACTReg & (SCRamSelReg==SCInSelReg) & (StartIndexBus[1:0]==2'd2));
	SCRamMatchReg[3]<=(SCDataBus[79] & (SCInSelReg[23:5] == SCDataBus[78:60])) | (SCInACTReg & SCRamACTReg & (SCRamSelReg==SCInSelReg) & (StartIndexBus[1:0]==2'd3));
	// LRU Bits
	SCRamLRUReg<=(SCInACTReg & SCRamACTReg & (SCRamSelReg[4:0]==SCInSelReg[4:0])) ? LRUNewValue : LRUBus;
	end

//-----------------------------------------------
// descriptor RAM input stage registers
StartACTReg<=(StartACTReg & ~RamEnaWire) | SCRamACTReg;
if (StartEnaWire)
	begin
	StartCmdReg<=SCRamCMDReg;					// command
	StartSizeReg<=SCRamSizeReg;					// operand size
	StartOffsetReg<=SCRamOffsetReg;			// offset
	StartDataReg<=SCRamDataReg;
	StartIndexReg<=StartIndexBus;
	StartTagReg<=SCRamTagReg;
	StartCPLReg<=SCRamPLReg;
	StartSelReg<=SCRamSelReg;
	StartTaskReg<=SCRamTaskReg;
	StartNewSelFlag<=~SCRamMatchReg[3] & ~SCRamMatchReg[2] & ~SCRamMatchReg[1] & ~SCRamMatchReg[0];
	end
StartReloadFlag<=(StartIndexReg==DescIndexReg) & TAGI[12] & TAGI[1] & ~TAGI[0] & DRDY;

//-----------------------------------------------
// descriptor RAM output stage registers
RamACTReg<=(RamACTReg & (~AdderEnaWire | DescLoadFlag)) | StartACTReg;
if (RamEnaWire)
	begin
	RamIndexReg<=StartIndexReg;
	RamOffsetReg<=StartOffsetReg;				// offset register
	RamCmdReg<=StartCmdReg;						// command register
	RamSizeReg<=StartSizeReg;					// size register
	RamDataReg<=StartDataReg;					// data register
	RamTagReg<=StartTagReg;						// tag register
	RamCPLReg<=StartCPLReg;						// CPL register
	RamStreamReg<=DRAMBus[57] & DRAMBus[56];	// stream indicator
	RamSelReg<=StartSelReg;						// original selector
	RamTaskReg<=StartTaskReg;					// task
	RamNewSelFlag<=StartNewSelFlag;
	end

RamReloadFlag<=(RamIndexReg==DescIndexReg) & TAGI[12] & TAGI[1] & ~TAGI[0] & DRDY;

if (RamEnaWire | RamReloadFlag)
	begin
	RamBaseReg<=RamReloadFlag ? DescReg[39:0] : DRAMBus[39:0];					// base address from descriptor
	RamTaskIDReg<=RamReloadFlag ? DescReg[55:40] : DRAMBus[55:40];				// task ID
	RamCtrlReg<=RamReloadFlag ? DescReg[63:56] : DRAMBus[63:56];				// control byte
	RamLLSReg<=RamReloadFlag ? DescReg[87:64] : DRAMBus[87:64];					// lower link selector
	RamULSReg<=RamReloadFlag ? DescReg[111:88] : DRAMBus[111:88];				// upper link selector
	RamLLReg<=RamReloadFlag ? DescReg[143:112] : DRAMBus[143:112];				// lower limit
	RamULReg<=RamReloadFlag ? DescReg[175:144] : DRAMBus[175:144];				// upper limit
	end

//-----------------------------------------------
// activation request on address adder stage
AdderACTReg<=(AdderACTReg & ~OutEnaWire) | (DescCountFlag & ~DescCntReg[0]  &  ~AdderACTReg) | (RamACTReg & ~AdderACTReg & ~DescLoadFlag & (RamSelReg<DTLIMIT)) | RestartFlag;
// adder ram stage registers
if (AdderEnaWire | RestartFlag)
	begin
	AdderCMDReg<=RestartFlag ? AdderSrcCMDReg : DescLoadFlag | RamCmdReg;
	AdderOffsetReg<=RestartFlag ? AdderSourceOffsetReg-{DescReg[143:112],5'd0} : (DescLoadFlag ? {SelectorReg,DescCntReg[2:1],3'd0} : RamOffsetReg-{RamLLReg,5'd0});
	AdderBaseReg<=RestartFlag ? DescReg[39:0] : (DescLoadFlag ? DTBASE : RamBaseReg);
	AdderTagReg<=RestartFlag ? {1'b0,AdderSrcTagReg} : (DescLoadFlag ? {2'b10,DescIndexReg,2'b00,DescCntReg[2:1]} : {1'b0,RamTagReg});
	AdderSizeReg<=RestartFlag ? AdderSrcSizeReg : (DescLoadFlag ? 2'b11 : RamSizeReg);

	// data 
	if (~DescLoadFlag & ~RestartFlag)
		begin
		AdderDataReg<=RamDataReg;
		AdderTaskReg<=RamTaskReg;
		AdderPLReg<=RamCPLReg;
		AdderSourceOffsetReg<=RamOffsetReg;
		AdderSrcTagReg<=RamTagReg;
		AdderSrcSizeReg<=RamSizeReg;
		AdderSrcCMDReg<=RamCmdReg;
		AdderIndexReg<=RamIndexReg;
		AdderSelReg<=RamSelReg;
		end
	// checking error situations
	LLErrorFlag<=RestartFlag ? ((AdderSourceOffsetReg[36:5]<DescReg[143:112]) & (DescReg[57:56]==2'd2)) : RamACTReg & ~DescLoadFlag & ~RamStreamReg & ~RamNewSelFlag & (RamOffsetReg[36:5]<RamLLReg);		// lower limit
	ULErrorFlag<=RestartFlag ? ((AdderSourceOffsetReg[36:5]>=DescReg[175:144]) & (DescReg[57:56]==2'd2)) : RamACTReg & ~DescLoadFlag & ~RamStreamReg & ~RamNewSelFlag & (RamOffsetReg[36:5]>=RamULReg);	// upper limit
	// access tyte error flag
	ARErrorFlag<=RestartFlag ? (AdderSrcCMDReg & ~DescReg[60]) | (~AdderSrcCMDReg & ~DescReg[61]) :
						RamACTReg & ~DescLoadFlag & ~RamNewSelFlag & ((RamCmdReg & ~RamCtrlReg[4]) | (~RamCmdReg & ~RamCtrlReg[5]));
	PLErrorFlag<=RestartFlag ? AdderPLReg>DescReg[59:58] :
						RamACTReg & ~DescLoadFlag & ~RamNewSelFlag & (RamCPLReg>RamCtrlReg[3:2]);
	TIDErrorFlag<=RestartFlag ? (AdderTaskReg != DescReg[55:40]) & (|AdderTaskReg) & (|DescReg[55:40]) :
						RamACTReg & ~DescLoadFlag & ~RamNewSelFlag & (RamTaskReg != RamTaskIDReg) & (|RamTaskReg) & (|RamTaskIDReg);
	INVDErrorFlag<=RestartFlag ?  ~DescReg[57] : RamACTReg & ~DescLoadFlag & ~RamNewSelFlag & ~RamCtrlReg[1];
	NetErrorFlag<=RestartFlag ? ~DescReg[62] : RamACTReg & ~DescLoadFlag & ~RamNewSelFlag & ~RamCtrlReg[6];
	//ARErrorFlag<=RestartFlag ? ((AdderTaskReg != DescReg[55:40]) & (|AdderTaskReg) & (|DescReg[55:40])) | ~DescReg[62] | (AdderSrcCMDReg & ~DescReg[60]) | (~AdderSrcCMDReg & ~DescReg[61]) | (AdderPLReg>DescReg[59:58]) | ~DescReg[57] :
	//					RamACTReg & ~DescLoadFlag & ~RamNewSelFlag & (((RamTaskReg != RamTaskIDReg) & (|RamTaskReg) & (|RamTaskIDReg)) | ~RamCtrlReg[6] | (RamCmdReg & ~RamCtrlReg[4]) | (~RamCmdReg & ~RamCtrlReg[5]) | (RamCPLReg>RamCtrlReg[3:2]) | ~RamCtrlReg[1]);
	// link selector flags
	LLSelFlag<=RestartFlag ? (|DescReg[87:64]) & (DescReg[87:64]<DTLIMIT):(|RamLLSReg) & (RamLLSReg<DTLIMIT);
	ULSelFlag<=RestartFlag ? (|DescReg[111:88]) & (DescReg[111:88]<DTLIMIT):(|RamULSReg) & (RamULSReg<DTLIMIT);
	// new selector flag
	AdderNewSelFlag<=~RestartFlag & ~DescLoadFlag & RamACTReg & RamNewSelFlag & (RamSelReg<DTLIMIT);
	// Size
	AdderLLSReg<=RestartFlag ? DescReg[87:64]:RamLLSReg;																// lower link selector
	AdderULSReg<=RestartFlag ? DescReg[111:88]:RamULSReg;																// upper link selector
	AdderStreamFlag<=RestartFlag ? (DescReg[57] & DescReg[56]) : ~DescLoadFlag & RamStreamReg;
	end
SelErrorFlag<=AdderEnaWire & RamACTReg & RamNewSelFlag & (RamSelReg>=DTLIMIT);

//-----------------------------------------------
// transaction activation
ACT<=(ACT & ~NEXT) | (AdderACTReg & ~LLErrorFlag & ~ULErrorFlag & ~ARErrorFlag & ~PLErrorFlag & ~TIDErrorFlag & ~INVDErrorFlag & ~NetErrorFlag & ~AdderStreamFlag & ~AdderNewSelFlag);
// stream activation
SACT<=(SACT & ~SNEXT) | (AdderACTReg & AdderStreamFlag & ~AdderNewSelFlag & ~ARErrorFlag);
// output stage
if (OutEnaWire)
	begin
	ADDR<={AdderBaseReg,5'd0}+{8'd0,AdderOffsetReg};			// address register
	CMD<=AdderCMDReg;											// command
	DTO<=AdderDataReg;											// data
	SIZEO<=AdderSizeReg;										// data size
	TAGO<=AdderTagReg;											// tag
	SEL<=AdderSelReg;											// selector
	end

//-----------------------------------------------
//	Controller for descriptor load operation
//-----------------------------------------------
DescLoadFlag<=(DescLoadFlag | (~ARErrorFlag & ((LLErrorFlag & LLSelFlag) | (ULErrorFlag & ULSelFlag))) | AdderNewSelFlag) & ~RestartFlag;

DescCountFlag<=(DescCountFlag | (~ARErrorFlag & ((LLErrorFlag & LLSelFlag) | (ULErrorFlag & ULSelFlag))) | AdderNewSelFlag) &
				(~DescCountFlag | ~DescCntReg[2] | DescCntReg[1] | DescCntReg[0] | ~AdderEnaWire);
	
// counter for descriptor loading
if (DescCountFlag & (DescCntReg[0] | AdderEnaWire)) DescCntReg<=(DescCntReg+3'd1) & {3{~DescCntReg[2] | DescCntReg[1] | DescCntReg[0]}};
// selector for descriptor loading
if (AdderNewSelFlag & ~DescLoadFlag) SelectorReg<=AdderSelReg;
	else if (LLErrorFlag & ~DescLoadFlag) SelectorReg<=AdderLLSReg;
			else if (ULErrorFlag & ~DescLoadFlag) SelectorReg<=AdderULSReg;
// index for descriptor loading
if ((AdderNewSelFlag | LLErrorFlag | ULErrorFlag) & ~DescLoadFlag) DescIndexReg<=AdderIndexReg;

// input data register filled in descriptor load process
// TAG=2xx0h - base address, task ID and control byte
// TAG=2xx1h - link selectors
// TAG=2xx2h - limits
// DescREG[39:0] - base address
// DescReG[55:40] - TASK ID
// DescREG[63:56] - control byte
// DescREG[87:64] - lower link selector
// DescReg[111:88] - upper link selector
// DescReg[143:112] - lower limit
// DescReg[175:144] - upper limit
if (TAGI[12] & ~TAGI[1] & ~TAGI[0] & DRDY) DescReg[63:0]<=DTI;
if (TAGI[12] & ~TAGI[1] & TAGI[0] & DRDY) DescReg[111:64]<={DTI[55:32],DTI[23:0]};
if (TAGI[12] & TAGI[1] & ~TAGI[0] & DRDY) DescReg[175:112]<=DTI;
RestartFlag<=TAGI[12] & TAGI[1] & ~TAGI[0] & DRDY;

end
end

endmodule
