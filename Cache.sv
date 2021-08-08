
module Cache(
	input wire RESET, CLK,
	// internal ISI
	output wire INEXT,
	input wire IACT, ICMD,
	input wire [41:0] IADDR,
	input wire [63:0] IDATi,
	input wire [7:0] IBE,
	input wire [20:0] ITAGi,
	output reg IDRDY,
	output reg [63:0] IDATo,
	output reg [20:0] ITAGo,
	// external ISI
	input wire ENEXT,
	output reg EACT, ECMD,
	output reg [41:0] EADDR,
	output reg [63:0] EDATo,
	output reg [7:0] EBE,
	output reg [1:0] ETAGo,
	input wire EDRDY,
	input wire [63:0] EDATi,
	input wire [1:0] ETAGi
	);

integer i0,i1,i2;

reg EnaReg, InACTReg, InCMDReg, ControlBypass, RamACTReg, RamCMDReg, CMemWE, ReadBypass, DeepReadBypass, LockLRURecycle, DeepControlBypass;
reg [63:0] InDataReg, RamDataReg, DOutReg, DeepDataReg;
reg [41:0] InAddrReg, RamAddrReg;
reg [7:0] InBEReg, RamBEReg, DeepBEReg;
reg [20:0] InTagReg, RamTagReg;


// uncached write buffers
reg [255:0] WBuff [0:3];
// paragraph addresses for uncached write buffers
reg [39:0] WAddr [0:3];
reg [3:0] WBInWriteFlag, WBMatchFlag, WBResetFlag, WBInitReg;
// modified byte flags for uncached write buffers
reg [31:0] WFlags [0:3];
reg [1:0] WLRU [0:3];
reg [8:0] CAReg, CWAReg;
logic [139:0] CMemBus, CMemInBus, CMemRegBus;
reg [139:0] CMemReg, CMemWDReg, CMemBypassReg;

logic RamEna, NeedWriteCache;

reg [3:0] MatchFlag, CachedWFlag, CLRUSel, WLRUSel, CacheValidFlag;

reg [8:0] CntReg;
reg [1:0]  BurstCntr, EACTReg, ECMDReg, SrcReg;
reg [3:0] ChanReg [0:1];

reg [41:0] EADDRReg [0:1];
reg [7:0] EBEReg [0:1];
reg [1:0] TagReg [0:1];

logic ReadRequest, WriteRequest;

logic [63:0] ExtDataBus, DMem_0_Bus, DMem_1_Bus, DMem_2_Bus, DMem_3_Bus;

logic [3:0] WBInWriteBus;

reg [63:0] DMemReg_0, DMemReg_1, DMemReg_2, DMemReg_3, RetReg;

logic [63:0] WDataBus [3:0];

// processing :
// 1. uncached write allocation
// 2. prepare new cache line for load data from memory
reg [3:0] Machine;
parameter RS=4'd0, CLS=4'd1, WS=4'd2, CWS=4'd3, CRS=4'd4, RWS=4'd5, RSTS=4'd6, DBWS=4'd7, WWS=4'd8, WBAS=4'd9, UNS=4'd10;

reg ClearFlag, RetFlag;
	
// cache buffers
altsyncram	DMem_0 (.addressstall_b(~(EnaReg | (RamEna & (Machine==CWS)) | ((Machine==RSTS) & NeedWriteCache & InACTReg))),
				.wren_a((EnaReg & CachedWFlag[0]) | (CLRUSel[0] & EDRDY)), .clock0(CLK),
				.byteena_a(~RamBEReg | {8{EDRDY}}), .address_a(EDRDY ? {RamAddrReg[10:2],ETAGi} : RamAddrReg[10:0]),
				.address_b((Machine==CWS) ? {RamAddrReg[10:2], BurstCntr}: (Machine==RSTS) ? InAddrReg [10:0] : IADDR[10:0]),
				.data_a(EDRDY ? ExtDataBus : RamDataReg), .q_b (DMem_0_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
				.byteena_b (1'b1), .clock1 (1'b1), .clocken0 (1'b1), .clocken1 (1'b1), .clocken2 (1'b1), .clocken3 (1'b1), .data_b ({64{1'b1}}),
				.eccstatus (), .q_a (), .rden_a (1'b1), .rden_b (1'b1), .wren_b (1'b0));
	defparam
		DMem_0.address_aclr_b = "NONE",
		DMem_0.address_reg_b = "CLOCK0",
		DMem_0.byte_size = 8,
		DMem_0.clock_enable_input_a = "BYPASS",
		DMem_0.clock_enable_input_b = "BYPASS",
		DMem_0.clock_enable_output_b = "BYPASS",
		DMem_0.lpm_type = "altsyncram",
		DMem_0.numwords_a = 2048,
		DMem_0.numwords_b = 2048,
		DMem_0.operation_mode = "DUAL_PORT",
		DMem_0.outdata_aclr_b = "NONE",
		DMem_0.outdata_reg_b = "UNREGISTERED",
		DMem_0.power_up_uninitialized = "FALSE",
		DMem_0.read_during_write_mode_mixed_ports = "DONT_CARE",
		DMem_0.widthad_a = 11,
		DMem_0.widthad_b = 11,
		DMem_0.width_a = 64,
		DMem_0.width_b = 64,
		DMem_0.width_byteena_a = 8;

altsyncram	DMem_1 (.addressstall_b(~(EnaReg | (RamEna & (Machine==CWS)) | ((Machine==RSTS) & NeedWriteCache & InACTReg))),
				.wren_a((EnaReg & CachedWFlag[1]) | (CLRUSel[1] & EDRDY)), .clock0(CLK),
				.byteena_a(~RamBEReg | {8{EDRDY}}), .address_a(EDRDY ? {RamAddrReg[10:2],ETAGi} : RamAddrReg[10:0]),
				.address_b((Machine==CWS) ? {RamAddrReg[10:2], BurstCntr}: (Machine==RSTS) ? InAddrReg [10:0] : IADDR[10:0]),
				.data_a(EDRDY ? ExtDataBus : RamDataReg), .q_b (DMem_1_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
				.byteena_b (1'b1), .clock1 (1'b1), .clocken0 (1'b1), .clocken1 (1'b1), .clocken2 (1'b1), .clocken3 (1'b1), .data_b ({64{1'b1}}),
				.eccstatus (), .q_a (), .rden_a (1'b1), .rden_b (1'b1), .wren_b (1'b0));
	defparam
		DMem_1.address_aclr_b = "NONE",
		DMem_1.address_reg_b = "CLOCK0",
		DMem_1.byte_size = 8,
		DMem_1.clock_enable_input_a = "BYPASS",
		DMem_1.clock_enable_input_b = "BYPASS",
		DMem_1.clock_enable_output_b = "BYPASS",
		DMem_1.lpm_type = "altsyncram",
		DMem_1.numwords_a = 2048,
		DMem_1.numwords_b = 2048,
		DMem_1.operation_mode = "DUAL_PORT",
		DMem_1.outdata_aclr_b = "NONE",
		DMem_1.outdata_reg_b = "UNREGISTERED",
		DMem_1.power_up_uninitialized = "FALSE",
		DMem_1.read_during_write_mode_mixed_ports = "DONT_CARE",
		DMem_1.widthad_a = 11,
		DMem_1.widthad_b = 11,
		DMem_1.width_a = 64,
		DMem_1.width_b = 64,
		DMem_1.width_byteena_a = 8;

altsyncram	DMem_2 (.addressstall_b(~(EnaReg | (RamEna & (Machine==CWS)) | ((Machine==RSTS) & NeedWriteCache & InACTReg))),
				.wren_a((EnaReg & CachedWFlag[2]) | (CLRUSel[2] & EDRDY)), .clock0(CLK),
				.byteena_a(~RamBEReg | {8{EDRDY}}), .address_a(EDRDY ? {RamAddrReg[10:2],ETAGi} : RamAddrReg[10:0]),
				.address_b((Machine==CWS) ? {RamAddrReg[10:2], BurstCntr}: (Machine==RSTS) ? InAddrReg [10:0] : IADDR[10:0]),
				.data_a(EDRDY ? ExtDataBus : RamDataReg), .q_b (DMem_2_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
				.byteena_b (1'b1), .clock1 (1'b1), .clocken0 (1'b1), .clocken1 (1'b1), .clocken2 (1'b1), .clocken3 (1'b1), .data_b ({64{1'b1}}),
				.eccstatus (), .q_a (), .rden_a (1'b1), .rden_b (1'b1), .wren_b (1'b0));
	defparam
		DMem_2.address_aclr_b = "NONE",
		DMem_2.address_reg_b = "CLOCK0",
		DMem_2.byte_size = 8,
		DMem_2.clock_enable_input_a = "BYPASS",
		DMem_2.clock_enable_input_b = "BYPASS",
		DMem_2.clock_enable_output_b = "BYPASS",
		DMem_2.lpm_type = "altsyncram",
		DMem_2.numwords_a = 2048,
		DMem_2.numwords_b = 2048,
		DMem_2.operation_mode = "DUAL_PORT",
		DMem_2.outdata_aclr_b = "NONE",
		DMem_2.outdata_reg_b = "UNREGISTERED",
		DMem_2.power_up_uninitialized = "FALSE",
		DMem_2.read_during_write_mode_mixed_ports = "DONT_CARE",
		DMem_2.widthad_a = 11,
		DMem_2.widthad_b = 11,
		DMem_2.width_a = 64,
		DMem_2.width_b = 64,
		DMem_2.width_byteena_a = 8;

altsyncram	DMem_3 (.addressstall_b(~(EnaReg | (RamEna & (Machine==CWS)) | ((Machine==RSTS) & NeedWriteCache & InACTReg))),
				.wren_a((EnaReg & CachedWFlag[3]) | (CLRUSel[3] & EDRDY)), .clock0(CLK),
				.byteena_a(~RamBEReg | {8{EDRDY}}), .address_a(EDRDY ? {RamAddrReg[10:2],ETAGi} : RamAddrReg[10:0]),
				.address_b((Machine==CWS) ? {RamAddrReg[10:2], BurstCntr}: (Machine==RSTS) ? InAddrReg [10:0] : IADDR[10:0]),
				.data_a(EDRDY ? ExtDataBus : RamDataReg), .q_b (DMem_3_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
				.byteena_b (1'b1), .clock1 (1'b1), .clocken0 (1'b1), .clocken1 (1'b1), .clocken2 (1'b1), .clocken3 (1'b1), .data_b ({64{1'b1}}),
				.eccstatus (), .q_a (), .rden_a (1'b1), .rden_b (1'b1), .wren_b (1'b0));
	defparam
		DMem_3.address_aclr_b = "NONE",
		DMem_3.address_reg_b = "CLOCK0",
		DMem_3.byte_size = 8,
		DMem_3.clock_enable_input_a = "BYPASS",
		DMem_3.clock_enable_input_b = "BYPASS",
		DMem_3.clock_enable_output_b = "BYPASS",
		DMem_3.lpm_type = "altsyncram",
		DMem_3.numwords_a = 2048,
		DMem_3.numwords_b = 2048,
		DMem_3.operation_mode = "DUAL_PORT",
		DMem_3.outdata_aclr_b = "NONE",
		DMem_3.outdata_reg_b = "UNREGISTERED",
		DMem_3.power_up_uninitialized = "FALSE",
		DMem_3.read_during_write_mode_mixed_ports = "DONT_CARE",
		DMem_3.widthad_a = 11,
		DMem_3.widthad_b = 11,
		DMem_3.width_a = 64,
		DMem_3.width_b = 64,
		DMem_3.width_byteena_a = 8;

// control memory
// control RAM with address bits 30:0, valid flag 31, modify flag 32, LRU bits 34:33,
altsyncram	CRAM(.addressstall_b(~EnaReg), .wren_a(((|MatchFlag) & RamACTReg & EnaReg)|ClearFlag|(Machine==UNS)), .clock0(CLK),
				.address_a(ClearFlag ? CntReg : RamAddrReg[10:2]), .address_b(IADDR[10:2]), .data_a(CMemInBus), .q_b(CMemBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1),
				.clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1), .data_b({140{1'b1}}), .eccstatus(),
				.q_a(), .rden_a(1'b1), .rden_b(1'b1), .wren_b(1'b0));
	defparam
		CRAM.address_aclr_b = "NONE",
		CRAM.address_reg_b = "CLOCK0",
		CRAM.clock_enable_input_a = "BYPASS",
		CRAM.clock_enable_input_b = "BYPASS",
		CRAM.clock_enable_output_b = "BYPASS",
		CRAM.lpm_type = "altsyncram",
		CRAM.numwords_a = 512,
		CRAM.numwords_b = 512,
		CRAM.operation_mode = "DUAL_PORT",
		CRAM.outdata_aclr_b = "NONE",
		CRAM.outdata_reg_b = "UNREGISTERED",
		CRAM.power_up_uninitialized = "FALSE",
		CRAM.read_during_write_mode_mixed_ports = "OLD_DATA",
		CRAM.widthad_a = 9,
		CRAM.widthad_b = 9,
		CRAM.width_a = 140,
		CRAM.width_b = 140,
		CRAM.width_byteena_a = 1;
		

assign INEXT=EnaReg;

//-------------------------------------------------------------------------------------------------
//						Asynchronous logic
//-------------------------------------------------------------------------------------------------
always_comb
begin

// New value for control memory
CMemInBus={((CMemReg[139:138]-{1'd0,|CMemReg[139:138]})|{2{MatchFlag[3] | CacheValidFlag[3]}}) & {2{~ClearFlag}},
				(CacheValidFlag[3] ? |WBMatchFlag : CMemReg[137] | CachedWFlag[3]) & ~ClearFlag, (CMemReg[136] | CacheValidFlag[3]) & ~ClearFlag,
				CacheValidFlag[3] ? RamAddrReg[41:11] : CMemReg[135:105],
			((CMemReg[104:103]-{1'd0,|CMemReg[104:103]})|{2{MatchFlag[2] | CacheValidFlag[2]}}) & {2{~ClearFlag}},
				(CacheValidFlag[2] ? |WBMatchFlag : CMemReg[102] | CachedWFlag[2]) & ~ClearFlag, (CMemReg[101] | CacheValidFlag[2]) & ~ClearFlag,
				CacheValidFlag[2] ? RamAddrReg[41:11] : CMemReg[100:70],
			((CMemReg[69:68]-{1'd0,|CMemReg[69:68]})|{2{MatchFlag[1] | CacheValidFlag[1]}}) & {2{~ClearFlag}},
				(CacheValidFlag[1] ? |WBMatchFlag : CMemReg[67] | CachedWFlag[1]) & ~ClearFlag, (CMemReg[66] | CacheValidFlag[1]) & ~ClearFlag,
				CacheValidFlag[1] ? RamAddrReg[41:11] : CMemReg[65:35],
			((CMemReg[34:33]-{1'd0,|CMemReg[34:33]})|{2{MatchFlag[0] | CacheValidFlag[0]}}) & {2{~ClearFlag}},
				(CacheValidFlag[0] ? |WBMatchFlag : CMemReg[32] | CachedWFlag[0]) & ~ClearFlag, (CMemReg[31] | CacheValidFlag[0]) & ~ClearFlag,
				CacheValidFlag[0] ? RamAddrReg[41:11] : CMemReg[30:0]} & {140{~ClearFlag}};
// new value for control register
CMemRegBus=ControlBypass ? CMemInBus : (DeepControlBypass ? CMemBypassReg : CMemBus);
				
// create databuses for write to the cache
ExtDataBus[7:0]=(WBuff[0][7:0] & {8{WBMatchFlag[0] & WFlags[0][0] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][71:64] & {8{WBMatchFlag[0] & WFlags[0][8] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][135:128] & {8{WBMatchFlag[0] & WFlags[0][16] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][199:192] & {8{WBMatchFlag[0] & WFlags[0][24] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][7:0] & {8{WBMatchFlag[1] & WFlags[1][0] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][71:64] & {8{WBMatchFlag[1] & WFlags[1][8] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][135:128] & {8{WBMatchFlag[1] & WFlags[1][16] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][199:192] & {8{WBMatchFlag[1] & WFlags[1][24] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][7:0] & {8{WBMatchFlag[2] & WFlags[2][0] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][71:64] & {8{WBMatchFlag[2] & WFlags[2][8] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][135:128] & {8{WBMatchFlag[2] & WFlags[2][16] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][199:192] & {8{WBMatchFlag[2] & WFlags[2][24] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][7:0] & {8{WBMatchFlag[3] & WFlags[3][0] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][71:64] & {8{WBMatchFlag[3] & WFlags[3][8] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][135:128] & {8{WBMatchFlag[3] & WFlags[3][16] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][199:192] & {8{WBMatchFlag[3] & WFlags[3][24] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[7:0] & {8{~(WBMatchFlag[0] & WFlags[0][0] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][8] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][16] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][24] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][0] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][8] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][16] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][24] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][0] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][8] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][16] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][24] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][0] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][8] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][16] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][24] & ETAGi[1] & ETAGi[0])}});
ExtDataBus[15:8]=(WBuff[0][15:8] & {8{WBMatchFlag[0] & WFlags[0][1] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][79:72] & {8{WBMatchFlag[0] & WFlags[0][9] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][143:136] & {8{WBMatchFlag[0] & WFlags[0][17] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][207:200] & {8{WBMatchFlag[0] & WFlags[0][25] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][15:8] & {8{WBMatchFlag[1] & WFlags[1][1] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][79:72] & {8{WBMatchFlag[1] & WFlags[1][9] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][143:136] & {8{WBMatchFlag[1] & WFlags[1][17] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][207:200] & {8{WBMatchFlag[1] & WFlags[1][25] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][15:8] & {8{WBMatchFlag[2] & WFlags[2][1] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][79:72] & {8{WBMatchFlag[2] & WFlags[2][9] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][143:136] & {8{WBMatchFlag[2] & WFlags[2][17] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][207:200] & {8{WBMatchFlag[2] & WFlags[2][25] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][15:8] & {8{WBMatchFlag[3] & WFlags[3][1] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][79:72] & {8{WBMatchFlag[3] & WFlags[3][9] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][143:136] & {8{WBMatchFlag[3] & WFlags[3][17] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][207:200] & {8{WBMatchFlag[3] & WFlags[3][25] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[15:8] & {8{~(WBMatchFlag[0] & WFlags[0][1] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][9] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][17] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][25] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][1] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][9] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][17] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][25] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][1] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][9] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][17] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][25] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][1] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][9] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][17] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][25] & ETAGi[1] & ETAGi[0])}});
ExtDataBus[23:16]=(WBuff[0][23:16] & {8{WBMatchFlag[0] & WFlags[0][2] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][87:80] & {8{WBMatchFlag[0] & WFlags[0][10] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][151:144] & {8{WBMatchFlag[0] & WFlags[0][18] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][215:208] & {8{WBMatchFlag[0] & WFlags[0][26] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][23:16] & {8{WBMatchFlag[1] & WFlags[1][2] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][87:80] & {8{WBMatchFlag[1] & WFlags[1][10] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][151:144] & {8{WBMatchFlag[1] & WFlags[1][18] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][215:208] & {8{WBMatchFlag[1] & WFlags[1][26] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][23:16] & {8{WBMatchFlag[2] & WFlags[2][2] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][87:80] & {8{WBMatchFlag[2] & WFlags[2][10] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][151:144] & {8{WBMatchFlag[2] & WFlags[2][18] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][215:208] & {8{WBMatchFlag[2] & WFlags[2][26] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][23:16] & {8{WBMatchFlag[3] & WFlags[3][2] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][87:80] & {8{WBMatchFlag[3] & WFlags[3][10] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][151:144] & {8{WBMatchFlag[3] & WFlags[3][18] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][215:208] & {8{WBMatchFlag[3] & WFlags[3][26] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[23:16] & {8{~(WBMatchFlag[0] & WFlags[0][2] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][10] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][18] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][26] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][2] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][10] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][18] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][26] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][2] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][10] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][18] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][26] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][2] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][10] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][18] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][26] & ETAGi[1] & ETAGi[0])}});
ExtDataBus[31:24]=(WBuff[0][31:24] & {8{WBMatchFlag[0] & WFlags[0][3] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][95:88] & {8{WBMatchFlag[0] & WFlags[0][11] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][159:152] & {8{WBMatchFlag[0] & WFlags[0][19] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][223:216] & {8{WBMatchFlag[0] & WFlags[0][27] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][31:24] & {8{WBMatchFlag[1] & WFlags[1][3] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][95:88] & {8{WBMatchFlag[1] & WFlags[1][11] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][159:152] & {8{WBMatchFlag[1] & WFlags[1][19] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][223:216] & {8{WBMatchFlag[1] & WFlags[1][27] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][31:24] & {8{WBMatchFlag[2] & WFlags[2][3] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][95:88] & {8{WBMatchFlag[2] & WFlags[2][11] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][159:152] & {8{WBMatchFlag[2] & WFlags[2][19] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][223:216] & {8{WBMatchFlag[2] & WFlags[2][27] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][31:24] & {8{WBMatchFlag[3] & WFlags[3][3] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][95:88] & {8{WBMatchFlag[3] & WFlags[3][11] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][159:152] & {8{WBMatchFlag[3] & WFlags[3][19] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][223:216] & {8{WBMatchFlag[3] & WFlags[3][27] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[31:24] & {8{~(WBMatchFlag[0] & WFlags[0][3] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][11] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][19] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][27] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][3] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][11] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][19] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][27] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][3] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][11] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][19] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][27] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][3] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][11] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][19] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][27] & ETAGi[1] & ETAGi[0])}});
ExtDataBus[39:32]=(WBuff[0][39:32] & {8{WBMatchFlag[0] & WFlags[0][4] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][103:96] & {8{WBMatchFlag[0] & WFlags[0][12] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][167:160] & {8{WBMatchFlag[0] & WFlags[0][20] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][231:224] & {8{WBMatchFlag[0] & WFlags[0][28] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][39:32] & {8{WBMatchFlag[1] & WFlags[1][4] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][103:96] & {8{WBMatchFlag[1] & WFlags[1][12] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][167:160] & {8{WBMatchFlag[1] & WFlags[1][20] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][231:224] & {8{WBMatchFlag[1] & WFlags[1][28] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][39:32] & {8{WBMatchFlag[2] & WFlags[2][4] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][103:96] & {8{WBMatchFlag[2] & WFlags[2][12] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][167:160] & {8{WBMatchFlag[2] & WFlags[2][20] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][231:224] & {8{WBMatchFlag[2] & WFlags[2][28] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][39:32] & {8{WBMatchFlag[3] & WFlags[3][4] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][103:96] & {8{WBMatchFlag[3] & WFlags[3][12] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][167:160] & {8{WBMatchFlag[3] & WFlags[3][20] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][231:224] & {8{WBMatchFlag[3] & WFlags[3][28] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[39:32] & {8{~(WBMatchFlag[0] & WFlags[0][4] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][12] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][20] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][28] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][4] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][12] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][20] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][28] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][4] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][12] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][20] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][28] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][4] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][12] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][20] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][28] & ETAGi[1] & ETAGi[0])}});
ExtDataBus[47:40]=(WBuff[0][47:40] & {8{WBMatchFlag[0] & WFlags[0][5] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][111:104] & {8{WBMatchFlag[0] & WFlags[0][13] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][175:168] & {8{WBMatchFlag[0] & WFlags[0][21] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][239:232] & {8{WBMatchFlag[0] & WFlags[0][29] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][47:40] & {8{WBMatchFlag[1] & WFlags[1][5] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][111:104] & {8{WBMatchFlag[1] & WFlags[1][13] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][175:168] & {8{WBMatchFlag[1] & WFlags[1][21] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][239:232] & {8{WBMatchFlag[1] & WFlags[1][29] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][47:40] & {8{WBMatchFlag[2] & WFlags[2][5] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][111:104] & {8{WBMatchFlag[2] & WFlags[2][13] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][175:168] & {8{WBMatchFlag[2] & WFlags[2][21] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][239:232] & {8{WBMatchFlag[2] & WFlags[2][29] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][47:40] & {8{WBMatchFlag[3] & WFlags[3][5] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][111:104] & {8{WBMatchFlag[3] & WFlags[3][13] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][175:168] & {8{WBMatchFlag[3] & WFlags[3][21] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][239:232] & {8{WBMatchFlag[3] & WFlags[3][29] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[47:40] & {8{~(WBMatchFlag[0] & WFlags[0][5] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][13] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][21] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][29] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][5] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][13] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][21] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][29] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][5] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][13] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][21] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][29] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][5] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][13] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][21] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][29] & ETAGi[1] & ETAGi[0])}});
ExtDataBus[55:48]=(WBuff[0][55:48] & {8{WBMatchFlag[0] & WFlags[0][6] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][119:112] & {8{WBMatchFlag[0] & WFlags[0][14] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][183:176] & {8{WBMatchFlag[0] & WFlags[0][22] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][247:240] & {8{WBMatchFlag[0] & WFlags[0][30] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][55:48] & {8{WBMatchFlag[1] & WFlags[1][6] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][119:112] & {8{WBMatchFlag[1] & WFlags[1][14] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][183:176] & {8{WBMatchFlag[1] & WFlags[1][22] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][247:240] & {8{WBMatchFlag[1] & WFlags[1][30] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][55:48] & {8{WBMatchFlag[2] & WFlags[2][6] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][119:112] & {8{WBMatchFlag[2] & WFlags[2][14] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][183:176] & {8{WBMatchFlag[2] & WFlags[2][22] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][247:240] & {8{WBMatchFlag[2] & WFlags[2][30] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][55:48] & {8{WBMatchFlag[3] & WFlags[3][6] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][119:112] & {8{WBMatchFlag[3] & WFlags[3][14] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][183:176] & {8{WBMatchFlag[3] & WFlags[3][22] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][247:240] & {8{WBMatchFlag[3] & WFlags[3][30] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[55:48] & {8{~(WBMatchFlag[0] & WFlags[0][6] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][14] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][22] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][30] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][6] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][14] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][22] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][30] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][6] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][14] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][22] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][30] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][6] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][14] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][22] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][30] & ETAGi[1] & ETAGi[0])}});
ExtDataBus[63:56]=(WBuff[0][63:56] & {8{WBMatchFlag[0] & WFlags[0][7] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][127:120] & {8{WBMatchFlag[0] & WFlags[0][15] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[0][191:184] & {8{WBMatchFlag[0] & WFlags[0][23] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[0][255:248] & {8{WBMatchFlag[0] & WFlags[0][31] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][63:56] & {8{WBMatchFlag[1] & WFlags[1][7] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][127:120] & {8{WBMatchFlag[1] & WFlags[1][15] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[1][191:184] & {8{WBMatchFlag[1] & WFlags[1][23] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[1][255:248] & {8{WBMatchFlag[1] & WFlags[1][31] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][63:56] & {8{WBMatchFlag[2] & WFlags[2][7] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][127:120] & {8{WBMatchFlag[2] & WFlags[2][15] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[2][191:184] & {8{WBMatchFlag[2] & WFlags[2][23] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[2][255:248] & {8{WBMatchFlag[2] & WFlags[2][31] & ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][63:56] & {8{WBMatchFlag[3] & WFlags[3][7] & ~ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][127:120] & {8{WBMatchFlag[3] & WFlags[3][15] & ~ETAGi[1] & ETAGi[0]}})|
				(WBuff[3][191:184] & {8{WBMatchFlag[3] & WFlags[3][23] & ETAGi[1] & ~ETAGi[0]}})|
				(WBuff[3][255:248] & {8{WBMatchFlag[3] & WFlags[3][31] & ETAGi[1] & ETAGi[0]}})|
				(EDATi[63:56] & {8{~(WBMatchFlag[0] & WFlags[0][7] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][15] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[0] & WFlags[0][23] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[0] & WFlags[0][31] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][7] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][15] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[1] & WFlags[1][23] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[1] & WFlags[1][31] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][7] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][15] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[2] & WFlags[2][23] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[2] & WFlags[2][31] & ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][7] & ~ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][15] & ~ETAGi[1] & ETAGi[0]) &
								~(WBMatchFlag[3] & WFlags[3][23] & ETAGi[1] & ~ETAGi[0]) & ~(WBMatchFlag[3] & WFlags[3][31] & ETAGi[1] & ETAGi[0])}});


// enable new memory transaction
RamEna=~EACT | ENEXT;

// if cache line must be writed to the memory
NeedWriteCache=(CLRUSel[0] & CMemReg[32]) | (CLRUSel[1] & CMemReg[67]) | (CLRUSel[2] & CMemReg[102]) | (CLRUSel[3] & CMemReg[137]);

WBInWriteBus[0]=(((WAddr[0]==InAddrReg[41:2]) & WBInitReg[0]) | (~WBInitReg[0] & (~WBInitReg[1] | (WAddr[1]!=InAddrReg[41:2])) & 
																(~WBInitReg[2] | (WAddr[2]!=InAddrReg[41:2])) & (~WBInitReg[3] | (WAddr[3]!=InAddrReg[41:2])))) &
						InACTReg & ~InCMDReg &
						((CMemRegBus[30:0]!=InAddrReg[41:11]) | ~CMemRegBus[31]) & ((CMemRegBus[65:35]!=InAddrReg[41:11]) | ~CMemRegBus[66]) &
						((CMemRegBus[100:70]!=InAddrReg[41:11]) | ~CMemRegBus[101]) & ((CMemRegBus[135:105]!=InAddrReg[41:11]) | ~CMemRegBus[136]);
WBInWriteBus[1]=(((WAddr[1]==InAddrReg[41:2]) & WBInitReg[1]) | (~WBInitReg[1] & WBInitReg[0] & (WAddr[0]!=InAddrReg[41:2]) & 
																(~WBInitReg[2] | (WAddr[2]!=InAddrReg[41:2])) & (~WBInitReg[3] | (WAddr[3]!=InAddrReg[41:2])))) &
						InACTReg & ~InCMDReg &
						((CMemRegBus[30:0]!=InAddrReg[41:11]) | ~CMemRegBus[31]) & ((CMemRegBus[65:35]!=InAddrReg[41:11]) | ~CMemRegBus[66]) &
						((CMemRegBus[100:70]!=InAddrReg[41:11]) | ~CMemRegBus[101]) & ((CMemRegBus[135:105]!=InAddrReg[41:11]) | ~CMemRegBus[136]);
WBInWriteBus[2]=(((WAddr[2]==InAddrReg[41:2]) & WBInitReg[2]) | (~WBInitReg[2] & WBInitReg[1] & WBInitReg[0] & (WAddr[0]!=InAddrReg[41:2]) & (WAddr[1]!=InAddrReg[41:2]) &
																(~WBInitReg[3] | (WAddr[3]!=InAddrReg[41:2])))) &
						InACTReg & ~InCMDReg &
						((CMemRegBus[30:0]!=InAddrReg[41:11]) | ~CMemRegBus[31]) & ((CMemRegBus[65:35]!=InAddrReg[41:11]) | ~CMemRegBus[66]) &
						((CMemRegBus[100:70]!=InAddrReg[41:11]) | ~CMemRegBus[101]) & ((CMemRegBus[135:105]!=InAddrReg[41:11]) | ~CMemRegBus[136]);
WBInWriteBus[3]=(((WAddr[3]==InAddrReg[41:2]) & WBInitReg[3]) | (~WBInitReg[3] & WBInitReg[2] & WBInitReg[1] & WBInitReg[0] & (WAddr[0]!=InAddrReg[41:2]) &
						(WAddr[1]!=InAddrReg[41:2]) & (WAddr[2]!=InAddrReg[41:2]))) & InACTReg & ~InCMDReg &
						((CMemRegBus[30:0]!=InAddrReg[41:11]) | ~CMemRegBus[31]) & ((CMemRegBus[65:35]!=InAddrReg[41:11]) | ~CMemRegBus[66]) &
						((CMemRegBus[100:70]!=InAddrReg[41:11]) | ~CMemRegBus[101]) & ((CMemRegBus[135:105]!=InAddrReg[41:11]) | ~CMemRegBus[136]);
end

//-------------------------------------------------------------------------------------------------
//						Synchronous logic
//-------------------------------------------------------------------------------------------------
always_ff @(posedge CLK or negedge RESET)
begin
if (RESET==1'b0)
	begin
	WBInitReg<=4'd0;
	EnaReg<=1'b0;
	WFlags[0]<=32'd0;
	WFlags[1]<=32'd0;
	WFlags[2]<=32'd0;
	WFlags[3]<=32'd0;
	WLRU[0]<=2'd0;
	WLRU[1]<=2'd0;
	WLRU[2]<=2'd0;
	WLRU[3]<=2'd0;
	Machine<=RS;
	ReadRequest<=1'b0;
	WriteRequest<=1'b0;
	EACT<=1'b0;
	EACTReg<=2'd0;
	ClearFlag<=1'b1;
	InACTReg<=1'b0;
	RamACTReg<=1'b0;
	WBInWriteFlag<=4'd0;
	WBMatchFlag<=4'd0;
	RetFlag<=1'b0;
	CacheValidFlag<=4'd0;
	IDRDY<=0;
	LockLRURecycle<=0;
	DeepControlBypass<=0;
	CMemBypassReg<=0;
	end
else begin

// request to read new cache line
ReadRequest<=EnaReg & InACTReg & InCMDReg & ~(((CMemRegBus[30:0]==InAddrReg[41:11]) & CMemRegBus[31])|
											((CMemRegBus[65:35]==InAddrReg[41:11]) & CMemRegBus[66])|
											((CMemRegBus[100:70]==InAddrReg[41:11]) & CMemRegBus[101])|
											((CMemRegBus[135:105]==InAddrReg[41:11]) & CMemRegBus[136]));
// request for new write buffer relocation
WriteRequest<=EnaReg & InACTReg & ~InCMDReg & ~(((CMemRegBus[30:0]==InAddrReg[41:11]) & CMemRegBus[31])|
											((CMemRegBus[65:35]==InAddrReg[41:11]) & CMemRegBus[66])|
											((CMemRegBus[100:70]==InAddrReg[41:11]) & CMemRegBus[101])|
											((CMemRegBus[135:105]==InAddrReg[41:11]) & CMemRegBus[136])) &
				(WAddr[0]!=InAddrReg[41:2]) & (WAddr[1]!=InAddrReg[41:2]) & (WAddr[2]!=InAddrReg[41:2]) & (WAddr[3]!=InAddrReg[41:2]) &
				(&WBInitReg);


EnaReg<=(EnaReg & (~InACTReg | ((CMemRegBus[30:0]==InAddrReg[41:11]) & CMemRegBus[31]) | ((CMemRegBus[65:35]==InAddrReg[41:11]) & CMemRegBus[66]) |
				((CMemRegBus[100:70]==InAddrReg[41:11]) & CMemRegBus[101]) | ((CMemRegBus[135:105]==InAddrReg[41:11]) & CMemRegBus[136]) |
				(~InCMDReg & (~WBInitReg[0] | ~WBInitReg[1] | ~WBInitReg[2] | ~WBInitReg[3] |
							(WAddr[0]==InAddrReg[41:2])|(WAddr[1]==InAddrReg[41:2])|(WAddr[2]==InAddrReg[41:2])|(WAddr[3]==InAddrReg[41:2]))))) | 
		(Machine==UNS) | (Machine==WBAS) | (&CntReg & (Machine==CLS));
 
LockLRURecycle<=(Machine==UNS) & ControlBypass;

CMemBypassReg<=CMemInBus;

DeepControlBypass<=(RamAddrReg[10:2]==IADDR[10:2]) & IACT & INEXT & (((|MatchFlag) & RamACTReg & EnaReg)|(Machine==UNS));

if (((Machine==UNS) & ControlBypass)|(EnaReg & ~LockLRURecycle))
	begin
	
	// control register
	CMemReg<=CMemRegBus;
	
	// Calculate LRU for cache
	CLRUSel[0]<=((CMemRegBus[34:33]<=CMemRegBus[69:68])&(CMemRegBus[34:33]<=CMemRegBus[104:103])&(CMemRegBus[34:33]<=CMemRegBus[139:138])& 
				CMemRegBus[66] & CMemRegBus[101] & CMemRegBus[136]) | ~CMemRegBus[31];
	CLRUSel[1]<=((CMemRegBus[69:68]<CMemRegBus[34:33])&(CMemRegBus[69:68]<=CMemRegBus[104:103])&(CMemRegBus[69:68]<=CMemRegBus[139:138])&
				CMemRegBus[31] & CMemRegBus[101] & CMemRegBus[136]) | (~CMemRegBus[66] & CMemRegBus[31]);
	CLRUSel[2]<=((CMemRegBus[104:103]<CMemRegBus[34:33])&(CMemRegBus[104:103]<CMemRegBus[69:68])&(CMemRegBus[104:103]<=CMemRegBus[139:138])&
				CMemRegBus[31] & CMemRegBus[66] & CMemRegBus[136]) | (~CMemRegBus[101] & CMemRegBus[31] & CMemRegBus[66]);
	CLRUSel[3]<=((CMemRegBus[139:138]<CMemRegBus[34:33])&(CMemRegBus[139:138]<CMemRegBus[69:68])&(CMemRegBus[139:138]<CMemRegBus[104:103])&
				CMemRegBus[31] & CMemRegBus[66] & CMemRegBus[101]) | (~CMemRegBus[136] & CMemRegBus[31] & CMemRegBus[66] & CMemRegBus[101]);
	end

if (EnaReg)
	begin
	// activation
	InACTReg<=IACT;
	RamACTReg<=InACTReg;
	// command
	InCMDReg<=ICMD;
	RamCMDReg<=InCMDReg;
	// input address
	InAddrReg<=IADDR;
	RamAddrReg<=InAddrReg;
	// input data
	InDataReg<=IDATi;
	RamDataReg<=InDataReg;
	// write data into WBuff, if address equal
	WBInWriteFlag[0]<=WBInWriteBus[0];
	WBInWriteFlag[1]<=WBInWriteBus[1];
	WBInWriteFlag[2]<=WBInWriteBus[2];
	WBInWriteFlag[3]<=WBInWriteBus[3];
	// write buffer match flags set to 1 when address of write buffer equal to the read address
	WBMatchFlag[0]<=(WAddr[0]==InAddrReg[41:2]) & WBInitReg[0] & InACTReg & InCMDReg;
	WBMatchFlag[1]<=(WAddr[1]==InAddrReg[41:2]) & WBInitReg[1] & InACTReg & InCMDReg;
	WBMatchFlag[2]<=(WAddr[2]==InAddrReg[41:2]) & WBInitReg[2] & InACTReg & InCMDReg;
	WBMatchFlag[3]<=(WAddr[3]==InAddrReg[41:2]) & WBInitReg[3] & InACTReg & InCMDReg;
	// WLRU control registers
	WLRUSel[0]<=(WLRU[0]<=WLRU[1])&(WLRU[0]<=WLRU[2])&(WLRU[0]<=WLRU[3]);
	WLRUSel[1]<=(WLRU[1]<WLRU[0])&(WLRU[1]<=WLRU[2])&(WLRU[1]<=WLRU[3]);
	WLRUSel[2]<=(WLRU[2]<WLRU[0])&(WLRU[2]<WLRU[1])&(WLRU[2]<=WLRU[3]);
	WLRUSel[3]<=(WLRU[3]<WLRU[0])&(WLRU[3]<WLRU[1])&(WLRU[3]<WLRU[2]);
	// tag
	InTagReg<=ITAGi;
	// byte enables
	InBEReg<=IBE;
	// RAM output stage
	// cache match flags
	MatchFlag[0]<=(CMemRegBus[30:0]==InAddrReg[41:11]) & CMemRegBus[31];
	MatchFlag[1]<=(CMemRegBus[65:35]==InAddrReg[41:11]) & CMemRegBus[66];
	MatchFlag[2]<=(CMemRegBus[100:70]==InAddrReg[41:11]) & CMemRegBus[101];
	MatchFlag[3]<=(CMemRegBus[135:105]==InAddrReg[41:11]) & CMemRegBus[136];
	// cached write flags
	CachedWFlag[0]<=(CMemRegBus[30:0]==InAddrReg[41:11]) & CMemRegBus[31] & InACTReg & ~InCMDReg;
	CachedWFlag[1]<=(CMemRegBus[65:35]==InAddrReg[41:11]) & CMemRegBus[66] & InACTReg & ~InCMDReg;
	CachedWFlag[2]<=(CMemRegBus[100:70]==InAddrReg[41:11]) & CMemRegBus[101] & InACTReg & ~InCMDReg;
	CachedWFlag[3]<=(CMemRegBus[135:105]==InAddrReg[41:11]) & CMemRegBus[136] & InACTReg & ~InCMDReg;
	// Tag register
	RamTagReg<=InTagReg;
	// BE signals
	RamBEReg<=InBEReg;
	// bypass control register
	ControlBypass<=(InAddrReg[10:2]==IADDR[10:2]) & IACT & InACTReg;
	// bypass data registers flags
	ReadBypass<=(IADDR[41:0]==InAddrReg[41:0]) & InACTReg & ~InCMDReg & IACT & ICMD & (Machine!=CWS);
	// deep bypass flags
	DeepReadBypass<=(IADDR[41:0]==RamAddrReg[41:0]) & RamACTReg & ~RamCMDReg & IACT & ICMD & (Machine!=CWS);
	// delayed registers
	DeepBEReg<=RamBEReg;
	DeepDataReg<=RamDataReg;
	end

if (EnaReg)
	begin
	DMemReg_0<={(~RamBEReg[7] & ReadBypass) ? RamDataReg[63:56] : (~DeepBEReg[7] & DeepReadBypass) ? DeepDataReg[63:56] : DMem_0_Bus[63:56],
				(~RamBEReg[6] & ReadBypass) ? RamDataReg[55:48] : (~DeepBEReg[6] & DeepReadBypass) ? DeepDataReg[55:48] : DMem_0_Bus[55:48],
				(~RamBEReg[5] & ReadBypass) ? RamDataReg[47:40] : (~DeepBEReg[5] & DeepReadBypass) ? DeepDataReg[47:40] : DMem_0_Bus[47:40],
				(~RamBEReg[4] & ReadBypass) ? RamDataReg[39:32] : (~DeepBEReg[4] & DeepReadBypass) ? DeepDataReg[39:32] : DMem_0_Bus[39:32],
				(~RamBEReg[3] & ReadBypass) ? RamDataReg[31:24] : (~DeepBEReg[3] & DeepReadBypass) ? DeepDataReg[31:24] : DMem_0_Bus[31:24],
				(~RamBEReg[2] & ReadBypass) ? RamDataReg[23:16] : (~DeepBEReg[2] & DeepReadBypass) ? DeepDataReg[23:16] : DMem_0_Bus[23:16],
				(~RamBEReg[1] & ReadBypass) ? RamDataReg[15:8] : (~DeepBEReg[1] & DeepReadBypass) ? DeepDataReg[15:8] : DMem_0_Bus[15:8],
				(~RamBEReg[0] & ReadBypass) ? RamDataReg[7:0] : (~DeepBEReg[0] & DeepReadBypass) ? DeepDataReg[7:0] : DMem_0_Bus[7:0]};
	DMemReg_1<={(~RamBEReg[7] & ReadBypass) ? RamDataReg[63:56] : (~DeepBEReg[7] & DeepReadBypass) ? DeepDataReg[63:56] : DMem_1_Bus[63:56],
				(~RamBEReg[6] & ReadBypass) ? RamDataReg[55:48] : (~DeepBEReg[6] & DeepReadBypass) ? DeepDataReg[55:48] : DMem_1_Bus[55:48],
				(~RamBEReg[5] & ReadBypass) ? RamDataReg[47:40] : (~DeepBEReg[5] & DeepReadBypass) ? DeepDataReg[47:40] : DMem_1_Bus[47:40],
				(~RamBEReg[4] & ReadBypass) ? RamDataReg[39:32] : (~DeepBEReg[4] & DeepReadBypass) ? DeepDataReg[39:32] : DMem_1_Bus[39:32],
				(~RamBEReg[3] & ReadBypass) ? RamDataReg[31:24] : (~DeepBEReg[3] & DeepReadBypass) ? DeepDataReg[31:24] : DMem_1_Bus[31:24],
				(~RamBEReg[2] & ReadBypass) ? RamDataReg[23:16] : (~DeepBEReg[2] & DeepReadBypass) ? DeepDataReg[23:16] : DMem_1_Bus[23:16],
				(~RamBEReg[1] & ReadBypass) ? RamDataReg[15:8] : (~DeepBEReg[1] & DeepReadBypass) ? DeepDataReg[15:8] : DMem_1_Bus[15:8],
				(~RamBEReg[0] & ReadBypass) ? RamDataReg[7:0] : (~DeepBEReg[0] & DeepReadBypass) ? DeepDataReg[7:0] : DMem_1_Bus[7:0]};
	DMemReg_2<={(~RamBEReg[7] & ReadBypass) ? RamDataReg[63:56] : (~DeepBEReg[7] & DeepReadBypass) ? DeepDataReg[63:56] : DMem_2_Bus[63:56],
				(~RamBEReg[6] & ReadBypass) ? RamDataReg[55:48] : (~DeepBEReg[6] & DeepReadBypass) ? DeepDataReg[55:48] : DMem_2_Bus[55:48],
				(~RamBEReg[5] & ReadBypass) ? RamDataReg[47:40] : (~DeepBEReg[5] & DeepReadBypass) ? DeepDataReg[47:40] : DMem_2_Bus[47:40],
				(~RamBEReg[4] & ReadBypass) ? RamDataReg[39:32] : (~DeepBEReg[4] & DeepReadBypass) ? DeepDataReg[39:32] : DMem_2_Bus[39:32],
				(~RamBEReg[3] & ReadBypass) ? RamDataReg[31:24] : (~DeepBEReg[3] & DeepReadBypass) ? DeepDataReg[31:24] : DMem_2_Bus[31:24],
				(~RamBEReg[2] & ReadBypass) ? RamDataReg[23:16] : (~DeepBEReg[2] & DeepReadBypass) ? DeepDataReg[23:16] : DMem_2_Bus[23:16],
				(~RamBEReg[1] & ReadBypass) ? RamDataReg[15:8] : (~DeepBEReg[1] & DeepReadBypass) ? DeepDataReg[15:8] : DMem_2_Bus[15:8],
				(~RamBEReg[0] & ReadBypass) ? RamDataReg[7:0] : (~DeepBEReg[0] & DeepReadBypass) ? DeepDataReg[7:0] : DMem_2_Bus[7:0]};
	DMemReg_3<={(~RamBEReg[7] & ReadBypass) ? RamDataReg[63:56] : (~DeepBEReg[7] & DeepReadBypass) ? DeepDataReg[63:56] : DMem_3_Bus[63:56],
				(~RamBEReg[6] & ReadBypass) ? RamDataReg[55:48] : (~DeepBEReg[6] & DeepReadBypass) ? DeepDataReg[55:48] : DMem_3_Bus[55:48],
				(~RamBEReg[5] & ReadBypass) ? RamDataReg[47:40] : (~DeepBEReg[5] & DeepReadBypass) ? DeepDataReg[47:40] : DMem_3_Bus[47:40],
				(~RamBEReg[4] & ReadBypass) ? RamDataReg[39:32] : (~DeepBEReg[4] & DeepReadBypass) ? DeepDataReg[39:32] : DMem_3_Bus[39:32],
				(~RamBEReg[3] & ReadBypass) ? RamDataReg[31:24] : (~DeepBEReg[3] & DeepReadBypass) ? DeepDataReg[31:24] : DMem_3_Bus[31:24],
				(~RamBEReg[2] & ReadBypass) ? RamDataReg[23:16] : (~DeepBEReg[2] & DeepReadBypass) ? DeepDataReg[23:16] : DMem_3_Bus[23:16],
				(~RamBEReg[1] & ReadBypass) ? RamDataReg[15:8] : (~DeepBEReg[1] & DeepReadBypass) ? DeepDataReg[15:8] : DMem_3_Bus[15:8],
				(~RamBEReg[0] & ReadBypass) ? RamDataReg[7:0] : (~DeepBEReg[0] & DeepReadBypass) ? DeepDataReg[7:0] : DMem_3_Bus[7:0]};
	end

if (EnaReg | (|WBResetFlag))
	begin
	// Write buffer initialisation flags
	WBInitReg[0]<=(WBInitReg[0] | WBInWriteBus[0]) & ~WBResetFlag[0];
	WBInitReg[1]<=(WBInitReg[1] | WBInWriteBus[1]) & ~WBResetFlag[1];
	WBInitReg[2]<=(WBInitReg[2] | WBInWriteBus[2]) & ~WBResetFlag[2];
	WBInitReg[3]<=(WBInitReg[3] | WBInWriteBus[3]) & ~WBResetFlag[3];
	end
// reset valid flag for write buffer when cache line filled from memory and address of write buffer equal to address of cached line
WBResetFlag[0]<=(Machine==RSTS) & (WAddr[0]==RamAddrReg[41:2]) & WBInitReg[0];
WBResetFlag[1]<=(Machine==RSTS) & (WAddr[1]==RamAddrReg[41:2]) & WBInitReg[1];
WBResetFlag[2]<=(Machine==RSTS) & (WAddr[2]==RamAddrReg[41:2]) & WBInitReg[2];
WBResetFlag[3]<=(Machine==RSTS) & (WAddr[3]==RamAddrReg[41:2]) & WBInitReg[3];

// WLRU registers
WLRU[0]<=(WLRU[0]-{1'b0,((|WLRU[0]) & EnaReg & (|WBInWriteFlag)) | (Machine==WBAS)}) | {2{(EnaReg & WBInWriteFlag[0]) | ((Machine==WBAS) & WLRUSel[0])}};
WLRU[1]<=(WLRU[1]-{1'b0,((|WLRU[1]) & EnaReg & (|WBInWriteFlag)) | (Machine==WBAS)}) | {2{(EnaReg & WBInWriteFlag[1]) | ((Machine==WBAS) & WLRUSel[1])}};
WLRU[2]<=(WLRU[2]-{1'b0,((|WLRU[2]) & EnaReg & (|WBInWriteFlag)) | (Machine==WBAS)}) | {2{(EnaReg & WBInWriteFlag[2]) | ((Machine==WBAS) & WLRUSel[2])}};
WLRU[3]<=(WLRU[3]-{1'b0,((|WLRU[3]) & EnaReg & (|WBInWriteFlag)) | (Machine==WBAS)}) | {2{(EnaReg & WBInWriteFlag[3]) | ((Machine==WBAS) & WLRUSel[3])}};

// Write buffer address registers
if ((EnaReg & WBInWriteBus[0]) | ((Machine==WBAS) & WLRUSel[0])) WAddr[0]<=(Machine==WBAS) ? RamAddrReg[41:2] : InAddrReg[41:2];
if ((EnaReg & WBInWriteBus[1]) | ((Machine==WBAS) & WLRUSel[1])) WAddr[1]<=(Machine==WBAS) ? RamAddrReg[41:2] : InAddrReg[41:2];
if ((EnaReg & WBInWriteBus[2]) | ((Machine==WBAS) & WLRUSel[2])) WAddr[2]<=(Machine==WBAS) ? RamAddrReg[41:2] : InAddrReg[41:2];
if ((EnaReg & WBInWriteBus[3]) | ((Machine==WBAS) & WLRUSel[3])) WAddr[3]<=(Machine==WBAS) ? RamAddrReg[41:2] : InAddrReg[41:2];
	
	// write data into local write buffers
for (i0=0; i0<4; i0=i0+1)
	begin
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[0]) WBuff[i0][7:0]<=RamDataReg[7:0];
	WFlags[i0][0]<=((WFlags[i0][0] & ((Machine!=WBAS) | ~WLRUSel[i0])) | 
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[0])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[1]) WBuff[i0][15:8]<=RamDataReg[15:8];
	WFlags[i0][1]<=((WFlags[i0][1] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[1])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[2]) WBuff[i0][23:16]<=RamDataReg[23:16];
	WFlags[i0][2]<=((WFlags[i0][2] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[2])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[3]) WBuff[i0][31:24]<=RamDataReg[31:24];
	WFlags[i0][3]<=((WFlags[i0][3] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[3])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[4]) WBuff[i0][39:32]<=RamDataReg[39:32];
	WFlags[i0][4]<=((WFlags[i0][4] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[4])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[5]) WBuff[i0][47:40]<=RamDataReg[47:40];
	WFlags[i0][5]<=((WFlags[i0][5] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[5])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[6]) WBuff[i0][55:48]<=RamDataReg[55:48];
	WFlags[i0][6]<=((WFlags[i0][6] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[6])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[7]) WBuff[i0][63:56]<=RamDataReg[63:56];
	WFlags[i0][7]<=((WFlags[i0][7] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[7])) & ~WBResetFlag[i0];

	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[0]) WBuff[i0][71:64]<=RamDataReg[7:0];
	WFlags[i0][8]<=((WFlags[i0][8] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[0])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[1]) WBuff[i0][79:72]<=RamDataReg[15:8];
	WFlags[i0][9]<=((WFlags[i0][9] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[1])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[2]) WBuff[i0][87:80]<=RamDataReg[23:16];
	WFlags[i0][10]<=((WFlags[i0][10] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[2])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[3]) WBuff[i0][95:88]<=RamDataReg[31:24];
	WFlags[i0][11]<=((WFlags[i0][11] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[3])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[4]) WBuff[i0][103:96]<=RamDataReg[39:32];
	WFlags[i0][12]<=((WFlags[i0][12] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[4])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[5]) WBuff[i0][111:104]<=RamDataReg[47:40];
	WFlags[i0][13]<=((WFlags[i0][13] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[5])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[6]) WBuff[i0][119:112]<=RamDataReg[55:48];
	WFlags[i0][14]<=((WFlags[i0][14] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[6])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[7]) WBuff[i0][127:120]<=RamDataReg[63:56];
	WFlags[i0][15]<=((WFlags[i0][15] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & ~RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[7])) & ~WBResetFlag[i0];

	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[0]) WBuff[i0][135:128]<=RamDataReg[7:0];
	WFlags[i0][16]<=((WFlags[i0][16] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[0])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[1]) WBuff[i0][143:136]<=RamDataReg[15:8];
	WFlags[i0][17]<=((WFlags[i0][17] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[1])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[2]) WBuff[i0][151:144]<=RamDataReg[23:16];
	WFlags[i0][18]<=((WFlags[i0][18] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[2])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[3]) WBuff[i0][159:152]<=RamDataReg[31:24];
	WFlags[i0][19]<=((WFlags[i0][19] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[3])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[4]) WBuff[i0][167:160]<=RamDataReg[39:32];
	WFlags[i0][20]<=((WFlags[i0][20] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[4])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[5]) WBuff[i0][175:168]<=RamDataReg[47:40];
	WFlags[i0][21]<=((WFlags[i0][21] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[5])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[6]) WBuff[i0][183:176]<=RamDataReg[55:48];
	WFlags[i0][22]<=((WFlags[i0][22] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[6])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[7]) WBuff[i0][191:184]<=RamDataReg[63:56];
	WFlags[i0][23]<=((WFlags[i0][23] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & ~RamAddrReg[0] & ~RamBEReg[7])) & ~WBResetFlag[i0];

	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[0]) WBuff[i0][199:192]<=RamDataReg[7:0];
	WFlags[i0][24]<=((WFlags[i0][24] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[0])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[1]) WBuff[i0][207:200]<=RamDataReg[15:8];
	WFlags[i0][25]<=((WFlags[i0][25] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[1])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[2]) WBuff[i0][215:208]<=RamDataReg[23:16];
	WFlags[i0][26]<=((WFlags[i0][26] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[2])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[3]) WBuff[i0][223:216]<=RamDataReg[31:24];
	WFlags[i0][27]<=((WFlags[i0][27] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[3])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[4]) WBuff[i0][231:224]<=RamDataReg[39:32];
	WFlags[i0][28]<=((WFlags[i0][28] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[4])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[5]) WBuff[i0][239:232]<=RamDataReg[47:40];
	WFlags[i0][29]<=((WFlags[i0][29] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[5])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[6]) WBuff[i0][247:240]<=RamDataReg[55:48];
	WFlags[i0][30]<=((WFlags[i0][30] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[6])) & ~WBResetFlag[i0];
	if (((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[7]) WBuff[i0][255:248]<=RamDataReg[63:56];
	WFlags[i0][31]<=((WFlags[i0][31] & ((Machine!=WBAS) | ~WLRUSel[i0])) |
					(((EnaReg & WBInWriteFlag[i0]) | ((Machine==WBAS) & WLRUSel[i0])) & RamAddrReg[1] & RamAddrReg[0] & ~RamBEReg[7])) & ~WBResetFlag[i0];
	end
		
// data output
IDATo<=(DMemReg_0 & {64{MatchFlag[0]}})|
		(DMemReg_1 & {64{MatchFlag[1]}})|
		(DMemReg_2 & {64{MatchFlag[2]}})|
		(DMemReg_3 & {64{MatchFlag[3]}})|
		(RetReg & {64{RetFlag}});
// data tag
ITAGo<=RamTagReg;
// data ready signal
IDRDY<=((|MatchFlag) & RamACTReg & RamCMDReg) | RetFlag;

// register for retrieving data when cache line filled from memory
if (EDRDY & (ETAGi==RamAddrReg[1:0])) RetReg<=ExtDataBus;
RetFlag<=EDRDY & (ETAGi==RamAddrReg[1:0]);

// counter for clearing control ram at the start
CntReg<=(CntReg+9'd1) & {9{(Machine==CLS)}};
ClearFlag<=(ClearFlag | (Machine==RS)) & ~((Machine==CLS) & (|CntReg));

// burst counter
if (RamEna) BurstCntr<=(BurstCntr+2'd1) & {2{(Machine==CWS)|(Machine==CRS)|(Machine==DBWS)}};

// control machine
case (Machine)
	RS:	Machine<=CLS;
	CLS: if (&CntReg) Machine<=WS;
			else Machine<=CLS;
	WS:	if (ReadRequest & NeedWriteCache) Machine<=CWS;
			else if (ReadRequest & ~NeedWriteCache) Machine<=CRS;
					else if (WriteRequest) Machine<=DBWS;
							else Machine<=WS;
	// write modified cache line
	CWS: if ((&BurstCntr) & RamEna) Machine<=CRS;
			else Machine<=CWS;
	// load new cache line
	CRS: if ((&BurstCntr) & RamEna) Machine<=RWS;
			else Machine<=CRS;
	// waiting until all data readed from SDRAM
	RWS: if (EDRDY & ETAGi[1] & ETAGi[0]) Machine<=RSTS;
			else Machine<=RWS;
	// restart transaction pipeline
	RSTS: Machine<=UNS;
	// deblocking pipeline
	UNS: Machine<=WS;
	// relocate new write buffer
	DBWS: if ((&BurstCntr) & RamEna) Machine<=WWS;
			else  Machine<=DBWS;
	WWS: if (EACTReg[1] & ENEXT & (&TagReg[1])) Machine<=WBAS;
			else Machine<=WWS;
	WBAS: Machine<=WS;
	endcase

// validation cache lines
CacheValidFlag[0]<=(Machine==RSTS) & CLRUSel[0];
CacheValidFlag[1]<=(Machine==RSTS) & CLRUSel[1];
CacheValidFlag[2]<=(Machine==RSTS) & CLRUSel[2];
CacheValidFlag[3]<=(Machine==RSTS) & CLRUSel[3];

// interface to the SDRAM

EACT<=(EACT & ~ENEXT) | EACTReg[1];
if (RamEna)
	begin
	//activation
	EACTReg[0]<=(Machine==CRS) | (Machine==DBWS);
	EACTReg[1]<=(Machine==CWS) | EACTReg[0];
	// command
	ECMDReg[0]<=(Machine==CRS);
	ECMDReg[1]<=(Machine!=CWS) & ECMDReg[0];
	ECMD<=ECMDReg[1];
	// type of data source
	SrcReg[0]<=(Machine==DBWS);
	SrcReg[1]<=SrcReg[0];
	// channel selection
	ChanReg[0]<=(Machine==DBWS) ? {WLRUSel[3],WLRUSel[2],WLRUSel[1],WLRUSel[0]} : {CLRUSel[3],CLRUSel[2],CLRUSel[1],CLRUSel[0]};
	ChanReg[1]<=ChanReg[0];
	// address
	EADDRReg[0][1:0]<=BurstCntr;
	EADDRReg[0][10:2]<=(Machine==DBWS) ? (WAddr[0][8:0] & {9{WLRUSel[0]}}) | (WAddr[1][8:0] & {9{WLRUSel[1]}}) | (WAddr[2][8:0] & {9{WLRUSel[2]}}) | (WAddr[3][8:0] & {9{WLRUSel[3]}}) : RamAddrReg[10:2];
	EADDRReg[0][41:11]<=({31{(Machine==CRS)}} & RamAddrReg[41:11]) |
					({31{(Machine==DBWS)}} & ((WAddr[0][39:9] & {31{WLRUSel[0]}}) | (WAddr[1][39:9] & {31{WLRUSel[1]}}) | (WAddr[2][39:9] & {31{WLRUSel[2]}}) | (WAddr[3][39:9] & {31{WLRUSel[3]}})));
	EADDRReg[1]<=(Machine==CWS) ? {({31{(Machine==CWS)}} & ((CMemReg[30:0] & {31{CLRUSel[0]}}) | (CMemReg[65:35] & {31{CLRUSel[1]}}) | (CMemReg[100:70] & {31{CLRUSel[2]}}) | (CMemReg[135:105] & {31{CLRUSel[3]}}))),RamAddrReg[10:2],BurstCntr} : EADDRReg[0];
	EADDR<=EADDRReg[1];
	// BE signals
	for (i2=0; i2<8; i2=i2+1)
		EBEReg[0][i2]<=(Machine!=CRS) & ((WLRUSel[0] & ((~WFlags[0][i2] & ~BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[0][8+i2] & ~BurstCntr[1] & BurstCntr[0])|
																	(~WFlags[0][16+i2] & BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[0][24+i2] & BurstCntr[1] & BurstCntr[0])))|
													(WLRUSel[1] & ((~WFlags[1][i2] & ~BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[1][8+i2] & ~BurstCntr[1] & BurstCntr[0])|
																	(~WFlags[1][16+i2] & BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[1][24+i2] & BurstCntr[1] & BurstCntr[0])))|
													(WLRUSel[2] & ((~WFlags[2][i2] & ~BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[2][8+i2] & ~BurstCntr[1] & BurstCntr[0])|
																	(~WFlags[2][16+i2] & BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[2][24+i2] & BurstCntr[1] & BurstCntr[0])))|
													(WLRUSel[3] & ((~WFlags[3][i2] & ~BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[3][8+i2] & ~BurstCntr[1] & BurstCntr[0])|
																	(~WFlags[3][16+i2] & BurstCntr[1] & ~BurstCntr[0])|
																	(~WFlags[3][24+i2] & BurstCntr[1] & BurstCntr[0]))));
	EBEReg[1]<=EBEReg[0] & {8{(Machine!=CWS)}};
	EBE<=EBEReg[1];
	// Tag
	TagReg[0]<=BurstCntr;
	TagReg[1]<=(Machine==CWS) ? BurstCntr : TagReg[0];
	ETAGo<=TagReg[1];
	// data
	EDATo<=SrcReg[1] ? ({64{ChanReg[1][0]}} & ((WBuff[0][63:0] & {64{~TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[0][127:64] & {64{~TagReg[1][1] & TagReg[1][0]}})|
												(WBuff[0][191:128] & {64{TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[0][255:192] & {64{TagReg[1][1] & TagReg[1][0]}})))|
						({64{ChanReg[1][1]}} & ((WBuff[1][63:0] & {64{~TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[1][127:64] & {64{~TagReg[1][1] & TagReg[1][0]}})|
												(WBuff[1][191:128] & {64{TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[1][255:192] & {64{TagReg[1][1] & TagReg[1][0]}})))|
						({64{ChanReg[1][2]}} & ((WBuff[2][63:0] & {64{~TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[2][127:64] & {64{~TagReg[1][1] & TagReg[1][0]}})|
												(WBuff[2][191:128] & {64{TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[2][255:192] & {64{TagReg[1][1] & TagReg[1][0]}})))|
						({64{ChanReg[1][3]}} & ((WBuff[3][63:0] & {64{~TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[3][127:64] & {64{~TagReg[1][1] & TagReg[1][0]}})|
												(WBuff[3][191:128] & {64{TagReg[1][1] & ~TagReg[1][0]}})|
												(WBuff[3][255:192] & {64{TagReg[1][1] & TagReg[1][0]}}))) :
						({64{ChanReg[0][0]}} & DMem_0_Bus)|({64{ChanReg[0][1]}} & DMem_1_Bus)|
						({64{ChanReg[0][2]}} & DMem_2_Bus)|({64{ChanReg[0][3]}} & DMem_3_Bus);
	end
end
end

endmodule
