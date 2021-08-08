module FFTCache(
	input wire RESET, CLK,
	// internal ISI
	output wire INEXT,
	input wire IACT, ICMD, ISEL,
	input wire [33:0] IOFFSET,
	input wire [31:0] ISELECTOR,
	input wire [63:0] IDATi,
	input wire [1:0] ITAGi,
	output reg IDRDY,
	output reg [63:0] IDATo,
	output reg [1:0] ITAGo,
	// external ISI
	input wire ENEXT,
	output reg EACT, ECMD, ESEL,
	output reg [33:0] EOFFSET,
	output reg [31:0] ESELECTOR,
	output reg [63:0] EDATo,
	output reg [1:0] ETAGo,
	input wire EDRDY,
	input wire [63:0] EDATi,
	input wire [1:0] ETAGi
	);

integer i0,i1,i2;

reg EnaReg, InACTReg, InCMDReg, InSELReg, ControlBypass, RamACTReg, RamCMDReg, RamSELReg, CMemWE, ReadBypass, DeepReadBypass,
	LockLRURecycle, DeepControlBypass, LockFlag;
reg [63:0] InDataReg, RamDataReg, DOutReg, DeepDataReg;
reg [33:0] InOFFSETReg, RamOFFSETReg;
reg [31:0] InSELECTORReg, RamSELECTORReg;
reg [1:0] InTagReg, RamTagReg;


reg [8:0] CAReg, CWAReg;
logic [231:0] CMemBus, CMemInBus, CMemRegBus;
reg [231:0] CMemReg, CMemBypassReg;

logic RamEna;

reg [3:0] MatchFlag, CachedWFlag, CLRUSel, WLRUSel, CacheValidFlag;

reg [8:0] CntReg;
reg [1:0]  BurstCntr;

logic ReadRequest, WriteRequest;

logic [63:0] DMem_0_Bus, DMem_1_Bus, DMem_2_Bus, DMem_3_Bus;

reg [63:0] DMemReg_0, DMemReg_1, DMemReg_2, DMemReg_3, RetReg;

// processing :
// 1. uncached write allocation
// 2. prepare new cache line for load data from memory
reg [3:0] Machine;
parameter RS=4'd0, CLS=4'd1, WS=4'd2, CRS=4'd3, RWS=4'd4, RSTS=4'd5, DBWS=4'd6, UNS=4'd7;

reg ClearFlag, RetFlag;
	
// cache buffers
altsyncram	DMem_0 (.addressstall_b(~EnaReg),
				.wren_a((EnaReg & CachedWFlag[0]) | (CLRUSel[0] & EDRDY)), .clock0(CLK),
				.byteena_a(8'hFF), .address_a(EDRDY ? {RamOFFSETReg[10:2],ETAGi} : RamOFFSETReg[10:0]),
				.address_b(IOFFSET[10:0]),
				.data_a(EDRDY ? EDATi : RamDataReg), .q_b (DMem_0_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
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

altsyncram	DMem_1 (.addressstall_b(~EnaReg),
				.wren_a((EnaReg & CachedWFlag[1]) | (CLRUSel[1] & EDRDY)), .clock0(CLK),
				.byteena_a(8'hFF), .address_a(EDRDY ? {RamOFFSETReg[10:2],ETAGi} : RamOFFSETReg[10:0]),
				.address_b(IOFFSET[10:0]),
				.data_a(EDRDY ? EDATi : RamDataReg), .q_b (DMem_1_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
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

altsyncram	DMem_2 (.addressstall_b(~EnaReg),
				.wren_a((EnaReg & CachedWFlag[2]) | (CLRUSel[2] & EDRDY)), .clock0(CLK),
				.byteena_a(8'hFF), .address_a(EDRDY ? {RamOFFSETReg[10:2],ETAGi} : RamOFFSETReg[10:0]),
				.address_b(IOFFSET[10:0]),
				.data_a(EDRDY ? EDATi : RamDataReg), .q_b (DMem_2_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
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

altsyncram	DMem_3 (.addressstall_b(~EnaReg),
				.wren_a((EnaReg & CachedWFlag[3]) | (CLRUSel[3] & EDRDY)), .clock0(CLK),
				.byteena_a(8'hFF), .address_a(EDRDY ? {RamOFFSETReg[10:2],ETAGi} : RamOFFSETReg[10:0]),
				.address_b(IOFFSET[10:0]),
				.data_a(EDRDY ? EDATi : RamDataReg), .q_b (DMem_3_Bus), .aclr0 (1'b0), .aclr1 (1'b0), .addressstall_a (1'b0),
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
// Line0: 57:56 - LRU bits 55 - valid flag, 54:0 - selector and offset
// Line1: 115:114 - LRU bits 113 - valid flag, 112:58 - selector and offset
// Line2: 173:172 - LRU bits 171 - valid flag, 170:116 - selector and offset
// Line3: 231:230 - LRU bits 229 - valid flag, 228:174 - selector and offset
altsyncram	CRAM(.addressstall_b(~EnaReg), .wren_a(((|MatchFlag) & RamACTReg & EnaReg)|ClearFlag|(Machine==UNS)), .clock0(CLK),
				.address_a(ClearFlag ? CntReg : RamOFFSETReg[10:2]), .address_b(IOFFSET[10:2]), .data_a(CMemInBus), .q_b(CMemBus),
				.aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1),
				.clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1), .data_b({232{1'b1}}), .eccstatus(),
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
		CRAM.width_a = 232,
		CRAM.width_b = 232,
		CRAM.width_byteena_a = 1;
		

assign INEXT=EnaReg & ~LockFlag;

//-------------------------------------------------------------------------------------------------
//						Asynchronous logic
//-------------------------------------------------------------------------------------------------
always_comb
begin

// New value for control memory
// Line0: 57:56 - LRU bits 55 - valid flag, 54:0 - selector and offset
// Line1: 115:114 - LRU bits 113 - valid flag, 112:58 - selector and offset
// Line2: 173:172 - LRU bits 171 - valid flag, 170:116 - selector and offset
// Line3: 231:230 - LRU bits 229 - valid flag, 228:174 - selector and offset
CMemInBus={((CMemReg[231:230]-{1'd0,|CMemReg[231:230]})|{2{MatchFlag[3] | CacheValidFlag[3]}}) & {2{~ClearFlag}},
				(CMemReg[229] | CacheValidFlag[3]) & ~ClearFlag,
				CacheValidFlag[3] ? {RamSELECTORReg, RamOFFSETReg[33:11]} : CMemReg[228:174],
			((CMemReg[173:172]-{1'd0,|CMemReg[173:172]})|{2{MatchFlag[2] | CacheValidFlag[2]}}) & {2{~ClearFlag}},
				(CMemReg[171] | CacheValidFlag[2]) & ~ClearFlag,
				CacheValidFlag[2] ? {RamSELECTORReg, RamOFFSETReg[33:11]} : CMemReg[170:116],
			((CMemReg[115:114]-{1'd0,|CMemReg[115:114]})|{2{MatchFlag[1] | CacheValidFlag[1]}}) & {2{~ClearFlag}},
				(CMemReg[113] | CacheValidFlag[1]) & ~ClearFlag,
				CacheValidFlag[1] ? {RamSELECTORReg, RamOFFSETReg[33:11]} : CMemReg[112:58],
			((CMemReg[57:56]-{1'd0,|CMemReg[57:56]})|{2{MatchFlag[0] | CacheValidFlag[0]}}) & {2{~ClearFlag}},
				(CMemReg[55] | CacheValidFlag[0]) & ~ClearFlag,
				CacheValidFlag[0] ? {RamSELECTORReg, RamOFFSETReg[33:11]} : CMemReg[54:0]} & {232{~ClearFlag}};
// new value for control register
CMemRegBus=ControlBypass ? CMemInBus : (DeepControlBypass ? CMemBypassReg : CMemBus);
				
// enable new memory transaction
RamEna=~EACT | ENEXT;

end

//-------------------------------------------------------------------------------------------------
//						Synchronous logic
//-------------------------------------------------------------------------------------------------
always_ff @(posedge CLK or negedge RESET)
begin
if (RESET==1'b0)
	begin
	EnaReg<=1'b0;
	Machine<=RS;
	ReadRequest<=1'b0;
	WriteRequest<=1'b0;
	EACT<=1'b0;
	ClearFlag<=1'b1;
	InACTReg<=1'b0;
	RamACTReg<=1'b0;
	RetFlag<=1'b0;
	CacheValidFlag<=4'd0;
	IDRDY<=0;
	LockLRURecycle<=0;
	DeepControlBypass<=0;
	CMemBypassReg<=0;
	CntReg<=0;
	LockFlag<=1'b1;
	end
else begin

// lock flag for all input requests
LockFlag<=(LockFlag | (IACT & ~ICMD & (&ITAGi) & EnaReg)) & ~((Machine==CLS) & (&CntReg));

// request to read new cache line
ReadRequest<=EnaReg & InACTReg & InCMDReg & ~(((CMemRegBus[54:0]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[55])|
											((CMemRegBus[112:58]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[113])|
											((CMemRegBus[170:116]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[171])|
											((CMemRegBus[228:174]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[229]));

EnaReg<=(EnaReg & (~InACTReg | ((CMemRegBus[54:0]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[55] & InCMDReg) |
								((CMemRegBus[112:58]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[113] & InCMDReg) |
								((CMemRegBus[170:116]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[171] & InCMDReg) |
								((CMemRegBus[228:174]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[229] & InCMDReg))) |
					(Machine==UNS) | (&CntReg & (Machine==CLS)) | ((Machine==DBWS) & RamEna);
 
LockLRURecycle<=(Machine==UNS) & ControlBypass;

CMemBypassReg<=CMemInBus;

DeepControlBypass<=(RamOFFSETReg[10:2]==IOFFSET[10:2]) & IACT & INEXT & (((|MatchFlag) & RamACTReg & EnaReg)|(Machine==UNS));

if (((Machine==UNS) & ControlBypass)|(EnaReg & ~LockLRURecycle))
	begin
	
	// control register
	CMemReg<=CMemRegBus;

	// Calculate LRU for cache
	// Line0: 57:56 - LRU bits 55 - valid flag, 54:0 - selector and offset
	// Line1: 115:114 - LRU bits 113 - valid flag, 112:58 - selector and offset
	// Line2: 173:172 - LRU bits 171 - valid flag, 170:116 - selector and offset
	// Line3: 231:230 - LRU bits 229 - valid flag, 228:174 - selector and offset
	CLRUSel[0]<=((CMemRegBus[57:56]<=CMemRegBus[115:114])&
				(CMemRegBus[57:56]<=CMemRegBus[173:172])&
				(CMemRegBus[57:56]<=CMemRegBus[231:230])& 
				CMemRegBus[113] & CMemRegBus[171] & CMemRegBus[229]) | ~CMemRegBus[55];
	CLRUSel[1]<=((CMemRegBus[115:114]<CMemRegBus[57:56])&
				(CMemRegBus[115:114]<=CMemRegBus[173:172])&
				(CMemRegBus[115:114]<=CMemRegBus[231:230])&
				CMemRegBus[55] & CMemRegBus[171] & CMemRegBus[229]) | (~CMemRegBus[113] & CMemRegBus[55]);
	CLRUSel[2]<=((CMemRegBus[173:172]<CMemRegBus[57:56])&
				(CMemRegBus[173:172]<CMemRegBus[115:114])&
				(CMemRegBus[173:172]<=CMemRegBus[231:230])&
				CMemRegBus[55] & CMemRegBus[113] & CMemRegBus[229]) | (~CMemRegBus[171] & CMemRegBus[55] & CMemRegBus[113]);
	CLRUSel[3]<=((CMemRegBus[231:230]<CMemRegBus[57:56])&
				(CMemRegBus[231:230]<CMemRegBus[115:114])&
				(CMemRegBus[231:230]<CMemRegBus[173:172])&
				CMemRegBus[55] & CMemRegBus[113] & CMemRegBus[171]) | (~CMemRegBus[229] & CMemRegBus[55] & CMemRegBus[113] & CMemRegBus[171]);
	
	end

if (EnaReg)
	begin
	// request for new write buffer relocation
	WriteRequest<=IACT & ~ICMD & ~LockFlag;
	// activation
	InACTReg<=IACT & ~LockFlag;
	RamACTReg<=InACTReg;
	// command
	InCMDReg<=ICMD;
	RamCMDReg<=InCMDReg;
	// SEL tag
	InSELReg<=ISEL;
	RamSELReg<=InSELReg;
	// input offset and selector
	InOFFSETReg<=IOFFSET;
	RamOFFSETReg<=InOFFSETReg;
	InSELECTORReg<=ISELECTOR;
	RamSELECTORReg<=InSELECTORReg;
	// input data
	InDataReg<=IDATi;
	RamDataReg<=InDataReg;
	// tag
	InTagReg<=ITAGi;
	// RAM output stage
	// cache match flags
	MatchFlag[0]<=(CMemRegBus[54:0]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[55];
	MatchFlag[1]<=(CMemRegBus[112:58]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[113];
	MatchFlag[2]<=(CMemRegBus[170:116]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[171];
	MatchFlag[3]<=(CMemRegBus[228:174]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[229];
	// cached write flags
	CachedWFlag[0]<=(CMemRegBus[54:0]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[55] & InACTReg & ~InCMDReg;
	CachedWFlag[1]<=(CMemRegBus[112:58]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[113] & InACTReg & ~InCMDReg;
	CachedWFlag[2]<=(CMemRegBus[170:116]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[171] & InACTReg & ~InCMDReg;
	CachedWFlag[3]<=(CMemRegBus[228:174]=={InSELECTORReg, InOFFSETReg[33:11]}) & CMemRegBus[229] & InACTReg & ~InCMDReg;
	// Tag register
	RamTagReg<=InTagReg;
	// bypass control register
	ControlBypass<=(InOFFSETReg[10:2]==IOFFSET[10:2]) & IACT & InACTReg & ~LockFlag;
	// bypass data registers flags
	ReadBypass<=({ISELECTOR, IOFFSET}=={InSELECTORReg, InOFFSETReg}) & InACTReg & ~InCMDReg & IACT & ICMD & ~LockFlag;
	// deep bypass flags
	DeepReadBypass<=({ISELECTOR, IOFFSET}=={RamSELECTORReg, RamOFFSETReg}) & RamACTReg & ~RamCMDReg & IACT & ICMD & ~LockFlag;
	// delayed registers
	DeepDataReg<=RamDataReg;
	end

if (EnaReg)
	begin
	DMemReg_0<=ReadBypass ? RamDataReg : DeepReadBypass ? DeepDataReg : DMem_0_Bus;
	DMemReg_1<=ReadBypass ? RamDataReg : DeepReadBypass ? DeepDataReg : DMem_1_Bus;
	DMemReg_2<=ReadBypass ? RamDataReg : DeepReadBypass ? DeepDataReg : DMem_2_Bus;
	DMemReg_3<=ReadBypass ? RamDataReg : DeepReadBypass ? DeepDataReg : DMem_3_Bus;
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
if (EDRDY & (ETAGi==RamOFFSETReg[1:0])) RetReg<=EDATi;
RetFlag<=EDRDY & (ETAGi==RamOFFSETReg[1:0]);

// counter for clearing control ram at the start
CntReg<=(CntReg+9'd1) & {9{(Machine==CLS)}};
ClearFlag<=(ClearFlag | (Machine==RS)) & ~((Machine==CLS) & (&CntReg));

// burst counter
if (RamEna) BurstCntr<=(BurstCntr+2'd1) & {2{(Machine==CRS)}};

// control machine
case (Machine)
	RS:	Machine<=CLS;
	CLS: if (&CntReg) Machine<=WS;
			else Machine<=CLS;
	WS:	if (ReadRequest) Machine<=CRS;
			else if (WriteRequest) Machine<=DBWS;
					else Machine<=WS;
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
	// write transit data
	DBWS: if (~RamEna) Machine<=DBWS;
			else if ((&RamTagReg) & ~RamCMDReg) Machine<=RS;
					else Machine<=WS;
	endcase

// validation cache lines
CacheValidFlag[0]<=(Machine==RSTS) & CLRUSel[0];
CacheValidFlag[1]<=(Machine==RSTS) & CLRUSel[1];
CacheValidFlag[2]<=(Machine==RSTS) & CLRUSel[2];
CacheValidFlag[3]<=(Machine==RSTS) & CLRUSel[3];

// interface to the SDRAM

EACT<=(EACT & ~ENEXT) | (Machine==CRS) | (Machine==DBWS);
if (RamEna)
	begin
	// command
	ECMD<=(Machine==CRS);
	// Offset
	EOFFSET[1:0]<=(Machine==DBWS) ? RamOFFSETReg[1:0] : BurstCntr;
	EOFFSET[33:2]<=RamOFFSETReg[33:2];
	// Selector
	ESELECTOR<=RamSELECTORReg;
	// Tag
	ETAGo<=(Machine==DBWS) ? RamTagReg : BurstCntr;
	// data
	EDATo<=RamDataReg;
	// selection bit
	ESEL<=RamSELReg;
	end
end
end

endmodule
