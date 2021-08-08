module EU64 (
	input wire CLK, RESET, CLOAD,
	input wire [7:0] CPU,
	input wire [15:0] TASKID,
	input wire [23:0] CPSR,
	// interface to datapatch of memory subsystem
	input wire NEXT, StreamNEXT, NetNEXT,
	output reg ACT, StreamACT, NetACT, CMD, 
	output reg [1:0] OS,
	output reg [44:0] ADDRESS,
	output reg [36:0] OFFSET,
	output reg [31:0] SELECTOR,
	output reg [63:0] DTo,
	output reg [7:0] TAGo,
	input wire DRDY,
	input wire [7:0] TAGi,
	input wire [2:0] SZi,
	input wire [63:0] DTi,
	// FFT memory interface
	input wire FFTMemNEXT, FFTNetNEXT,
	output wire FFTMemACT, FFTNetACT, FFTMemCMD,
	output wire [41:0] FFTMemADDR,
	output wire [33:0] FFTMemOFFSET,
	output wire [31:0] FFTMemSEL,
	output wire [63:0] FFTMemDO,
	output wire [7:0] FFTMemTO,
	// interface to the descriptor pre-loading system
	input wire [39:0] DTBASE,
	input wire [23:0] DTLIMIT,
	input wire [1:0] CPL,
	// interface to message control subsystem
	output wire [63:0] PARAM,
	output wire [3:0] PARREG,
	output wire MSGREQ, MALLOCREQ, PARREQ, ENDMSG, BKPT,
	output reg EMPTY,
	output wire HALT,
	output wire NPCS,
	output wire SMSGBKPT,
	input wire CEXEC, ESMSG, CONTINUE,
	input wire CSTOP, LIRESET,
	// context store interface
	input wire [5:0] RA,
	output reg [63:0] CDATA,
	output wire [36:0] RIP,
	// error reporting interface
	output wire ESTB,
	output wire [28:0] ECD,
	// FFT engine error report interface
	output wire FFTESTB,
	output wire [28:0] FFTECD,
	output wire [23:0] FFTCPSR,
	// test purpose pins
	output wire FATAL,
	// Performance monitor outputs
	output reg [3:0] IV,
	output wire [23:0] CSEL
	);

integer i;

// general purpose register file
reg [127:0] GPR [15:0];
// arithmetic flags
reg [31:0] AFR [15:0];
reg [15:0] FlagWrite;
reg ContextLoadFlag;
// address register file
reg [36:0] ADR [15:0];
// GPR valid flags
reg [15:0] GPVFReg;
wire [15:0] GPVFINVD;

// FACC unit resources
wire [19:0] FMULACCBus;
reg [19:0] FMULACCReg; //0 - ACT, 3:1 - ctrlSel, 7:4 - DST, 11:8 - CtrlOffsetReg, 15:12 - DataOffsetReg, 18:16 - DataSel, 19-CMD
reg FMULACCACTReg, FMULACCCMDReg, FFT32NEXTReg;
wire FMULACCNEXT;
reg [3:0] FMULACCDSTReg;
reg [2:0] FMULACCDataSelReg, FMULACCCtrlSelReg;
reg [34:0] FMULACCDataOffsetReg;
reg [33:0] FMULACCCtrlOffsetReg;
wire FMULACCRDY, FMULACCSIGN, FMULACCZERO, FMULACCINF, FMULACCNAN;
wire [31:0] FMULACCR;
wire [3:0] FMULACCDST;
wire FMULACCMemACT, FMULACCSIZE, FMULACCTAGo;
wire [2:0] FMULACCMemSEL;
wire [34:0] FMULACCMemOffset;
reg FMULACCDRDYFlag;
reg [5:0] FMULACCBypassControlOffsetReg, FMULACCBypassDataOffsetReg, FFTBypassReg;
wire SkipFifoData, SkipFifoFlag;
reg [4:0] FFTParReg;

// FFT resources
wire FFT32NEXT;
reg [31:0] FFTDataSelReg, FFTCtrlSelReg;
reg [33:0] FFTDataBaseReg, FFTCtrlBaseReg;
reg [18:0] FFTIndexReg;

// FADD unit resources
wire [13:0] FADDInstBus;
reg [13:0] FADDInstReg;
reg [127:0] FADDAReg, FADDBReg;
reg [2:0] FADDSAReg, FADDSBReg;
reg [3:0] FADDDSTReg, FADDDSTR;
reg [5:0] FADDBypassAReg, FADDBypassBReg;
reg FADDACTReg, FADDCMDReg, FADDRDYR;
wire [127:0] FADDR;
wire [3:0] FADDDST;
wire [2:0] FADDSR;
wire FADDRDY, FADDZero, FADDSign, FADDInf, FADDNaN;

// FMUL resources
wire [12:0] FMULInstBus;
reg [12:0] FMULInstReg;
reg [127:0] FMULAReg, FMULBReg;
reg FMULACTReg;
reg [2:0] FMULSAReg, FMULSBReg;
reg [3:0] FMULDSTReg;
reg [5:0] FMULBypassAReg, FMULBypassBReg;
wire [63:0] FMULR;
wire [127:0] FMULRQ;
wire [3:0] FMULDST [1:0];
wire FMULSR;
wire [1:0] FMULRDY, FMULZero, FMULSign, FMULInf, FMULNaN;
reg [1:0] FMULRDYR;
reg [3:0] FMULDSTR [1:0];

// FDIV unit resources
wire [13:0] FDIVInstBus;
reg [13:0] FDIVInstReg;
reg [127:0] FDIVAReg, FDIVBReg;
reg FDIVACTReg, FDIVCMDReg;
reg [2:0] FDIVSAReg, FDIVSBReg, FDIVSRReg;
reg [3:0] FDIVDSTReg;
reg [5:0] FDIVBypassAReg, FDIVBypassBReg;
wire [127:0] FDIVR;
wire [3:0] FDIVDST;
wire [2:0] FDIVSR;
wire FDIVRDY, FDIVZero, FDIVInf, FDIVNaN, FDIVSign, FDIVNEXT;

// Integer ALU Resources
wire [24:0] ALUInstBus;
reg [24:0] ALUInstReg;
reg ALUACTReg, ALURDYR;
reg [63:0] ALUAReg, ALUBReg, ALUCReg, ALUDReg;
reg [1:0] ALUSAReg, ALUSBReg, ALUSRReg, ALUSCReg, ALUSDReg;
reg [2:0] ALUOpCODEReg;
reg [3:0] ALUDSTReg, ALUDSTR;
reg [5:0] ALUBypassAReg, ALUBypassBReg, ALUBypassCReg, ALUBypassDReg;
wire [63:0] ALUR;
wire [3:0] ALUDST;
wire ALURDY, ALUOVR, ALUSign, ALUZero;
wire [15:0] ALUCOUT;
wire [1:0] ALUSR;

// Parallel shifter resources
wire [12:0] ShiftInstBus;
reg [12:0] ShiftInstReg;
reg [63:0] ShiftAReg;
reg [5:0] ShiftBReg, ShiftBypassAReg, ShiftBypassBReg;
reg [3:0] ShiftDSTReg, ShiftDSTR;
reg [2:0] ShiftOPRReg;
reg [1:0] ShiftSAReg, ShiftSRReg;
reg ShiftACTReg, ShiftRDYR;
wire [63:0] ShiftR;
wire [3:0] ShiftDST;
wire ShiftRDY, ShiftOVR, ShiftZERO, ShiftCOUT, ShiftSign;
wire [1:0] ShiftSR;

// miscellaneous unit resources
wire [11:0] MiscInstBus;
reg [11:0] MiscInstReg;
reg [2:0] MiscOPRReg;
reg MiscACTReg, MiscRDYR;
reg [127:0] MiscAReg;
reg [3:0] MiscDSTReg, MiscDSTR;
reg [2:0] MiscSAReg, MiscSDReg, MiscSRReg;
reg [15:0] MiscCINReg;
reg [5:0] MiscBypassAReg, MiscBypassBReg;
wire [127:0] MiscR;
wire [3:0] MiscDST;
wire [2:0] MiscSR;
wire MiscRDY, MiscCOUT, MiscOVR, MiscSign, MiscZero, MiscNaN;

// data movement resources
reg [14:0] MovInstReg;
wire [14:0] MovInstBus;
reg [127:0] MovReg;
reg [2:0] MovSR;
reg [15:0] LIResetFlag, LIFlag;
reg [5:0] CopyBypassReg;

// loop instruction bus
reg [15:0] LoopInstReg;
wire [15:0] LoopInstBus;
reg LoopInstFlag;
reg [5:0] LoopBypassReg;

// prefetch control resources
reg [16:0] PrefInstReg;
wire [16:0] PrefInstBus;
reg [5:0] PrefBypassReg, PrefCCBypassReg;
wire IFetch, InsRDY, PrefACT, PrefCall, PrefRet, PrefEMPTY, PrefERST, CodeError;
wire [3:0] IVF;
wire [63:0] INST;
wire [36:0] PrefOffset;
wire [35:0] PrefRealIP;
wire [2:0] PrefTag;
reg PrefCC, PrefCallReg, PrefRetReg;
wire SeqEMPTY, MFLag;

// memory read/write resources
reg [15:0] MemInstReg;
wire [15:0] MemInstBus;
reg MemDelayFlag, MemFifoFullFlag;
wire [216:0] MemFifoBus;
wire MemFifoEmpty;
wire [5:0] MemFifoUsedW;
reg MemBusy, MemReq, MemLoadSel, MemOpr, MemADRPushPop, MemPUSH, MemPOP, MemLDST, MemLDO, MemLoadOffset, MemSecondCycle, MemReadAR, MemLSelNode, MemNext;
reg [2:0] MemSEL, MemSize, MemAMode;
reg [31:0] MemAFR;
reg [36:0] ARData, MemGPROffset;
reg [127:0] MemGPR;
reg [3:0] MemDST, MemOFF;
reg [4:0] MemoryOSValue;
reg [15:0] MemASelNode, MemFSelNode;
reg [1:0] MemCNT;
reg [2:0] SZiReg;
reg [7:0] TAGiReg;

// check memory access stage
reg CheckACT, CheckCMD, CheckNetwork, CheckNext, CheckPref;
reg [1:0] CheckOS;
reg [36:0] CheckOffset;
reg [31:0] CheckLL, CheckUL, CheckSelector;
reg [3:0] CheckAR;
reg [39:0] CheckBase;
reg [23:0] CheckLowerSel, CheckUpperSel;
reg [63:0] CheckData;
reg [7:0] CheckTag;
reg [2:0] CheckSEL;

/*
descriptor registers
39:0	- base address		40
71:40	- lower limit		32
103:72	- upper limit		32
127:104	- lower selector	24
151:128 - upper selector	24
154:152 - access rights		4
155 	- valid bit
*/
reg [155:0] DTR [7:0];
reg [8:0] DTRWriteFlag;
reg DTRLoadedFlag, DescriptorLoadState, ValidDescriptor, LoadNewDSC, LoadLowerDSC, LoadUpperDSC, RetryTransactionFlag;
reg [23:0] DLSelector;
// state machine for descriptor loading
reg [3:0] DLMachine;
parameter WS=4'd0, CSS=4'd1, ZSS=4'd2, INVSS=4'd3, LBS=4'd4, LLSS=4'd5, LSLS=4'd6, RWS=4'd7, INVOBS=4'd8, STS=4'd9, AERSS=4'd10;
reg SkipDataRead, InvalidType, InvalidCPL, InvalidTaskID, AccessError;
reg [3:0] STAGReg;
reg [23:0] InvalidSelector;


// wire result flags
reg [15:0] GPRByteNode, GPRWordNode, GPRDwordNode, GPRQwordNode, GPROwordNode, FADDSelNode, FMULSelNode, FMULQSelNode, 
			FMULACCSelNode, FDIVSelNode, ALUSelNode, ShiftSelNode, MiscSelNode, MovSelNode, MemSelNode, MemARSelNode,
			FFTSelNode;

/*
===========================================================================================================
				Modules
===========================================================================================================
*/
// memory operations queue														55							54:52						51:15				14:0
sc_fifo MemFIFO (.data({PrefRet, PrefCall, AFR[MemInstReg[15:12]], PrefCall ? {91'd0,PrefRealIP,1'b0}:GPR[MemInstReg[15:12]], AFR[MemInstReg[11:8]][27:25], GPR[MemInstReg[11:8]][36:0], MemInstReg[15:1]}),
				.wrreq((MemInstReg[0] & ~MemDelayFlag & ~MemFifoFullFlag) | PrefCall | PrefRet),
				.rdreq((~MemBusy | (MemNext & ~FMULACCMemACT) | MemLoadOffset | (MemLoadSel & (MemDST!={CheckSEL,CheckACT})) | MemReadAR) & ~MemCNT[1] & ~MemCNT[0]),
				.clock(CLK), .sclr(~RESET), .q(MemFifoBus), .empty(MemFifoEmpty), .almost_empty(), .usedw(MemFifoUsedW));
defparam MemFIFO.LPM_WIDTH=217, MemFIFO.LPM_NUMWORDS=64, MemFIFO.LPM_WIDTHU=6;

// Instruction prefetcher
Prefetcher Pref (.CLK(CLK), .RESET(RESET), .CSELCHG(MemASelNode[13]), .DRDY(DRDY), .TAGi(TAGi), .DTi(DTi), .IFETCH(IFetch), .IRDY(InsRDY),
				.VF(IVF), .IBUS(INST), .NEXT(~MemReq & MemNext & ~FMULACCMemACT & ~PrefCallReg & ~PrefRetReg), .ACT(PrefACT), .OFFSET(PrefOffset), .TAGo(PrefTag),
				// instruction for prefetcher
				.CBUS(LoopInstFlag ? {LoopInstReg, 1'b1} : PrefInstReg),
				.GPRBUS(LoopInstFlag ? ((GPR[LoopInstReg[15:12]][63:0] & {64{~(|LoopBypassReg)}}) | (FADDR[63:0] & {64{LoopBypassReg[0]}}) |
										(FMULR & {64{LoopBypassReg[1]}}) | (FMULRQ[63:0] & {64{LoopBypassReg[2]}}) |
										(ALUR & {64{LoopBypassReg[3]}}) | (ShiftR & {64{LoopBypassReg[4]}}) | (MiscR[63:0] & {64{LoopBypassReg[5]}})) :
									   ((GPR[PrefInstReg[16:13]][63:0] & {64{~(|PrefBypassReg)}}) | (FADDR[63:0] & {64{PrefBypassReg[0]}}) |
										(FMULR & {64{PrefBypassReg[1]}}) | (FMULRQ[63:0] & {64{PrefBypassReg[2]}}) |
										(ALUR & {64{PrefBypassReg[3]}}) | (ShiftR & {64{PrefBypassReg[4]}}) | (MiscR[63:0] & {64{PrefBypassReg[5]}}))),
				// flags
				.CC(PrefCC), .MOVFLAG(MFLag | MemInstReg[0]), .CALL(PrefCall), .RET(PrefRet), .ERST(PrefERST), .RIP(PrefRealIP), .CEXEC(CEXEC),
				.ESMSG(ESMSG), .CONTINUE(CONTINUE), .STOP(CSTOP | ESTB),
				// interface to message and memory control system
				.GPINDX(PARREG),
				.SMSG(MSGREQ), .MALLOC(MALLOCREQ), .GPAR(PARREQ), .EMSG(ENDMSG), .BKPT(BKPT), .EMPTY(PrefEMPTY), .HALT(HALT), .PARAM(PARAM),
				.SLEEP(NPCS), .SMSGBKPT(SMSGBKPT),
				// code fetch error
				.CFERROR(CodeError),
				// Fatal error reporting
				.FATAL(FATAL)
				);


// Instruction sequencer
Sequencer Seq (.CLK(CLK), .RESET(RESET), .IRDY(InsRDY), .VF(IVF), .IBUS(INST),
				.IRD(IFetch), .GPRVF(GPVFReg), .GPRINVD(GPVFINVD), .EMPTY(SeqEMPTY), .MOVFLAG(MFLag), .FDIVRDY(~FDIVInstReg[0]), .FMULACCRDY(~FMULACCReg[0]),
				.MEMRDY(~MemDelayFlag & ~MemFifoFullFlag), .FADDBus(FADDInstBus), .FMULBus(FMULInstBus),
				.FMULACCBus(FMULACCBus), .FDIVBus(FDIVInstBus), .ALUBus(ALUInstBus), .ShiftBus(ShiftInstBus), .MiscBus(MiscInstBus), .LoopBus(LoopInstBus),
				.MovBus(MovInstBus), .MemBus(MemInstBus), .CtrlBus(PrefInstBus));

// 32-bit multiplier/accumulator
Neuro32 FMULACC (.CLK(CLK), .RESET(RESET), .ACT(FMULACCACTReg & ~FMULACCCMDReg), .MAERR(CheckACT & CheckCMD & (DLMachine==STS) & (CheckTag[7:1]==7'b1100000)),
				.NEXT(FMULACCNEXT), .DSTi(FMULACCDSTReg), .DataSEL(FMULACCDataSelReg), .CtrlSEL(FMULACCCtrlSelReg), .DataOffset(FMULACCDataOffsetReg),
				.CtrlOffset(FMULACCCtrlOffsetReg), .RDY(FMULACCRDY), .SIGN(FMULACCSIGN), .ZERO(FMULACCZERO), .INF(FMULACCINF), .NAN(FMULACCNAN),
				.R(FMULACCR), .DSTo(FMULACCDST),
				// memory interface
				.MemNEXT(MemNext), .MemACT(FMULACCMemACT), .MemSEL(FMULACCMemSEL), .MemOffset(FMULACCMemOffset), .SIZE(FMULACCSIZE),
				.TAGo(FMULACCTAGo), .DRDY(FMULACCDRDYFlag | SkipFifoFlag),
				.TAGi((TAGiReg[0] & FMULACCDRDYFlag) | (SkipFifoData & ~FMULACCDRDYFlag)), .DTi(DTi));

SFifo SkipFifo(.CLK(CLK), .RST(RESET), .D(CheckTag[0]), .WR(CheckACT & CheckCMD & (DLMachine==STS) & (CheckTag[7:1]==7'b1100000)),
				.Q(SkipFifoData), .VALID(SkipFifoFlag), .RD(~FMULACCDRDYFlag));


// 32-bit FFT processor
fft FFT32(.CLK(CLK), .RESET(RESET), .CPU(CPU), .DTBASE(DTBASE), .DTLIMIT(DTLIMIT), .CPL(CPL), .TASKID(TASKID), .CPSR(CPSR),
				// command interface
				.NEXT(FFT32NEXT), .ACT(FMULACCACTReg & FMULACCCMDReg), .DataObject(FFTDataSelReg), .ControlObject(FFTCtrlSelReg),
				.DataOffset(FFTDataBaseReg+FMULACCDataOffsetReg[34:1]), .ControlOffset(FFTCtrlBaseReg+FMULACCCtrlOffsetReg),
				.PAR(FFTParReg), .INDEX(FFTIndexReg),
				// memory interface
				.MemNEXT(FFTMemNEXT), .NetNEXT(FFTNetNEXT), .MemACT(FFTMemACT), .NetACT(FFTNetACT), .MemCMD(FFTMemCMD), .MemADDR(FFTMemADDR),
				.MemOFFSET(FFTMemOFFSET), .MemSEL(FFTMemSEL), .MemDO(FFTMemDO), .MemTO(FFTMemTO), .MemDRDY(DRDY), .MemDI(DTi), .MemTI(TAGi),
				// error report interface
				.ESTB(FFTESTB), .ECD(FFTECD), .ECPSR(FFTCPSR));

// 128-bit adder module
FPADD128 FPAdder(.CLK(CLK), .RESET(RESET), .ACT(FADDACTReg), .CMD(FADDCMDReg), .A(FADDAReg), .B(FADDBReg),
		.SA(FADDSAReg), .SB(FADDSBReg), .DSTI(FADDDSTReg), .RDY(FADDRDY), .SIGN(FADDSign), .ZERO(FADDZero), .INF(FADDInf),
		.NAN(FADDNaN), .SR(FADDSR), .DSTO(FADDDST), .R(FADDR));

// 128-bit FP multiplier
FPMUL128 FPMult (.CLK(CLK), .RESET(RESET), .ACT(FMULACTReg), .DSTI(FMULDSTReg), .A(FMULAReg), .B(FMULBReg),
				.SA(FMULSAReg), .SB(FMULSBReg), .RDYSD(FMULRDY[0]), .RDYQ(FMULRDY[1]), .ZEROSD(FMULZero[0]), .SIGNSD(FMULSign[0]),
				.INFSD(FMULInf[0]), .NANSD(FMULNaN[0]), .ZEROQ(FMULZero[1]), .SIGNQ(FMULSign[1]), .INFQ(FMULInf[1]),
				.NANQ(FMULNaN[1]), .SR(FMULSR), .DSTSD(FMULDST[0]), .DSTQ(FMULDST[1]), .RSD(FMULR), .RQ(FMULRQ));

// FP 64-bit divisor
FPDIV128 FDIV (.CLK(CLK), .ACT(FDIVACTReg), .CMD(FDIVCMDReg), .SA(FDIVSAReg), .SB(FDIVSBReg), .RST(RESET), .A(FDIVAReg), .B(FDIVBReg),
				.DSTi(FDIVDSTReg), .R(FDIVR), .DSTo(FDIVDST), .RDY(FDIVRDY), .Zero(FDIVZero), .Inf(FDIVInf),
				.NaN(FDIVNaN), .Sign(FDIVSign), .SR(FDIVSR), .NEXT(FDIVNEXT));

// integer ALU
ALU64 IntALU (.CLK(CLK), .ACT(ALUACTReg), .CIN(1'b0), .DSTi(ALUDSTReg), .A(ALUAReg), .B(ALUBReg), .C(ALUCReg), .D(ALUDReg),
				.OpCODE(ALUOpCODEReg), .SA(ALUSAReg), .SB(ALUSBReg), .SC(ALUSCReg), .SD(ALUSDReg), .R(ALUR), .COUT(ALUCOUT),
				.DSTo(ALUDST), .SR(ALUSR), .RDY(ALURDY), .OVR(ALUOVR), .Zero(ALUZero), .Sign(ALUSign));

// shifter
Shifter64 Sht (.CLK(CLK), .ACT(ShiftACTReg), .CIN(0), .A(ShiftAReg), .B(ShiftBReg), .DSTi(ShiftDSTReg),
				.SA(ShiftSAReg), .OPR(ShiftOPRReg), .R(ShiftR), .DSTo(ShiftDST), .RDY(ShiftRDY), .OVR(ShiftOVR),
				.ZERO(ShiftZERO), .SIGN(ShiftSign), .COUT(ShiftCOUT), .SR(ShiftSR));

// misc unit
Misc64 Misc (.CLK(CLK), .ACT(MiscACTReg), .OpCODE(MiscOPRReg), .SA(MiscSAReg), .SD(MiscSDReg), .DSTi(MiscDSTReg),
				.CIN(MiscCINReg), .A(MiscAReg), .RDY(MiscRDY), .ZERO(MiscZero), .NaN(MiscNaN), .SIGN(MiscSign),
				.OVR(MiscOVR), .COUT(MiscCOUT), .SR(MiscSR), .DSTo(MiscDST), .R(MiscR));

/*
===========================================================================================================
				Assignments part
===========================================================================================================
*/
assign RIP={PrefRealIP, 1'b0};

// error reporting
assign ESTB=(((DLMachine==ZSS) | (DLMachine==INVSS) | (DLMachine==AERSS)) & ~CheckTag[6] & ~CheckTag[5]) | CodeError | (DLMachine==INVOBS);
assign ECD[23:0]=CodeError ? ADR[13][23:0] : ((DLMachine==INVOBS) ? InvalidSelector : CheckSelector[23:0]);
assign ECD[24]=CodeError | LoadLowerDSC | LoadUpperDSC;
assign ECD[25]=CodeError | (DLMachine==AERSS);
assign ECD[26]=InvalidCPL & ~CodeError;
assign ECD[27]=InvalidTaskID & ~CodeError;
assign ECD[28]=InvalidType & ~CodeError;

assign CSEL=ADR[13][23:0];

/*
===========================================================================================================
				Asynchronous part
===========================================================================================================
*/
always @*
	begin
	
	// LOOP instruction flag
	LoopInstFlag=MiscInstReg[3] & MiscInstReg[2] & MiscInstReg[1] & MiscInstReg[0];

	// Next node on the check access stage
	CheckNext=(~ACT | NEXT) & (~StreamACT | StreamNEXT) & (~NetACT | NetNEXT);
	
	// next node on the memory forming stage
	MemNext=~CheckACT | (CheckNext & ((CheckAR[3] & (CheckOffset[36:5]>=CheckLL) & (CheckOffset[36:5]<CheckUL) & (CheckCMD | CheckAR[1]) & 
													(~CheckCMD | CheckAR[0])) | (CheckAR[3] & CheckAR[2]) | CheckNetwork)) |
						((DLMachine==STS) & (~CheckTag[6] | ~CheckTag[5] | CheckTag[4] | CheckTag[3] | CheckNext));
	
	// forming condition code for prefetcher
	case (PrefInstReg[11:9])
		3'd0:	PrefCC=PrefInstReg[12] ^ ((AFR[PrefInstReg[8:5]][16] & ~(|PrefCCBypassReg)) | (FADDZero & PrefCCBypassReg[0]) | (FMULZero[0] & PrefCCBypassReg[1]) |
										(FMULZero[1] & PrefCCBypassReg[2]) | (ALUZero & PrefCCBypassReg[3]) | (ShiftZERO & PrefCCBypassReg[4]) | (MiscZero & PrefCCBypassReg[5]));
		3'd1:	PrefCC=PrefInstReg[12] ^ ((AFR[PrefInstReg[8:5]][15] & ~PrefCCBypassReg[3] & ~PrefCCBypassReg[5]) | (ALUCOUT[15] & PrefCCBypassReg[3]) |
										(MiscCOUT & PrefCCBypassReg[5]));
		3'd2:	PrefCC=PrefInstReg[12] ^ ((AFR[PrefInstReg[8:5]][17] & ~(|PrefCCBypassReg)) | (FADDSign & PrefCCBypassReg[0]) | (FMULSign[0] & PrefCCBypassReg[1]) |
										(FMULSign[1] & PrefCCBypassReg[2]) | (ALUSign & PrefCCBypassReg[3]) | (ShiftSign & PrefCCBypassReg[4]) | (MiscSign & PrefCCBypassReg[5]));
		3'd3:	PrefCC=PrefInstReg[12] ^ ((AFR[PrefInstReg[8:5]][18] & ~PrefCCBypassReg[3] & ~PrefCCBypassReg[4]) | (ALUOVR & PrefCCBypassReg[3]) | (ShiftOVR & PrefCCBypassReg[4]));
		3'd4:	PrefCC=PrefInstReg[12] ^ ((AFR[PrefInstReg[8:5]][19] & ~PrefCCBypassReg[0] & ~PrefCCBypassReg[1] & ~PrefCCBypassReg[2] & ~PrefCCBypassReg[5]) |
										(FADDInf & PrefCCBypassReg[0]) | (FMULInf[0] & PrefCCBypassReg[1]) | (FMULInf[1] & PrefCCBypassReg[2]) | (MiscOVR & PrefCCBypassReg[5]));
		3'd5:	PrefCC=PrefInstReg[12] ^ ((AFR[PrefInstReg[8:5]][20] & ~PrefCCBypassReg[0] & ~PrefCCBypassReg[1] & ~PrefCCBypassReg[2] & ~PrefCCBypassReg[5]) |
										(FADDNaN & PrefCCBypassReg[0]) | (FMULNaN[0] & PrefCCBypassReg[1]) | (FMULNaN[1] & PrefCCBypassReg[2]) | (MiscNaN & PrefCCBypassReg[5]));
		3'd6:	PrefCC=PrefInstReg[12] ^ ((AFR[PrefInstReg[8:5]][21] & ~PrefCCBypassReg[4]) | (ShiftCOUT & PrefCCBypassReg[4]));
		3'd7:	PrefCC=1'b1;
		endcase

	// forming flags for reset LI counters
	for (i=0; i<16; i=i+1)
		LIResetFlag[i]=(FADDInstReg[0] & ((FADDInstReg[5:2]==i) | (FADDInstReg[9:6]==i) | (FADDInstReg[13:10]==i)))|
						(FMULInstReg[0] & ((FMULInstReg[4:1]==i) | (FMULInstReg[8:5]==i) | (FMULInstReg[12:9]==i)))|
						(FMULACCReg[0] & ((FMULACCReg[7:4]==i) | (FMULACCReg[11:8]==i) | (FMULACCReg[15:12]==i)))|
						(FDIVInstReg[0] & (((FDIVInstReg[4:1]==i) & ~FDIVInstReg[13]) | (FDIVInstReg[8:5]==i) | (FDIVInstReg[12:9]==i)))|
						(ALUInstReg[0] & (((ALUInstReg[6:3]==i) & ~ALUInstReg[15]) | ((ALUInstReg[10:7]==i) & ~ALUInstReg[15]) | 
											((ALUInstReg[14:11]==i) & ~ALUInstReg[24]) | ((ALUInstReg[23:20]==i) & ALUInstReg[24]) |
											((ALUInstReg[19:16]==i) & ALUInstReg[24])))|
						(ShiftInstReg[0] & ((ShiftInstReg[12:9]==i) | (ShiftInstReg[4] & (ShiftInstReg[8:5]==i))))|
						(MiscInstReg[0] & ((MiscInstReg[11:8]==i) | (MiscInstReg[3] & ~(MiscInstReg[2] & MiscInstReg[1]) & (MiscInstReg[7:4]==i))))|
						(MovInstReg[0] & (((MovInstReg[14:11]==i) & (MovInstReg[2:1]!=2'b11)) | ((MovInstReg[10:7]==i) & (MovInstReg[2:1]==2'b00))))|
						(PrefInstReg[0] & (((PrefInstReg[8:5]==i) & (PrefInstReg[4:1]==4'd2)) | 
											((PrefInstReg[16:13]==i) & (PrefInstReg[8:1]==8'hF0) & 
																	((PrefInstReg[12:10]==3'd3) | (PrefInstReg[12:10]==3'd5) | (PrefInstReg[12:9]==4'hC)))))|
						(MemInstReg[0] & (((MemInstReg[15:12]==i) & (MemInstReg[4:1]!=4'd6) & (~(MemInstReg[11] & (&MemInstReg[3:1])))) | 
											((MemInstReg[11:8]==i) & ((MemInstReg[4:3]==2'b10) | 
																														(MemInstReg[4:1]==4'b1100) |
																														(MemInstReg[4:1]==4'b0000) |
																														(MemInstReg[4:1]==4'b0110)))));

	// Memory operand Size forming
	MemoryOSValue[4]=MemLDST & (MemSize==3'b100);
	MemoryOSValue[3]=MemPUSH | MemPOP | (MemLDST & (MemSize==3'b011));
	MemoryOSValue[2]=MemLDST & (MemSize==3'b010);
	MemoryOSValue[1]=MemLDST & (MemSize==3'b001);
	MemoryOSValue[0]=MemLDST & (MemSize==3'b000);

	// interface to the descriptor reloading system
	// Load New sel flag
	LoadNewDSC=CheckACT & ~CheckAR[3] & ~CheckNetwork;
	// load lower segment
	LoadLowerDSC=CheckACT & CheckAR[3] & ~CheckAR[2] & (CheckOffset[36:5]<CheckLL) & ~CheckNetwork;
	// load upper segment
	LoadUpperDSC=CheckACT & CheckAR[3] & ~CheckAR[2] & (CheckOffset[36:5]>=CheckUL) & ~CheckNetwork;
	// access error by type
	AccessError=CheckACT & CheckAR[3] & ~CheckNetwork & ((~CheckCMD & ~CheckAR[1]) | (CheckCMD & ~CheckAR[0])) & ~CheckPref;
	
	end

/*
===========================================================================================================
				Synchronous part
===========================================================================================================
*/
always @(negedge RESET or posedge CLK)
if (!RESET) begin
				LIFlag<=16'd0;
				GPVFReg<=16'hFFFF;
				AFR[0]<=32'd0;
				AFR[1]<=32'd0;
				AFR[2]<=32'd0;
				AFR[3]<=32'd0;
				AFR[4]<=32'd0;
				AFR[5]<=32'd0;
				AFR[6]<=32'd0;
				AFR[7]<=32'd0;
				AFR[8]<=32'd0;
				AFR[9]<=32'd0;
				AFR[10]<=32'd0;
				AFR[11]<=32'd0;
				AFR[12]<=32'd0;
				AFR[13]<=32'd0;
				AFR[14]<=32'd0;
				AFR[15]<=32'd0;
				ADR[0]<=0;
				ADR[1]<=0;
				ADR[2]<=0;
				ADR[3]<=0;
				ADR[4]<=0;
				ADR[5]<=0;
				ADR[6]<=0;
				ADR[7]<=0;
				ADR[8]<=0;
				ADR[9]<=0;
				ADR[10]<=0;
				ADR[11]<=0;
				ADR[12]<=0;
				ADR[13]<=0;
				ADR[14]<=0;
				ADR[15]<=0;
				GPR[0]<=0;
				GPR[1]<=0;
				GPR[2]<=0;
				GPR[3]<=0;
				GPR[4]<=0;
				GPR[5]<=0;
				GPR[6]<=0;
				GPR[7]<=0;
				GPR[8]<=0;
				GPR[9]<=0;
				GPR[10]<=0;
				GPR[11]<=0;
				GPR[12]<=0;
				GPR[13]<=0;
				GPR[14]<=0;
				GPR[15]<=0;
				FMULACCReg[0]<=1'b0;
				FADDInstReg[0]<=1'b0;
				FMULInstReg[0]<=1'b0;
				FDIVInstReg[0]<=1'b0;
				FDIVACTReg<=1'b0;
				ALUInstReg[0]<=1'b0;
				ShiftInstReg[0]<=1'b0;
				MiscInstReg[0]<=1'b0;
				LoopInstReg[0]<=1'b0;
				PrefInstReg[0]<=1'b0;
				MovInstReg[0]<=1'b0;
				MemInstReg[0]<=1'b0;
				MemBusy<=1'b0;
				MemCNT<=2'b00;
				CheckACT<=1'b0;
				ACT<=1'b0;
				StreamACT<=1'b0;
				NetACT<=1'b0;
				DLMachine<=WS;
				DescriptorLoadState<=1'b0;
				DTR[0]<=156'hB000000000000FFFFFFFF000000000000000000;
				DTR[1]<=156'hB000000000000FFFFFFFF000000000000000000;
				DTR[2]<=156'hB000000000000FFFFFFFF000000000000000000;
				DTR[3]<=156'hB000000000000FFFFFFFF000000000000000000;
				DTR[4]<=156'hB000000000000FFFFFFFF000000000000000000;
				DTR[5]<=156'hB000000000000FFFFFFFF00000000FFFFFFFFFC;
				DTR[6]<=156'hB000000000000FFFFFFFF000000000000000000;
				DTR[7]<=156'hB000000000000FFFFFFFF000000000000000000;
				PrefCallReg<=1'b0;
				PrefRetReg<=1'b0;
				MemReadAR<=0;
				MemLoadOffset<=0;
				MemLoadSel<=0;
				MemReq<=0;
				MemPUSH<=0;
				MemPOP<=0;
				MemADRPushPop<=0;
				MemLDO<=0;
				FMULACCACTReg<=0;
				FMULACCDRDYFlag<=0;
				FFT32NEXTReg<=0;
				IV<=0;
				MemFifoFullFlag<=0;
				end
else begin
	
	// Fifo full flag
	MemFifoFullFlag<=(MemFifoUsedW>6'd59);
	
	// Instructions transfered to the sequencer
	IV<=IVF & {4{IFetch & InsRDY}};
	
	// EU empty flag
	EMPTY<=PrefEMPTY & SeqEMPTY & (&GPVFReg);
	
	// output data for context store operations
	case (RA[5:4])
		2'd0: CDATA<=GPR[RA[3:0]][63:0];
		2'd1: CDATA<=GPR[RA[3:0]][127:64];
		2'd2: CDATA<={32'd0, AFR[RA[3:0]]};
		2'd3: CDATA<={27'd0, ADR[RA[3:0]]};
		endcase
	
	// Storing TAGi
	TAGiReg<=TAGi;

	// LI flags
	for (i=0; i<16; i=i+1)
		LIFlag[i]<=(&MovInstReg[2:0]) & (MovInstReg[14:11]==i);
	
	// processign GPR valid flags
	for (i=0; i<16; i=i+1)
			GPVFReg[i]<=(GPVFReg[i] & GPVFINVD[i])|
						((FADDDST==i) & FADDRDY)|
						((FMULDST[0]==i) & FMULRDY[0])|
						((FMULDST[1]==i) & FMULRDY[1])|
						FMULACCSelNode[i]|
						((FMULACCDSTReg==i) & FMULACCACTReg & FMULACCCMDReg)|
						((FDIVDST==i) & FDIVRDY)|
						((ALUDST==i) & ALURDY)|
						((ShiftDST==i) & ShiftRDY)|
						((MiscDST==i) & MiscRDY)|
						((MovInstReg[14:11]==i) & MovInstReg[0])|
						(MemReadAR & (MemDST==i))|
						(DRDY & (TAGi[3:0]==i) & ~TAGi[6] & ~TAGi[5] & (~TAGi[7] | TAGi[4])) |
						(SkipDataRead & (STAGReg==i));
	// result strobe nodes
	for (i=0; i<16; i=i+1)
		begin
		FlagWrite[i]<=(FADDRDYR & (FADDDSTR==i))|
						(FMULRDYR[0] & (FMULDSTR[0]==i))|
						(FMULRDYR[1] & (FMULDSTR[1]==i))|
						(FMULACCRDY & (FMULACCDST==i))|
						(FMULACCACTReg & FMULACCCMDReg & (FMULACCDSTReg==i))|
						(FDIVRDY & (FDIVDST==i))|
						(ALURDYR & (ALUDSTR==i))|
						(ShiftRDYR & (ShiftDSTR==i))|
						(MiscRDYR & (MiscDSTR==i))|
						(DRDY & (TAGi[3:0]==i) & ~TAGi[6] & ((TAGi[5] & ~TAGi[4]) | (~TAGi[5] & (~TAGi[7] | TAGi[4]))));

		GPRByteNode[i]<=(FADDRDYR & (FADDDSTR==i))|
						(FMULRDYR[0] & (FMULDSTR[0]==i))|
						(FMULRDYR[1] & (FMULDSTR[1]==i))|
						(FMULACCRDY & (FMULACCDST==i))|
						(FDIVRDY & (FDIVDST==i))|
						(ALURDYR & (ALUDSTR==i))|
						(ShiftRDYR & (ShiftDSTR==i))|
						(MiscRDYR & (MiscDSTR==i))|
						(MovInstReg[0] & (MovInstReg[14:11]==i) & (MovInstReg[2:1]!=2'b01))|
						(MemReadAR & (MemDST==i))|
						(DRDY & (TAGi[3:0]==i) & ~TAGi[6] & ~TAGi[5] & ~TAGi[4]);

		GPRWordNode[i]<=(FADDRDYR & (FADDDSTR==i))|
						(FMULRDYR[0] & (FMULDSTR[0]==i))|
						(FMULRDYR[1] & (FMULDSTR[1]==i))|
						(FMULACCRDY & (FMULACCDST==i))|
						(FDIVRDY & (FDIVDST==i))|
						(ALURDYR & (ALUDSTR==i) & (|ALUSR))|
						(ShiftRDYR & (ShiftDSTR==i) & (|ShiftSR))|
						(MiscRDYR & (MiscDSTR==i) & (|MiscSR))|
						(MovInstReg[0] & (MovInstReg[14:11]==i) & (MovInstReg[2:1]!=2'b01))|
						(MemReadAR & (MemDST==i))|
						(DRDY & (TAGi[3:0]==i) & (|SZi[1:0]) & ~TAGi[6] & ~TAGi[5] & ~TAGi[4]);

		GPRDwordNode[i]<=(FADDRDYR & (FADDDSTR==i))|
						(FMULRDYR[0] & (FMULDSTR[0]==i))|
						(FMULRDYR[1] & (FMULDSTR[1]==i))|
						(FMULACCRDY & (FMULACCDST==i))|
						(FDIVRDY & (FDIVDST==i))|
						(ALURDYR & (ALUDSTR==i) & ALUSR[1])|
						(ShiftRDYR & (ShiftDSTR==i) & ShiftSR[1])|
						(MiscRDYR & (MiscDSTR==i) & (MiscSR[2] | MiscSR[1]))|
						(MovInstReg[0] & (MovInstReg[14:11]==i) & (MovInstReg[2:1]!=2'b01))|
						(MemReadAR & (MemDST==i))|
						(DRDY & (TAGi[3:0]==i)& SZi[1] & ~TAGi[6] & ~TAGi[5] & ~TAGi[4]);

		GPRQwordNode[i]<=(FADDRDYR & (FADDDSTR==i))|
						(FMULRDYR[0] & (FMULDSTR[0]==i))|
						(FMULRDYR[1] & (FMULDSTR[1]==i))|
						(FDIVRDY & (FDIVDST==i) & (FDIVSR[2] | FDIVSR[0]))|
						(ALURDYR & (ALUDSTR==i) & (&ALUSR))|
						(ShiftRDYR & (ShiftDSTR==i) & ShiftSR[1] & ShiftSR[0])|
						(MiscRDYR & (MiscDSTR==i) & ((MiscSR[1] & MiscSR[0]) | MiscSR[2]))|
						(MovInstReg[0] & (MovInstReg[14:11]==i) & (MovInstReg[2:1]!=2'b01))|
						(MemReadAR & (MemDST==i))|
						(DRDY & (TAGi[3:0]==i) & (&SZi[1:0]) & ~TAGi[6] & ~TAGi[5] & ~TAGi[4]);
		
		GPROwordNode[i]<=(MovInstReg[0] & (MovInstReg[14:11]==i) & (MovInstReg[2:1]!=2'b01))|
						(FMULRDYR[1] & (FMULDSTR[1]==i))|
						(FDIVRDY & (FDIVDST==i) & FDIVSR[2])|
						(FADDRDYR & (FADDDSTR==i))|
						(MiscRDYR & (MiscDSTR==i) & MiscSR[2])|
						(DRDY & (TAGi[3:0]==i) & ~TAGi[6] & ~TAGi[5] & TAGi[4]);
		end
	// data selection nodes
	for (i=0; i<16; i=i+1)
		begin
		FADDSelNode[i]<=FADDRDYR & (FADDDSTR==i);
		FMULSelNode[i]<=FMULRDYR[0] & (FMULDSTR[0]==i);
		FMULQSelNode[i]<=FMULRDYR[1] & (FMULDSTR[1]==i);
		FMULACCSelNode[i]<=FMULACCRDY & (FMULACCDST==i);
		FFTSelNode[i]<=FMULACCACTReg & FMULACCCMDReg & (FMULACCDSTReg==i);
		FDIVSelNode[i]<=FDIVRDY & (FDIVDST==i);
		ALUSelNode[i]<=ALURDYR & (ALUDSTR==i);
		ShiftSelNode[i]<=ShiftRDYR & (ShiftDSTR==i);
		MiscSelNode[i]<=MiscRDYR & (MiscDSTR==i);
		MovSelNode[i]<=(MovInstReg[0] & (MovInstReg[14:11]==i) & (MovInstReg[2:1]!=2'b01));
		MemSelNode[i]<=DRDY & (TAGi[3:0]==i) & ~TAGi[6] & ~TAGi[5];
		MemFSelNode[i]<=DRDY & (TAGi[3:0]==i) & ~TAGi[6] & TAGi[5] & ~TAGi[4];
		MemASelNode[i]<=DRDY & (TAGi[3:0]==i) & ~TAGi[6] & TAGi[5] & TAGi[4];
		MemARSelNode[i]<=MemReadAR & (MemDST==i);
		end
	// load selector from memory node
	MemLSelNode<=DRDY & ~TAGi[6] & TAGi[0] & TAGi[5] & TAGi[4];

	// DTR selection nodes
	for (i=0; i<8; i=i+1) DTRWriteFlag[i]<=DRDY & ~TAGi[7] & TAGi[6] & ~TAGi[5] &(TAGi[4:2]==i);
	DTRWriteFlag[8]<=DRDY & ~TAGi[7] & TAGi[6] & ~TAGi[5];
	
	// Context loading Flag
	ContextLoadFlag<=CLOAD;	

	// delayed ready registers
	FADDRDYR<=FADDRDY;
	FMULRDYR[0]<=FMULRDY[0];
	FMULRDYR[1]<=FMULRDY[1];
	ALURDYR<=ALURDY;
	ShiftRDYR<=ShiftRDY;
	MiscRDYR<=MiscRDY;
	// delayed destination registers
	FADDDSTR<=FADDDST;
	FMULDSTR[0]<=FMULDST[0];
	FMULDSTR[1]<=FMULDST[1];
	ALUDSTR<=ALUDST;
	ShiftDSTR<=ShiftDST;
	MiscDSTR<=MiscDST;
	// delayed size registers
	FDIVSRReg<=FDIVSR;
	ALUSRReg<=ALUSR;
	ShiftSRReg<=ShiftSR;
	MiscSRReg<=MiscSR;
	SZiReg<=SZi;

	// storing results into registers
	for (i=0; i<16; i=i+1)
		begin
		// storing arithmetic flags
		if (FlagWrite[i])
			begin
			// CF[15:0]
			AFR[i][14:0]<=(ALUCOUT[14:0] & {15{ALUSelNode[i]}}) | (DTi[14:0] & {15{MemFSelNode[i]}});
			AFR[i][15]<=(ALUCOUT[15] & ALUSelNode[i])|(MiscCOUT & MiscSelNode[i])|(DTi[15] & MemFSelNode[i]);
			// ZF
			AFR[i][16]<=(FADDZero & FADDSelNode[i])|(FMULZero[0] & FMULSelNode[i])|(FMULZero[1] & FMULQSelNode[i])|
						(FDIVZero & FDIVSelNode[i])|(ALUZero & ALUSelNode[i])|(ShiftZERO & ShiftSelNode[i])|
						(FMULACCZERO & FMULACCSelNode[i])|(MiscZero & MiscSelNode[i])|(DTi[16] & MemFSelNode[i])|
						(FFT32NEXTReg & FFTSelNode[i]);
			// SF
			AFR[i][17]<=(FADDSign & FADDSelNode[i])|(FMULSign[0] & FMULSelNode[i])|(FMULSign[1] & FMULQSelNode[i])|
						(FDIVSign & FDIVSelNode[i])|(ALUSign & ALUSelNode[i])|(ShiftSign & ShiftSelNode[i])|
						(FMULACCSIGN & FMULACCSelNode[i])|(MiscSign & MiscSelNode[i])|(DTi[17] & MemFSelNode[i]);
			// OF
			AFR[i][18]<=(ALUOVR & ALUSelNode[i])|(ShiftOVR & ShiftSelNode[i])|(DTi[18] & MemFSelNode[i]);
			// IF
			AFR[i][19]<=(FADDInf & FADDSelNode[i])|(FMULInf[0] & FMULSelNode[i])|(FMULInf[1] & FMULQSelNode[i])|
						(FDIVInf & FDIVSelNode[i])|(MiscOVR & MiscSelNode[i])|(DTi[19] & MemFSelNode[i])|
						(FMULACCINF & FMULACCSelNode[i]);
			// NF
			AFR[i][20]<=(FADDNaN & FADDSelNode[i])|(FMULNaN[0] & FMULSelNode[i])|(FMULNaN[1] & FMULQSelNode[i])|
						(FDIVNaN & FDIVSelNode[i])|(MiscNaN & MiscSelNode[i])|(DTi[20] & MemFSelNode[i])|
						(FMULACCNAN & FMULACCSelNode[i])|(MemSelNode[i] & (&SZiReg));
			// DBF
			AFR[i][21]<=(ShiftCOUT & ShiftSelNode[i])|(DTi[21] & MemFSelNode[i]);
			end
		// Size of operand and address mode
		if (MemFSelNode[i]) AFR[i][27:22]<=DTi[27:22];
			else if (MemSelNode[i]) AFR[i][24:22]<=TAGiReg[7] ? 3'd4 : {SZiReg};
				else if (MovSelNode[i]) AFR[i][24:22]<=MovSR;
					else if (MemARSelNode[i]) AFR[i][24:22]<=3'b011;
						else if (FADDSelNode[i] | FMULSelNode[i] | FMULQSelNode[i] | FDIVSelNode[i] | ALUSelNode[i] | ShiftSelNode[i] | MiscSelNode[i] | FMULACCSelNode[i])
							begin
							AFR[i][24]<=FMULQSelNode[i] | (FADDSR[2] & FADDSelNode[i]) | (MiscSRReg[2] & MiscSelNode[i]) | (FDIVSRReg[2] & FDIVSelNode[i]);
							AFR[i][23:22]<=(FADDSR[1:0] & {2{FADDSelNode[i]}})|
										({1'b1, FMULSR} & {2{FMULSelNode[i]}})|
										(FDIVSRReg[1:0] & {2{FDIVSelNode[i]}})|
										(ALUSRReg & {2{ALUSelNode[i]}})|
										(ShiftSRReg & {2{ShiftSelNode[i]}})|
										(MiscSRReg[1:0] & {2{MiscSelNode[i]}})| {FMULACCSelNode[i], 1'b0};
							end
		//LI byte counter
		if (LIResetFlag[i] | LIRESET) AFR[i][31:28]<=4'b0000;
			else if (MemFSelNode[i]) AFR[i][31:28]<=DTi[31:28] & {4{ContextLoadFlag}};
				else if (LIFlag[i]) AFR[i][31:28]<=AFR[i][31:28]+4'd1;

		// GPR Bits 7:0
		if (GPRByteNode[i]) GPR[i][7:0]<=(FADDR[7:0] & {8{FADDSelNode[i]}})|
											(FMULR[7:0] & {8{FMULSelNode[i]}})|
											(FMULRQ[7:0] & {8{FMULQSelNode[i]}})|
											(FMULACCR[7:0] & {8{FMULACCSelNode[i]}})|
											(FDIVR[7:0] & {8{FDIVSelNode[i]}})|
											(ALUR[7:0] & {8{ALUSelNode[i]}})|
											(ShiftR[7:0] & {8{ShiftSelNode[i]}})|
											(MiscR[7:0] & {8{MiscSelNode[i]}})|
											(MovReg[7:0] & {8{MovSelNode[i]}})|
											(DTi[7:0] & {8{MemSelNode[i]}})|
											(ARData[7:0] & {8{MemARSelNode[i]}});
		// GPR Bits 15:8
		if (GPRWordNode[i]) GPR[i][15:8]<=(FADDR[15:8] & {8{FADDSelNode[i]}})|
											(FMULR[15:8] & {8{FMULSelNode[i]}})|
											(FMULRQ[15:8] & {8{FMULQSelNode[i]}})|
											(FMULACCR[15:8] & {8{FMULACCSelNode[i]}})|
											(FDIVR[15:8] & {8{FDIVSelNode[i]}})|
											(ALUR[15:8] & {8{ALUSelNode[i]}})|
											(ShiftR[15:8] & {8{ShiftSelNode[i]}})|
											(MiscR[15:8] & {8{MiscSelNode[i]}})|
											(MovReg[15:8] & {8{MovSelNode[i]}})|
											(DTi[15:8] & {8{MemSelNode[i]}})|
											(ARData[15:8] & {8{MemARSelNode[i]}});

		if (GPRDwordNode[i]) GPR[i][31:16]<=(FADDR[31:16] & {16{FADDSelNode[i]}})|
											(FMULR[31:16] & {16{FMULSelNode[i]}})|
											(FMULRQ[31:16] & {16{FMULQSelNode[i]}})|
											(FMULACCR[31:16] & {16{FMULACCSelNode[i]}})|
											(FDIVR[31:16] & {16{FDIVSelNode[i]}})|
											(ALUR[31:16] & {16{ALUSelNode[i]}})|
											(ShiftR[31:16] & {16{ShiftSelNode[i]}})|
											(MiscR[31:16] & {16{MiscSelNode[i]}})|
											(MovReg[31:16] & {16{MovSelNode[i]}})|
											(DTi[31:16] & {16{MemSelNode[i]}})|
											(ARData[31:16] & {16{MemARSelNode[i]}});

		if (GPRQwordNode[i]) GPR[i][63:32]<=(FADDR[63:32] & {32{FADDSelNode[i]}})|
											(FMULR[63:32] & {32{FMULSelNode[i]}})|
											(FMULRQ[63:32] & {32{FMULQSelNode[i]}})|
											(FDIVR[63:32] & {32{FDIVSelNode[i]}})|
											(ALUR[63:32] & {32{ALUSelNode[i]}})|
											(ShiftR[63:32] & {32{ShiftSelNode[i]}})|
											(MiscR[63:32] & {32{MiscSelNode[i]}})|
											(MovReg[63:32] & {32{MovSelNode[i]}})|
											(DTi[63:32] & {32{MemSelNode[i]}})|
											(ARData[36:32] & {32{MemARSelNode[i]}});
		
		if (GPROwordNode[i]) GPR[i][127:64]<=(MovReg[127:64] & {64{MovSelNode[i]}})|
											(DTi & {64{MemSelNode[i]}})|
											(FMULRQ[127:64] & {64{FMULQSelNode[i]}})|
											(FDIVR[127:64] & {64{FDIVSelNode[i]}})|
											(FADDR[127:64] & {64{FADDSelNode[i]}}) |
											(MiscR[127:64] & {64{MiscSelNode[i]}});
		end

	// processing source operands
	// FADD Channel
	FADDInstReg<=FADDInstBus;
	FADDACTReg<=FADDInstReg[0];
	FADDCMDReg<=FADDInstReg[1];
	FADDBypassAReg[0]<=FADDRDYR & (FADDDSTR==FADDInstBus[5:2]);
	FADDBypassAReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FADDInstBus[5:2]);
	FADDBypassAReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FADDInstBus[5:2]);
	FADDBypassAReg[3]<=ALURDYR & (ALUDSTR==FADDInstBus[5:2]);
	FADDBypassAReg[4]<=ShiftRDYR & (ShiftDSTR==FADDInstBus[5:2]);
	FADDBypassAReg[5]<=MiscRDYR & (MiscDSTR==FADDInstBus[5:2]);
	FADDAReg<=(GPR[FADDInstReg[5:2]] & {128{~(|FADDBypassAReg)}}) |
				(FADDR & {128{FADDBypassAReg[0]}}) |
				({64'd0, FMULR & {64{FADDBypassAReg[1]}}}) |
				(FMULRQ & {128{FADDBypassAReg[2]}}) |
				({GPR[FADDInstReg[5:2]][127:64], ALUR} & {128{FADDBypassAReg[3]}}) |
				({GPR[FADDInstReg[5:2]][127:64], ShiftR} & {128{FADDBypassAReg[4]}}) |
				(MiscR & {128{FADDBypassAReg[5]}});
	FADDSAReg<=(AFR[FADDInstReg[5:2]][24:22] & {3{~(|FADDBypassAReg)}}) |
				(FADDSR & {3{FADDBypassAReg[0]}})|
				({2'd1, FMULSR} & {3{FADDBypassAReg[1]}})|
				({FADDBypassAReg[2], 2'd0}) |
				({1'b0, ALUSRReg & {2{FADDBypassAReg[3]}}})|
				({1'b0, ShiftSRReg & {2{FADDBypassAReg[4]}}})|
				(MiscSRReg & {3{FADDBypassAReg[5]}});
	FADDBypassBReg[0]<=FADDRDYR & (FADDDSTR==FADDInstBus[9:6]);
	FADDBypassBReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FADDInstBus[9:6]);
	FADDBypassBReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FADDInstBus[9:6]);
	FADDBypassBReg[3]<=ALURDYR & (ALUDSTR==FADDInstBus[9:6]);
	FADDBypassBReg[4]<=ShiftRDYR & (ShiftDSTR==FADDInstBus[9:6]);
	FADDBypassBReg[5]<=MiscRDYR & (MiscDSTR==FADDInstBus[9:6]);
	FADDBReg<=(GPR[FADDInstReg[9:6]] & {128{~(|FADDBypassBReg)}}) |
				(FADDR & {128{FADDBypassBReg[0]}}) |
				({64'd0, FMULR & {64{FADDBypassBReg[1]}}}) |
				(FMULRQ & {128{FADDBypassBReg[2]}}) |
				({GPR[FADDInstReg[9:6]][127:64], ALUR} & {128{FADDBypassBReg[3]}}) |
				({GPR[FADDInstReg[9:6]][127:64], ShiftR} & {128{FADDBypassBReg[4]}}) |
				(MiscR & {128{FADDBypassBReg[5]}});
	FADDSBReg<=(AFR[FADDInstReg[9:6]][24:22] & {3{~(|FADDBypassBReg)}})|
				(FADDSR & {3{FADDBypassBReg[0]}})|
				({2'd1, FMULSR} & {3{FADDBypassBReg[1]}})|
				({FADDBypassBReg[2], 2'd0}) |
				({1'b0, ALUSRReg & {2{FADDBypassBReg[3]}}})|
				({1'b0, ShiftSRReg & {2{FADDBypassBReg[4]}}})|
				(MiscSRReg & {3{FADDBypassBReg[5]}});
	FADDDSTReg<=FADDInstReg[13:10];
	
	// FMUL channel
	FMULInstReg<=FMULInstBus;
	FMULACTReg<=FMULInstReg[0];
	FMULBypassAReg[0]<=FADDRDYR & (FADDDSTR==FMULInstBus[4:1]);
	FMULBypassAReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FMULInstBus[4:1]);
	FMULBypassAReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FMULInstBus[4:1]);
	FMULBypassAReg[3]<=ALURDYR & (ALUDSTR==FMULInstBus[4:1]);
	FMULBypassAReg[4]<=ShiftRDYR & (ShiftDSTR==FMULInstBus[4:1]);
	FMULBypassAReg[5]<=MiscRDYR & (MiscDSTR==FMULInstBus[4:1]);
	FMULAReg<=(GPR[FMULInstReg[4:1]] & {128{~(|FMULBypassAReg)}})|
				(FADDR & {128{FMULBypassAReg[0]}}) |
				({64'd0, FMULR & {64{FMULBypassAReg[1]}}}) |
				(FMULRQ & {128{FMULBypassAReg[2]}}) |
				({GPR[FMULInstReg[4:1]][127:64], ALUR} & {128{FMULBypassAReg[3]}}) |
				({GPR[FMULInstReg[4:1]][127:64], ShiftR} & {128{FMULBypassAReg[4]}}) |
				(MiscR & {128{FMULBypassAReg[5]}});
	FMULSAReg<=(AFR[FMULInstReg[4:1]][24:22] & {3{~(|FMULBypassAReg)}})|
				(FADDSR & {3{FMULBypassAReg[0]}})|
				({2'd1, FMULSR} & {3{FMULBypassAReg[1]}})|
				({FMULBypassAReg[2], 2'd0}) |
				({1'b0, ALUSRReg & {2{FMULBypassAReg[3]}}})|
				({1'b0, ShiftSRReg & {2{FMULBypassAReg[4]}}})|
				(MiscSRReg & {3{FMULBypassAReg[5]}});
	FMULBypassBReg[0]<=FADDRDYR & (FADDDSTR==FMULInstBus[8:5]);
	FMULBypassBReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FMULInstBus[8:5]);
	FMULBypassBReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FMULInstBus[8:5]);
	FMULBypassBReg[3]<=ALURDYR & (ALUDSTR==FMULInstBus[8:5]);
	FMULBypassBReg[4]<=ShiftRDYR & (ShiftDSTR==FMULInstBus[8:5]);
	FMULBypassBReg[5]<=MiscRDYR & (MiscDSTR==FMULInstBus[8:5]);
	FMULBReg<=(GPR[FMULInstReg[8:5]] & {128{~(|FMULBypassBReg)}})|
				(FADDR & {128{FMULBypassBReg[0]}}) |
				({64'd0, FMULR & {64{FMULBypassBReg[1]}}}) |
				(FMULRQ & {128{FMULBypassBReg[2]}}) |
				({GPR[FMULInstReg[8:5]][127:64], ALUR} & {128{FMULBypassBReg[3]}}) |
				({GPR[FMULInstReg[8:5]][127:64], ShiftR} & {128{FMULBypassBReg[4]}}) |
				(MiscR & {128{FMULBypassBReg[5]}});
	FMULSBReg<=(AFR[FMULInstReg[8:5]][24:22] & {3{~(|FMULBypassBReg)}})|
				(FADDSR & {3{FMULBypassBReg[0]}})|
				({2'd1, FMULSR} & {3{FMULBypassBReg[1]}})|
				({FMULBypassBReg[2], 2'd0}) |
				({1'b0, ALUSRReg & {2{FMULBypassBReg[3]}}})|
				({1'b0, ShiftSRReg & {2{FMULBypassBReg[4]}}})|
				(MiscSRReg & {3{FMULBypassBReg[5]}});
	FMULDSTReg<=FMULInstReg[12:9];

//=================================================================================================
//							FMULACC Channel
//0 - ACT, 3:1 - ctrlSel, 7:4 - DST, 11:8 - CtrlOffsetReg, 15:12 - DataOffsetReg, 18:16 - DataSel, 19 - CMD 0-FMULLACC/1-FFT
	FMULACCReg[0]<=(FMULACCReg[0] | FMULACCBus[0]) & (~FMULACCReg[0] | (FMULACCACTReg & ~FMULACCNEXT)) & RESET;
	if (~FMULACCReg[0]) FMULACCReg[19:1]<=FMULACCBus[19:1];
	
	FMULACCACTReg<=((FMULACCACTReg & ~FMULACCNEXT) | FMULACCReg[0]) & RESET;
	
	FFT32NEXTReg<=FFT32NEXT;
	
	FFTBypassReg[0]<=FADDRDYR & (FADDDSTR==FMULACCBus[7:4]);
	FFTBypassReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FMULACCBus[7:4]);
	FFTBypassReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FMULACCBus[7:4]);
	FFTBypassReg[3]<=ALURDYR & (ALUDSTR==FMULACCBus[7:4]);
	FFTBypassReg[4]<=ShiftRDYR & (ShiftDSTR==FMULACCBus[7:4]);
	FFTBypassReg[5]<=MiscRDYR & (MiscDSTR==FMULACCBus[7:4]);

	FMULACCBypassControlOffsetReg[0]<=FADDRDYR & (FADDDSTR==FMULACCBus[11:8]);
	FMULACCBypassControlOffsetReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FMULACCBus[11:8]);
	FMULACCBypassControlOffsetReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FMULACCBus[11:8]);
	FMULACCBypassControlOffsetReg[3]<=ALURDYR & (ALUDSTR==FMULACCBus[11:8]);
	FMULACCBypassControlOffsetReg[4]<=ShiftRDYR & (ShiftDSTR==FMULACCBus[11:8]);
	FMULACCBypassControlOffsetReg[5]<=MiscRDYR & (MiscDSTR==FMULACCBus[11:8]);

	FMULACCBypassDataOffsetReg[0]<=FADDRDYR & (FADDDSTR==FMULACCBus[15:12]);
	FMULACCBypassDataOffsetReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FMULACCBus[15:12]);
	FMULACCBypassDataOffsetReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FMULACCBus[15:12]);
	FMULACCBypassDataOffsetReg[3]<=ALURDYR & (ALUDSTR==FMULACCBus[15:12]);
	FMULACCBypassDataOffsetReg[4]<=ShiftRDYR & (ShiftDSTR==FMULACCBus[15:12]);
	FMULACCBypassDataOffsetReg[5]<=MiscRDYR & (MiscDSTR==FMULACCBus[15:12]);

	if (FMULACCNEXT | ~FMULACCACTReg)
		begin
		FMULACCDSTReg<=FMULACCReg[7:4];
		FMULACCCtrlSelReg<=FMULACCReg[3:1];
		FMULACCDataSelReg<=FMULACCReg[18:16];
		
		FMULACCCMDReg<=FMULACCReg[19];
		
		FFTCtrlBaseReg<=ADR[{FMULACCReg[3:1],1'b0}][36:3];
		FFTDataBaseReg<=ADR[{FMULACCReg[18:16],1'b0}][36:3];
		FFTCtrlSelReg<=ADR[{FMULACCReg[3:1],1'b1}][31:0];
		FFTDataSelReg<=ADR[{FMULACCReg[18:16],1'b1}][31:0];
		
		FFTParReg<=(GPR[FMULACCReg[7:4]][4:0] & {5{~(|FFTBypassReg)}})|
					(FADDR[4:0] & {5{FFTBypassReg[0]}}) |
					(FMULR[4:0] & {5{FFTBypassReg[1]}}) |
					(FMULRQ[4:0] & {5{FFTBypassReg[2]}}) |
					(ALUR[4:0] & {5{FFTBypassReg[3]}}) |
					(ShiftR[4:0] & {5{FFTBypassReg[4]}}) |
					(MiscR[4:0] & {5{FFTBypassReg[5]}});
					
		FFTIndexReg<=(GPR[FMULACCReg[7:4]][50:32] & {19{~(|FFTBypassReg)}})|
					(FADDR[50:32] & {19{FFTBypassReg[0]}}) |
					(FMULR[50:32] & {19{FFTBypassReg[1]}}) |
					(FMULRQ[50:32] & {19{FFTBypassReg[2]}}) |
					(ALUR[50:32] & {19{FFTBypassReg[3]}}) |
					(ShiftR[50:32] & {19{FFTBypassReg[4]}}) |
					(MiscR[50:32] & {19{FFTBypassReg[5]}});
		
		FMULACCCtrlOffsetReg<=(GPR[FMULACCReg[11:8]][36:3] & {34{~(|FMULACCBypassControlOffsetReg)}})|
					(FADDR[36:3] & {34{FMULACCBypassControlOffsetReg[0]}}) |
					(FMULR[36:3] & {34{FMULACCBypassControlOffsetReg[1]}}) |
					(FMULRQ[36:3] & {34{FMULACCBypassControlOffsetReg[2]}}) |
					(ALUR[36:3] & {34{FMULACCBypassControlOffsetReg[3]}}) |
					(ShiftR[36:3] & {34{FMULACCBypassControlOffsetReg[4]}}) |
					(MiscR[36:3] & {34{FMULACCBypassControlOffsetReg[5]}});
		
		FMULACCDataOffsetReg<=(GPR[FMULACCReg[15:12]][36:2] & {35{~(|FMULACCBypassDataOffsetReg)}})|
					(FADDR[36:2] & {35{FMULACCBypassDataOffsetReg[0]}}) |
					(FMULR[36:2] & {35{FMULACCBypassDataOffsetReg[1]}}) |
					(FMULRQ[36:2] & {35{FMULACCBypassDataOffsetReg[2]}}) |
					(ALUR[36:2] & {35{FMULACCBypassDataOffsetReg[3]}}) |
					(ShiftR[36:2] & {35{FMULACCBypassDataOffsetReg[4]}}) |
					(MiscR[36:2] & {35{FMULACCBypassDataOffsetReg[5]}});
		end
	FMULACCDRDYFlag<=DRDY & (TAGi[7:1]==7'b1100000);

	//FDIV channel
	FDIVInstReg[0]<=(FDIVInstReg[0] | FDIVInstBus[0]) & (~FDIVInstReg[0] | (FDIVACTReg & ~FDIVNEXT));
	if (~FDIVInstReg[0]) FDIVInstReg[13:1]<=FDIVInstBus[13:1];
	FDIVACTReg<=(FDIVACTReg & ~FDIVNEXT) | FDIVInstReg[0];
	FDIVBypassAReg[0]<=FADDRDYR & (FADDDSTR==FDIVInstBus[4:1]);
	FDIVBypassAReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FDIVInstBus[4:1]);
	FDIVBypassAReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FDIVInstBus[4:1]);
	FDIVBypassAReg[3]<=ALURDYR & (ALUDSTR==FDIVInstBus[4:1]);
	FDIVBypassAReg[4]<=ShiftRDYR & (ShiftDSTR==FDIVInstBus[4:1]);
	FDIVBypassAReg[5]<=MiscRDYR & (MiscDSTR==FDIVInstBus[4:1]);
	FDIVBypassBReg[0]<=FADDRDYR & (FADDDSTR==FDIVInstBus[8:5]);
	FDIVBypassBReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==FDIVInstBus[8:5]);
	FDIVBypassBReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==FDIVInstBus[8:5]);
	FDIVBypassBReg[3]<=ALURDYR & (ALUDSTR==FDIVInstBus[8:5]);
	FDIVBypassBReg[4]<=ShiftRDYR & (ShiftDSTR==FDIVInstBus[8:5]);
	FDIVBypassBReg[5]<=MiscRDYR & (MiscDSTR==FDIVInstBus[8:5]);
	if (FDIVNEXT | ~FDIVACTReg)
		begin
		FDIVAReg<=(GPR[FDIVInstReg[4:1]] & {128{~(|FDIVBypassAReg)}})|
					(FADDR & {128{FDIVBypassAReg[0]}}) |
					({64'd0, FMULR & {64{FDIVBypassAReg[1]}}}) |
					(FMULRQ & {128{FDIVBypassAReg[2]}}) |
					({GPR[FDIVInstReg[4:1]][127:64], ALUR} & {128{FDIVBypassAReg[3]}}) |
					({GPR[FDIVInstReg[4:1]][127:64], ShiftR} & {128{FDIVBypassAReg[4]}}) |
					(MiscR & {128{FDIVBypassAReg[5]}});
		FDIVSAReg<=(AFR[FDIVInstReg[4:1]][24:22] & {3{~(|FDIVBypassAReg)}})|
					(FADDSR & {3{FDIVBypassAReg[0]}})|
					({2'd1, FMULSR} & {3{FDIVBypassAReg[1]}})|
					({FDIVBypassAReg[2], 2'd0}) |
					({1'b0, ALUSRReg & {2{FDIVBypassAReg[3]}}})|
					({1'b0, ShiftSRReg & {2{FDIVBypassAReg[4]}}})|
					(MiscSRReg & {3{FDIVBypassAReg[5]}});
		FDIVBReg<=(GPR[FDIVInstReg[8:5]] & {128{~(|FDIVBypassBReg)}})|
					(FADDR & {128{FDIVBypassBReg[0]}}) |
					({64'd0, FMULR & {64{FDIVBypassBReg[1]}}}) |
					(FMULRQ & {128{FDIVBypassBReg[2]}}) |
					({GPR[FDIVInstReg[8:5]][127:64], ALUR} & {128{FDIVBypassBReg[3]}}) |
					({GPR[FDIVInstReg[8:5]][127:64], ShiftR} & {128{FDIVBypassBReg[4]}}) |
					(MiscR & {128{FDIVBypassBReg[5]}});
		FDIVSBReg<=(AFR[FDIVInstReg[8:5]][24:22] & {3{~(|FDIVBypassBReg)}})|
					(FADDSR & {3{FDIVBypassBReg[0]}})|
					({2'd1, FMULSR} & {3{FDIVBypassBReg[1]}})|
					({FDIVBypassBReg[2], 2'd0}) |
					({1'b0, ALUSRReg & {2{FDIVBypassBReg[3]}}})|
					({1'b0, ShiftSRReg & {2{FDIVBypassBReg[4]}}})|
					(MiscSRReg & {3{FDIVBypassBReg[5]}});
		FDIVDSTReg<=FDIVInstReg[12:9];
		FDIVCMDReg<=FDIVInstReg[13];
		end

	// ALU channel
	ALUInstReg<=ALUInstBus;
	ALUACTReg<=ALUInstReg[0];
	ALUOpCODEReg<={ALUInstReg[24], ALUInstReg[2:1]};

	ALUBypassAReg[0]<=FADDRDYR & (FADDDSTR==ALUInstBus[6:3]);
	ALUBypassAReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==ALUInstBus[6:3]);
	ALUBypassAReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==ALUInstBus[6:3]);
	ALUBypassAReg[3]<=ALURDYR & (ALUDSTR==ALUInstBus[6:3]);
	ALUBypassAReg[4]<=ShiftRDYR & (ShiftDSTR==ALUInstBus[6:3]);
	ALUBypassAReg[5]<=MiscRDYR & (MiscDSTR==ALUInstBus[6:3]);

	ALUAReg<=ALUInstReg[15] ? {58'd0, ALUInstReg[8:3]} :
				((GPR[ALUInstReg[6:3]][63:0] & {64{~(|ALUBypassAReg)}}) | (FADDR[63:0] & {64{ALUBypassAReg[0]}}) |
				(FMULR & {64{ALUBypassAReg[1]}}) | (FMULRQ[63:0] & {64{ALUBypassAReg[2]}}) |
				(ALUR & {64{ALUBypassAReg[3]}}) | (ShiftR & {64{ALUBypassAReg[4]}}) | (MiscR[63:0] & {64{ALUBypassAReg[5]}}));

	ALUSAReg<=((AFR[ALUInstReg[6:3]][23:22] & {2{~(|ALUBypassAReg)}}) | (FADDSR[1:0] & {2{ALUBypassAReg[0]}}) |
					({1'b1, FMULSR} & {2{ALUBypassAReg[1]}}) | (ALUSRReg & {2{ALUBypassAReg[3]}}) |
					(ShiftSRReg & {2{ALUBypassAReg[4]}}) | (MiscSRReg[1:0] & {2{ALUBypassAReg[5]}})) & {2{~ALUInstReg[15]}};
	
	ALUBypassBReg[0]<=FADDRDYR & (FADDDSTR==ALUInstBus[10:7]);
	ALUBypassBReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==ALUInstBus[10:7]);
	ALUBypassBReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==ALUInstBus[10:7]);
	ALUBypassBReg[3]<=ALURDYR & (ALUDSTR==ALUInstBus[10:7]);
	ALUBypassBReg[4]<=ShiftRDYR & (ShiftDSTR==ALUInstBus[10:7]);
	ALUBypassBReg[5]<=MiscRDYR & (MiscDSTR==ALUInstBus[10:7]);

	ALUBReg<=ALUInstReg[15] ? {58'd0, ALUInstReg[14:9]} : 
				((GPR[ALUInstReg[10:7]][63:0] & {64{~(|ALUBypassBReg)}}) | (FADDR[63:0] & {64{ALUBypassBReg[0]}}) |
				(FMULR & {64{ALUBypassBReg[1]}}) | (FMULRQ[63:0] & {64{ALUBypassBReg[2]}}) |
				(ALUR & {64{ALUBypassBReg[3]}}) | (ShiftR & {64{ALUBypassBReg[4]}}) | (MiscR[63:0] & {64{ALUBypassBReg[5]}}));

	ALUSBReg<=((AFR[ALUInstReg[10:7]][23:22] & {2{~(|ALUBypassBReg)}}) | (FADDSR[1:0] & {2{ALUBypassBReg[0]}}) |
					({1'b1, FMULSR} & {2{ALUBypassBReg[1]}}) | (ALUSRReg & {2{ALUBypassBReg[3]}}) |
					(ShiftSRReg & {2{ALUBypassBReg[4]}}) | (MiscSRReg[1:0] & {2{ALUBypassBReg[5]}})) & {2{~ALUInstReg[15]}};

	ALUBypassCReg[0]<=FADDRDYR & (FADDDSTR==ALUInstBus[19:16]);
	ALUBypassCReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==ALUInstBus[19:16]);
	ALUBypassCReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==ALUInstBus[19:16]);
	ALUBypassCReg[3]<=ALURDYR & (ALUDSTR==ALUInstBus[19:16]);
	ALUBypassCReg[4]<=ShiftRDYR & (ShiftDSTR==ALUInstBus[19:16]);
	ALUBypassCReg[5]<=MiscRDYR & (MiscDSTR==ALUInstBus[19:16]);

	ALUCReg<=(GPR[ALUInstReg[19:16]][63:0] & {64{~(|ALUBypassCReg)}}) | (FADDR[63:0] & {64{ALUBypassCReg[0]}}) |
				(FMULR & {64{ALUBypassCReg[1]}}) | (FMULRQ[63:0] & {64{ALUBypassCReg[2]}}) |
				(ALUR & {64{ALUBypassCReg[3]}}) | (ShiftR & {64{ALUBypassCReg[4]}}) | (MiscR[63:0] & {64{ALUBypassCReg[5]}});

	ALUSCReg<=((AFR[ALUInstReg[19:16]][23:22] & {2{~(|ALUBypassCReg)}}) | (FADDSR[1:0] & {2{ALUBypassCReg[0]}}) |
					({1'b1, FMULSR} & {2{ALUBypassCReg[1]}}) | (ALUSRReg & {2{ALUBypassCReg[3]}}) |
					(ShiftSRReg & {2{ALUBypassCReg[4]}}) | (MiscSRReg[1:0] & {2{ALUBypassCReg[5]}})) & {2{ALUInstReg[24]}};

	ALUBypassDReg[0]<=FADDRDYR & (FADDDSTR==ALUInstBus[23:20]);
	ALUBypassDReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==ALUInstBus[23:20]);
	ALUBypassDReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==ALUInstBus[23:20]);
	ALUBypassDReg[3]<=ALURDYR & (ALUDSTR==ALUInstBus[23:20]);
	ALUBypassDReg[4]<=ShiftRDYR & (ShiftDSTR==ALUInstBus[23:20]);
	ALUBypassDReg[5]<=MiscRDYR & (MiscDSTR==ALUInstBus[23:20]);

	ALUDReg<=(GPR[ALUInstReg[23:20]][63:0] & {64{~(|ALUBypassDReg)}}) | (FADDR[63:0] & {64{ALUBypassDReg[0]}}) |
				(FMULR & {64{ALUBypassDReg[1]}}) | (FMULRQ[63:0] & {64{ALUBypassDReg[2]}}) |
				(ALUR & {64{ALUBypassDReg[3]}}) | (ShiftR & {64{ALUBypassDReg[4]}}) | (MiscR[63:0] & {64{ALUBypassDReg[5]}});

	ALUSDReg<=((AFR[ALUInstReg[23:20]][23:22] & {2{~(|ALUBypassDReg)}}) | (FADDSR[1:0] & {2{ALUBypassDReg[0]}}) |
					({1'b1, FMULSR} & {2{ALUBypassDReg[1]}}) | (ALUSRReg & {2{ALUBypassDReg[3]}}) |
					(ShiftSRReg & {2{ALUBypassDReg[4]}}) | (MiscSRReg[1:0] & {2{ALUBypassDReg[5]}})) & {2{ALUInstReg[24]}};

	ALUDSTReg<=ALUInstReg[24] ? ALUInstReg[23:20] : ALUInstReg[14:11];

	// Shifter channel
	ShiftInstReg<=ShiftInstBus;
	ShiftACTReg<=ShiftInstReg[0];
	
	ShiftBypassAReg[0]<=FADDRDYR & (FADDDSTR==ShiftInstBus[12:9]);
	ShiftBypassAReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==ShiftInstBus[12:9]);
	ShiftBypassAReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==ShiftInstBus[12:9]);
	ShiftBypassAReg[3]<=ALURDYR & (ALUDSTR==ShiftInstBus[12:9]);
	ShiftBypassAReg[4]<=ShiftRDYR & (ShiftDSTR==ShiftInstBus[12:9]);
	ShiftBypassAReg[5]<=MiscRDYR & (MiscDSTR==ShiftInstBus[12:9]);
	
	ShiftAReg<=(GPR[ShiftInstReg[12:9]][63:0] & {64{~(|ShiftBypassAReg)}}) | 
				(FADDR[63:0] & {64{ShiftBypassAReg[0]}}) |
				(FMULR & {64{ShiftBypassAReg[1]}}) | 
				(FMULRQ[63:0] & {64{ShiftBypassAReg[2]}}) |
				(ALUR & {64{ShiftBypassAReg[3]}}) |
				(ShiftR & {64{ShiftBypassAReg[4]}}) |
				(MiscR[63:0] & {64{ShiftBypassAReg[5]}});

	ShiftSAReg<=(AFR[ShiftInstReg[12:9]][23:22] & {2{~(|ShiftBypassAReg)}}) |
				(FADDSR[1:0] & {2{ShiftBypassAReg[0]}}) |
				({1'b1, FMULSR} & {2{ShiftBypassAReg[1]}}) |
				(ALUSRReg & {2{ShiftBypassAReg[3]}}) |
				(ShiftSRReg & {2{ShiftBypassAReg[4]}}) |
				(MiscSRReg[1:0] & {2{ShiftBypassAReg[5]}});

	ShiftBypassBReg[0]<=FADDRDYR & (FADDDSTR==ShiftInstBus[8:5]);
	ShiftBypassBReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==ShiftInstBus[8:5]);
	ShiftBypassBReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==ShiftInstBus[8:5]);
	ShiftBypassBReg[3]<=ALURDYR & (ALUDSTR==ShiftInstBus[8:5]);
	ShiftBypassBReg[4]<=ShiftRDYR & (ShiftDSTR==ShiftInstBus[8:5]);
	ShiftBypassBReg[5]<=MiscRDYR & (MiscDSTR==ShiftInstBus[8:5]);

	ShiftBReg<=ShiftInstReg[4] ? ((GPR[ShiftInstReg[8:5]][5:0] & {6{~(|ShiftBypassBReg)}}) | 
								(FADDR[5:0] & {6{ShiftBypassBReg[0]}}) |
								(FMULR[5:0] & {6{ShiftBypassBReg[1]}}) | 
								(FMULRQ[5:0] & {6{ShiftBypassBReg[2]}}) |
								(ALUR[5:0] & {6{ShiftBypassBReg[3]}}) |
								(ShiftR[5:0] & {6{ShiftBypassBReg[4]}}) |
								(MiscR[5:0] & {6{ShiftBypassBReg[5]}})) : {2'b00, ShiftInstReg[8:5]};							

	ShiftDSTReg<=ShiftInstReg[12:9];
	ShiftOPRReg<=ShiftInstReg[3:1];

	// miscellaneous operations channel
	MiscInstReg<=MiscInstBus;
	MiscACTReg<=MiscInstReg[0];
	
	MiscBypassAReg[0]<=FADDRDYR & (FADDDSTR==MiscInstBus[7:4]);
	MiscBypassAReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==MiscInstBus[7:4]);
	MiscBypassAReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==MiscInstBus[7:4]);
	MiscBypassAReg[3]<=ALURDYR & (ALUDSTR==MiscInstBus[7:4]);
	MiscBypassAReg[4]<=ShiftRDYR & (ShiftDSTR==MiscInstBus[7:4]);
	MiscBypassAReg[5]<=MiscRDYR & (MiscDSTR==MiscInstBus[7:4]);

	MiscBypassBReg[0]<=FADDRDYR & (FADDDSTR==MiscInstBus[11:8]);
	MiscBypassBReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==MiscInstBus[11:8]);
	MiscBypassBReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==MiscInstBus[11:8]);
	MiscBypassBReg[3]<=ALURDYR & (ALUDSTR==MiscInstBus[11:8]);
	MiscBypassBReg[4]<=ShiftRDYR & (ShiftDSTR==MiscInstBus[11:8]);
	MiscBypassBReg[5]<=MiscRDYR & (MiscDSTR==MiscInstBus[11:8]);
	
	MiscCINReg<=(AFR[MiscInstReg[11:8]][15:0] & {16{~MiscBypassBReg[3]}}) | (ALUCOUT & {16{MiscBypassBReg[3]}});

	MiscAReg<=(MiscInstReg[3] & (~MiscInstReg[2] | ~MiscInstReg[1])) ? ((GPR[MiscInstReg[7:4]] & {128{~(|MiscBypassAReg)}}) | 
																		(FADDR & {128{MiscBypassAReg[0]}}) |
																		({64'd0, FMULR & {64{MiscBypassAReg[1]}}}) |
																		(FMULRQ & {128{MiscBypassAReg[2]}}) |
																		({GPR[MiscInstReg[7:4]][127:64], ALUR} & {128{MiscBypassAReg[3]}}) |
																		({GPR[MiscInstReg[7:4]][127:64], ShiftR} & {128{MiscBypassAReg[4]}}) |
																		(MiscR & {128{MiscBypassAReg[5]}})) :
																		((GPR[MiscInstReg[11:8]] & {128{~(|MiscBypassBReg)}}) | 
																		(FADDR & {128{MiscBypassBReg[0]}}) |
																		({64'd0, FMULR & {64{MiscBypassBReg[1]}}}) |
																		(FMULRQ & {128{MiscBypassBReg[2]}}) |
																		({GPR[MiscInstReg[11:8]][127:64], ALUR} & {128{MiscBypassBReg[3]}}) |
																		({GPR[MiscInstReg[11:8]][127:64], ShiftR} & {128{MiscBypassBReg[4]}}) |
																		(MiscR & {128{MiscBypassBReg[5]}}));

	MiscSAReg<=(MiscInstReg[3] & (~MiscInstReg[2] | ~MiscInstReg[1])) ? ((AFR[MiscInstReg[7:4]][24:22] & {3{~(|MiscBypassAReg)}})|
																		(FADDSR & {3{MiscBypassAReg[0]}})|
																		({2'd1, FMULSR} & {3{MiscBypassAReg[1]}})|
																		({MiscBypassAReg[2], 2'd0}) |
																		({1'b0, ALUSRReg & {2{MiscBypassAReg[3]}}})|
																		({1'b0, ShiftSRReg & {2{MiscBypassAReg[4]}}})|
																		(MiscSRReg & {3{MiscBypassAReg[5]}})) : 
																		((AFR[MiscInstReg[11:8]][24:22] & {3{~(|MiscBypassBReg)}})|
																		(FADDSR & {3{MiscBypassBReg[0]}})|
																		({2'd1, FMULSR} & {3{MiscBypassBReg[1]}})|
																		({MiscBypassBReg[2], 2'd0}) |
																		({1'b0, ALUSRReg & {2{MiscBypassBReg[3]}}})|
																		({1'b0, ShiftSRReg & {2{MiscBypassBReg[4]}}})|
																		(MiscSRReg & {3{MiscBypassBReg[5]}}));
	
	MiscSDReg<=(AFR[MiscInstReg[11:8]][24:22] & {3{~(|MiscBypassBReg)}})|
				(FADDSR & {3{MiscBypassBReg[0]}})|
				({2'd1, FMULSR} & {3{MiscBypassBReg[1]}})|
				({MiscBypassBReg[2], 2'd0}) |
				({1'b0, ALUSRReg & {2{MiscBypassBReg[3]}}})|
				({1'b0, ShiftSRReg & {2{MiscBypassBReg[4]}}})|
				(MiscSRReg & {3{MiscBypassBReg[5]}});

	MiscDSTReg<=MiscInstReg[11:8];
	MiscOPRReg<=MiscInstReg[3:1];
	
	// loop instruction channel
	LoopInstReg<=LoopInstBus;
	
	LoopBypassReg[0]<=FADDRDYR & (FADDDSTR==LoopInstBus[15:12]);
	LoopBypassReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==LoopInstBus[15:12]);
	LoopBypassReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==LoopInstBus[15:12]);
	LoopBypassReg[3]<=ALURDYR & (ALUDSTR==LoopInstBus[15:12]);
	LoopBypassReg[4]<=ShiftRDYR & (ShiftDSTR==LoopInstBus[15:12]);
	LoopBypassReg[5]<=MiscRDYR & (MiscDSTR==LoopInstBus[15:12]);
	
	// prefetcher channel
	PrefInstReg<=PrefInstBus;

	PrefBypassReg[0]<=FADDRDYR & (FADDDSTR==PrefInstBus[16:13]);
	PrefBypassReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==PrefInstBus[16:13]);
	PrefBypassReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==PrefInstBus[16:13]);
	PrefBypassReg[3]<=ALURDYR & (ALUDSTR==PrefInstBus[16:13]);
	PrefBypassReg[4]<=ShiftRDYR & (ShiftDSTR==PrefInstBus[16:13]);
	PrefBypassReg[5]<=MiscRDYR & (MiscDSTR==PrefInstBus[16:13]);
	
	PrefCCBypassReg[0]<=FADDRDYR & (FADDDSTR==PrefInstBus[8:5]);
	PrefCCBypassReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==PrefInstBus[8:5]);
	PrefCCBypassReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==PrefInstBus[8:5]);
	PrefCCBypassReg[3]<=ALURDYR & (ALUDSTR==PrefInstBus[8:5]);
	PrefCCBypassReg[4]<=ShiftRDYR & (ShiftDSTR==PrefInstBus[8:5]);
	PrefCCBypassReg[5]<=MiscRDYR & (MiscDSTR==PrefInstBus[8:5]);
	
	// register movement operation
	MovInstReg<=MovInstBus;

	CopyBypassReg[0]<=FADDRDYR & (FADDDSTR==MovInstBus[10:7]);
	CopyBypassReg[1]<=FMULRDYR[0] & (FMULDSTR[0]==MovInstBus[10:7]);
	CopyBypassReg[2]<=FMULRDYR[1] & (FMULDSTR[1]==MovInstBus[10:7]);
	CopyBypassReg[3]<=ALURDYR & (ALUDSTR==MovInstBus[10:7]);
	CopyBypassReg[4]<=ShiftRDYR & (ShiftDSTR==MovInstBus[10:7]);
	CopyBypassReg[5]<=MiscRDYR & (MiscDSTR==MovInstBus[10:7]);

	if (&MovInstReg[2:0]) 	begin
							// load immediate data mode
							case (AFR[MovInstReg[14:11]][31:28])
								4'h0: MovReg<={{120{MovInstReg[10]}},MovInstReg[10:3]};
								4'h1: MovReg<={{112{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][7:0]};
								4'h2: MovReg<={{104{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][15:0]};
								4'h3: MovReg<={{96{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][23:0]};
								4'h4: MovReg<={{88{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][31:0]};
								4'h5: MovReg<={{80{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][39:0]};
								4'h6: MovReg<={{72{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][47:0]};
								4'h7: MovReg<={{64{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][55:0]};
								4'h8: MovReg<={{56{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][63:0]};
								4'h9: MovReg<={{48{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][71:0]};
								4'hA: MovReg<={{40{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][79:0]};
								4'hB: MovReg<={{32{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][87:0]};
								4'hC: MovReg<={{24{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][95:0]};
								4'hD: MovReg<={{16{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][103:0]};
								4'hE: MovReg<={{8{MovInstReg[10]}},MovInstReg[10:3],GPR[MovInstReg[14:11]][111:0]};
								4'hF: MovReg<={MovInstReg[10:3],GPR[MovInstReg[14:11]][119:0]};
								endcase
							// size
							MovSR[0]<=(~AFR[MovInstReg[14:11]][31] & AFR[MovInstReg[14:11]][30])|
										(~AFR[MovInstReg[14:11]][31] & ~AFR[MovInstReg[14:11]][30] & ~AFR[MovInstReg[14:11]][29] & AFR[MovInstReg[14:11]][28]);
							MovSR[1]<=(AFR[MovInstReg[14:11]][30] | AFR[MovInstReg[14:11]][29]) & ~AFR[MovInstReg[14:11]][31];
							MovSR[2]<=AFR[MovInstReg[14:11]][31];
							end
			else begin
				if (MovInstReg[2:0]==3'b001)	begin
												// COPY operations
												MovReg<=(GPR[MovInstReg[10:7]] & {128{~(|CopyBypassReg)}}) | 
														(FADDR & {128{CopyBypassReg[0]}}) |
														({64'd0, FMULR & {64{CopyBypassReg[1]}}}) |
														(FMULRQ & {128{CopyBypassReg[2]}}) |
														({GPR[MovInstReg[10:7]][127:64], ALUR} & {128{CopyBypassReg[3]}}) |
														({GPR[MovInstReg[10:7]][127:64], ShiftR} & {128{CopyBypassReg[4]}}) |
														(MiscR & {128{CopyBypassReg[5]}});
												MovSR<=(AFR[MovInstReg[10:7]][24:22] & {3{~(|CopyBypassReg)}})|
														(FADDSR & {3{CopyBypassReg[0]}})|
														({2'd1, FMULSR} & {3{CopyBypassReg[1]}})|
														({CopyBypassReg[2], 2'd0}) |
														({1'b0, ALUSRReg & {2{CopyBypassReg[3]}}})|
														({1'b0, ShiftSRReg & {2{CopyBypassReg[4]}}})|
														(MiscSRReg & {3{CopyBypassReg[5]}});
												end
										else if (MovInstReg[2:0]==3'b011) 
												begin
												if (MovInstReg[7]) AFR[MovInstReg[14:11]][27:25]<=MovInstReg[10:8];  // AMODE
															else	AFR[MovInstReg[14:11]][24:22]<=MovInstReg[10:8]; // SIZE
												end
				end

	//
	// Memory operations and address register 
	//
	if (~MemDelayFlag & ~MemFifoFullFlag) MemInstReg<=MemInstBus;

	MemDelayFlag<=~MemDelayFlag & MemInstBus[0] & ((FADDRDYR & (FADDDSTR==MemInstBus[15:12])) | (FMULRDYR[0] & (FMULDSTR[0]==MemInstBus[15:12])) |
					(FMULRDYR[1] & (FMULDSTR[1]==MemInstBus[15:12])) | (ALURDYR & (ALUDSTR==MemInstBus[15:12])) | (ShiftRDYR & (ShiftDSTR==MemInstBus[15:12])) |
					(MiscRDYR & (MiscDSTR==MemInstBus[15:12])) | (FADDRDYR & (FADDDSTR==MemInstBus[11:8])) | (FMULRDYR[0] & (FMULDSTR[0]==MemInstBus[11:8])) |
					(FMULRDYR[1] & (FMULDSTR[1]==MemInstBus[11:8])) | (ALURDYR & (ALUDSTR==MemInstBus[11:8])) | (ShiftRDYR & (ShiftDSTR==MemInstBus[11:8])) |
					(MiscRDYR & (MiscDSTR==MemInstBus[11:8])));
	
	
	
	// busy flag of memory command
	MemBusy<=(MemBusy & ~MemLoadOffset & (~MemLoadSel | (MemDST=={CheckSEL,CheckACT})) & ~MemReadAR & ~(MemNext & ~FMULACCMemACT)) | MemCNT[1] | MemCNT[0] | ~MemFifoEmpty;
	
	// Memory interface unit operation flags
	if ((~MemBusy | (MemNext & ~FMULACCMemACT) | MemLoadOffset | (MemLoadSel & (MemDST!={CheckSEL,CheckACT})) | MemReadAR) & ~MemCNT[1] & ~MemCNT[0])
		begin
		// Read AR
		MemReadAR<=~MemFifoEmpty & ~MemFifoBus[3] & MemFifoBus[2] & ~MemFifoBus[1] & MemFifoBus[0] & ~MemFifoBus[215] & ~MemFifoBus[216];
		// Load AR (offset)
		MemLoadOffset<=~MemFifoEmpty & ~MemFifoBus[3] & MemFifoBus[2] & MemFifoBus[1] & ~MemFifoBus[0] & ~MemFifoBus[11] & ~MemFifoBus[215] & ~MemFifoBus[216];
		// Load AR (selector)
		MemLoadSel<=~MemFifoEmpty & ~MemFifoBus[3] & MemFifoBus[2] & MemFifoBus[1] & ~MemFifoBus[0] & MemFifoBus[11] & ~MemFifoBus[215] & ~MemFifoBus[216];
		// Request flag to the ATU
		MemReq<=~MemFifoEmpty & (MemFifoBus[3] | (~MemFifoBus[2] & ~MemFifoBus[1] & ~MemFifoBus[0]) |
												(MemFifoBus[2] & MemFifoBus[1] & MemFifoBus[0])) & ~MemFifoBus[215] & ~MemFifoBus[216];
		// Read/Write flag
		MemOpr<=MemFifoBus[3];
		// PUsh/POP/LDST flags
		MemPUSH<=~MemFifoEmpty & ~MemFifoBus[3] & MemFifoBus[2] & MemFifoBus[1] & MemFifoBus[0] & ~MemFifoBus[215] & ~MemFifoBus[216];
		MemPOP<=~MemFifoEmpty & MemFifoBus[3] & MemFifoBus[2] & MemFifoBus[1] & MemFifoBus[0] & ~MemFifoBus[215] & ~MemFifoBus[216];
		MemLDST<=~MemFifoEmpty & ((MemFifoBus[3] & (~MemFifoBus[2] | ~MemFifoBus[1] | ~MemFifoBus[0])) |
									(~MemFifoBus[3] & ~MemFifoBus[2] & ~MemFifoBus[1] & ~MemFifoBus[0])) & ~MemFifoBus[215] & ~MemFifoBus[216];
		MemLDO<=~MemFifoEmpty & MemFifoBus[3] & MemFifoBus[2] & ~MemFifoBus[1] & ~MemFifoBus[0] & ~MemFifoBus[215] & ~MemFifoBus[216];
		MemADRPushPop<=~MemFifoEmpty & MemFifoBus[2] & MemFifoBus[1] & MemFifoBus[0] & MemFifoBus[10] & ~MemFifoBus[215] & ~MemFifoBus[216];
		
		PrefCallReg<=~MemFifoEmpty & MemFifoBus[215];
		PrefRetReg<=~MemFifoEmpty & MemFifoBus[216];
		// Size
		if (MemFifoBus[3:0]==4'b0000) MemSize<=MemFifoBus[207:205];
								else MemSize<=MemFifoBus[2:0];
		// SEL field register
		if (MemFifoBus[3:0]==4'b0110) MemSEL<=MemFifoBus[14:12];
								else MemSEL<=MemFifoBus[6:4];
		// DST field register
		MemDST<=MemFifoBus[14:11];
		// Offset from GPR
		MemGPROffset<=MemFifoBus[51:15];
		// Offset register
		MemOFF<=MemFifoBus[10:7];
		// address mode
		MemAMode<=MemFifoBus[54:52];
		// GPR flags
		MemAFR<=MemFifoBus[214:183];
		// GPR
		MemGPR<=MemFifoBus[182:55];
		// data to GPR
		ARData<=ADR[MemOFF];
		end
	// Count of the bus cycles
	if ((~MemBusy | (MemNext & ~FMULACCMemACT) | MemLoadOffset | (MemLoadSel & (MemDST!={CheckSEL,CheckACT})) | MemReadAR) & ~MemFifoEmpty & (MemCNT==2'd0) & ~MemFifoBus[215] & ~MemFifoBus[216])
					begin
					MemCNT[0]<=(MemFifoBus[3:0]==4'b1100) |
							((MemFifoBus[3:0]==4'b0000) & MemFifoBus[207]);
					MemCNT[1]<=~MemFifoBus[10] & (MemFifoBus[2:0]==3'b111);
					end
				else if (MemNext & (MemCNT!=2'b00) & ~FMULACCMemACT) MemCNT<=MemCNT-2'd1;

	// resetting the descriptor valid flags 
	for (i=0; i<8; i=i+1)
		if (MemLoadSel & (MemSEL==i) & (MemDST!={CheckSEL,CheckACT & ~CheckAR[3]})) DTR[i][155]<=DTR[i][155] & (MemGPROffset[31:0]==ADR[{MemSEL,1'b1}][31:0]);
			else if (MemLSelNode & (TAGiReg[3:1]==i)) DTR[i][155]<=DTR[i][155] & (DTi[31:0]==ADR[TAGiReg[3:0]][31:0]);
	
	// second cycle for 128-bit transfers
	if ((MemNext & ~FMULACCMemACT) | ~MemBusy) MemSecondCycle<=(MemCNT==2'b01);
	//
	// processing address registers
	//
	for (i=0; i<16; i=i+1)
		if (i[0])
				begin
				// for selector registers
				if ((i!=15) & (i!=13))
						begin
						if (MemASelNode[i])	ADR[i]<=DTi[36:0];
							else if (MemLoadSel & (MemDST==i) & ((MemDST!={CheckSEL,CheckACT & ~CheckAR[3]}) | MemNext)) ADR[i]<=MemGPROffset;
						end
					else begin
						// stack selector and code selector can't be loaded from GPR's
						if (MemASelNode[i]) ADR[i]<=DTi[36:0];
							else if (MemLoadSel & (MemDST==i) & ~CPL[1] & ~CPL[0]) ADR[i]<=MemGPROffset;
						end
				end
			else begin
				// for offset registers
				if (i==14)
						begin
						// stack offset can be loaded from memory or modified in PUSH/POP/CALL/RET operations
						if (MemASelNode[i]) ADR[i]<=DTi[36:0];
							else if (MemLoadOffset & (MemDST==i)) ADR[i]<=MemGPROffset;
								else if ((MemPUSH | PrefCallReg) & MemNext & ~FMULACCMemACT) ADR[i]<=ADR[i]-8;
									else if ((MemPOP | PrefRetReg) & MemNext & ~FMULACCMemACT) ADR[i]<=ADR[i]+8;
						end
					else begin
						// all other offset registers
						if (MemASelNode[i]) ADR[i]<=DTi[36:0];
							else if (MemLoadOffset & (MemDST==i)) ADR[i]<=MemGPROffset;
									else if (MemLDST & MemNext & ~FMULACCMemACT & (({1'b0,MemSEL}<<1)==i) & MemReq & ~MemCNT[1] & ~MemCNT[0])
										begin
										if (MemAMode==3'b001) ADR[i]<=ADR[i]+MemGPROffset;
											else if (MemAMode[2] & ~MemAMode[0]) ADR[i]<=ADR[i]+MemoryOSValue;
												else if (MemAMode[2] & MemAMode[0]) ADR[i]<=ADR[i]-MemoryOSValue;
										end
						end
				end
	//
	// Conversion logical address to physical with access verification
	//
	CheckACT<=(CheckACT  & ~MemNext) | MemReq | PrefACT | PrefCallReg | PrefRetReg | FMULACCMemACT;
	
	// check offset and access type stage
	if (MemNext)
		begin
		// prefetcher flag
		CheckPref<=PrefACT & ~MemReq & ~PrefCallReg & ~PrefRetReg & ~FMULACCMemACT;
		// selector index
		if (FMULACCMemACT) CheckSEL<=FMULACCMemSEL;
			else if (PrefCallReg | PrefRetReg) CheckSEL<=3'd7;
					else if (MemReq) CheckSEL<=MemSEL | {3{MemPUSH | MemPOP}};
							else CheckSEL<=3'd6;
					
		// Command
		if (PrefCallReg) CheckCMD<=FMULACCMemACT;
			else if (MemReq) CheckCMD<=MemOpr | FMULACCMemACT | MemPOP;
					else CheckCMD<=1'b1;
			
		// operand size
		if (FMULACCMemACT) CheckOS<={1'b1,FMULACCSIZE};
			else if (MemReq) CheckOS<=MemSize[1:0] | {2{MemSize[2]}};
					else CheckOS<=2'b11;
		// Forming OFFSET 
		if (FMULACCMemACT) CheckOffset<=ADR[{1'b0,FMULACCMemSEL}<<1]+{FMULACCMemOffset,2'd0};
			else if (~MemReq & ~PrefCallReg & ~PrefRetReg) CheckOffset<=PrefOffset;
				else if (MemPUSH | PrefCallReg) CheckOffset<=ADR[14]-36'd8;
					else if (MemPOP | PrefRetReg) CheckOffset<=ADR[14];
							else case (MemAMode)
								3'b000 : CheckOffset<=ADR[{1'b0,MemSEL}<<1]+{33'd0,MemSecondCycle,3'd0};
								3'b001 : CheckOffset<=ADR[{1'b0,MemSEL}<<1]+{33'd0,MemSecondCycle,3'd0};
								3'b010 : CheckOffset<=MemGPROffset+{33'd0,MemSecondCycle,3'd0};
								3'b011 : CheckOffset<=ADR[{1'b0,MemSEL}<<1]+MemGPROffset+{33'd0,MemSecondCycle,3'd0};
								3'b100 : CheckOffset<=ADR[{1'b0,MemSEL}<<1]+{33'd0,MemSecondCycle,3'd0};
								3'b101 : CheckOffset<=ADR[{1'b0,MemSEL}<<1]+{33'd0,MemSecondCycle,3'd0};
								3'b110 : CheckOffset<=ADR[{1'b0,MemSEL}<<1]+MemGPROffset+{33'd0,MemSecondCycle,3'd0};
								3'b111 : CheckOffset<=ADR[{1'b0,MemSEL}<<1]+MemGPROffset+{33'd0,MemSecondCycle,3'd0};
								endcase
		// Base address limits and access rights
		if (FMULACCMemACT)
				begin
				CheckBase<=DTR[FMULACCMemSEL][39:0];
				CheckLL<=DTR[FMULACCMemSEL][71:40];
				CheckUL<=DTR[FMULACCMemSEL][103:72];
				CheckLowerSel<=DTR[FMULACCMemSEL][127:104];
				CheckUpperSel<=DTR[FMULACCMemSEL][151:128];
				CheckAR<=DTR[FMULACCMemSEL][155:152];
				CheckNetwork<=(CPU!=ADR[1+({1'b0,FMULACCMemSEL}<<1)][31:24]) & (|ADR[1+({1'b0,FMULACCMemSEL}<<1)][31:24]);
				CheckSelector<=ADR[1+({1'b0,FMULACCMemSEL}<<1)][31:0];
				end
			else begin
				if (MemPOP | MemPUSH | PrefCallReg | PrefRetReg)			//NEW
						begin
						// stack access
						CheckBase<=DTR[7][39:0];
						CheckLL<=DTR[7][71:40];
						CheckUL<=DTR[7][103:72];
						CheckLowerSel<=DTR[7][127:104];
						CheckUpperSel<=DTR[7][151:128];
						CheckAR<=DTR[7][155:152];
						CheckNetwork<=(CPU!=ADR[15][31:24]) & (|ADR[15][31:24]);
						CheckSelector<=ADR[15][31:0];
						end
					else begin
						if (MemReq)
							begin
							// data transactions
							CheckBase<=DTR[MemSEL][39:0];
							CheckLL<=DTR[MemSEL][71:40];
							CheckUL<=DTR[MemSEL][103:72];
							CheckLowerSel<=DTR[MemSEL][127:104];
							CheckUpperSel<=DTR[MemSEL][151:128];
							CheckAR<=DTR[MemSEL][155:152];
							CheckNetwork<=(CPU!=ADR[1+({1'b0,MemSEL}<<1)][31:24]) & (|ADR[1+({1'b0,MemSEL}<<1)][31:24]);
							CheckSelector<=ADR[1+({1'b0,MemSEL}<<1)][31:0];
							end
							else begin
							// code fetch
							CheckBase<=DTR[6][39:0];
							CheckLL<=DTR[6][71:40];
							CheckUL<=DTR[6][103:72];
							CheckLowerSel<=DTR[6][127:104];
							CheckUpperSel<=DTR[6][151:128];
							CheckAR<=DTR[6][155:152];
							CheckNetwork<=(CPU!=ADR[13][31:24]) & (|ADR[13][31:24]);
							CheckSelector<=ADR[13][31:0];
							end
					end
				end
		// forming data to memory
		if (PrefCallReg) CheckData<=MemGPR[63:0];
			else if (MemPUSH & (MemCNT==2'b10)) CheckData<=MemAFR;
					else if (MemPUSH & (MemCNT==2'b01)) CheckData<=MemGPR[127:64];
							else if (MemPUSH & MemSecondCycle) CheckData<=MemGPR[63:0];
									else if (MemPUSH & (MemCNT==2'b00)) CheckData<=ADR[MemDST];
											else if (MemSecondCycle) CheckData<=MemGPR[127:64];
													else CheckData<=MemGPR[63:0];
		// tag
		if (FMULACCMemACT) CheckTag<={7'b1100000, FMULACCTAGo};
			else if ((PrefCallReg | PrefRetReg) | (~MemReq & ~PrefCallReg & ~PrefRetReg))
						begin
						CheckTag[6:4]<=3'b110;
						CheckTag[3]<=PrefCallReg | PrefRetReg;
						CheckTag[2:0]<=PrefTag;
						CheckTag[7]<=1'b0;
						end
					else begin
						CheckTag[3:0]<=MemDST;
						CheckTag[4]<=(MemSecondCycle & MemLDST) | (~MemLDST & (MemCNT==2'b01)) | MemADRPushPop;
						CheckTag[5]<=(MemPUSH & (MemCNT==2'b10))|(MemPOP & (MemCNT==2'b00))| MemADRPushPop;
						CheckTag[6]<=1'b0;
						CheckTag[7]<=MemLDO;
						end
		end
	// reloading descriptor when he loaded from table
	if (DTRLoadedFlag)
		begin
		CheckBase<=DTR[CheckSEL][39:0];
		CheckLL<=DTR[CheckSEL][71:40];
		CheckUL<=DTR[CheckSEL][103:72];
		CheckLowerSel<=DTR[CheckSEL][127:104];
		CheckUpperSel<=DTR[CheckSEL][151:128];
		CheckAR<=DTR[CheckSEL][155:152];
		end

	//
	// output stage
	//
	// local memory access activation
	ACT<=(ACT & ~NEXT) | (CheckNext & CheckACT & CheckAR[3] & ~CheckNetwork & ~CheckAR[2] & (CheckOffset[36:5]>=CheckLL) & (CheckOffset[36:5]<CheckUL) & (CheckCMD | CheckAR[1]) & 
									(~CheckCMD | CheckAR[0])) | (DLMachine==LBS) | (DLMachine==LLSS) | (DLMachine==LSLS) |
									((DLMachine==STS) & CheckTag[6] & CheckTag[5] & ~CheckTag[4] & ~CheckTag[3]);
	// stream access activation
	StreamACT<=(StreamACT & ~StreamNEXT) | (CheckACT & CheckAR[3] & ~CheckNetwork & CheckAR[2] & CheckNext);
	
	// network access activation
	NetACT<=(NetACT & ~NetNEXT) | (CheckNext & CheckACT & CheckNetwork);
	
	if ((~ACT | NEXT) & (~StreamACT | StreamNEXT) & (~NetACT | NetNEXT))
		begin
		// command
		CMD<=CheckCMD | DescriptorLoadState;
		// operand size
		OS<=CheckOS | {2{DescriptorLoadState}};
		// forming physical address
		ADDRESS[2:0]<=DescriptorLoadState ? 3'b000 : CheckOffset[2:0];
		ADDRESS[3]<=DescriptorLoadState ? (DLMachine==LLSS) : CheckOffset[3];
		ADDRESS[4]<=DescriptorLoadState ? (DLMachine==LSLS) : CheckOffset[4];
		ADDRESS[44:5]<=DescriptorLoadState ? DTBASE + {16'd0, DLSelector} : (CheckNetwork ? {8'd0,CheckOffset[36:5]} : CheckBase+{8'd0,(CheckOffset[36:5]-CheckLL)});
		// offset
		OFFSET<=CheckOffset[36:0];
		// selector (for network accesses)
		SELECTOR<=CheckSelector;
		// data to memory
		DTo<=CheckData;
		// tag
		TAGo[0]<=DescriptorLoadState ? (DLMachine==LLSS) : CheckTag[0];
		TAGo[1]<=DescriptorLoadState ? (DLMachine==LSLS) : CheckTag[1];
		TAGo[4:2]<=DescriptorLoadState ? CheckSEL : CheckTag[4:2] | {((DLMachine==STS) & CheckTag[6] & CheckTag[5] & ~CheckTag[4] & ~CheckTag[3]), 2'b00};
		TAGo[5]<=DescriptorLoadState ? 1'b0 : CheckTag[5];
		TAGo[6]<=DescriptorLoadState ? 1'b1 : CheckTag[6];
		TAGo[7]<=DescriptorLoadState ? 1'b0 : CheckTag[7];
		end

	//
	// Loading DT registers
	//
	DescriptorLoadState<=(DLMachine==CSS)|(DLMachine==LBS)|(DLMachine==LLSS)|(DLMachine==LSLS);
	// state machine for descriptor loading
		case (DLMachine)
				// wait state
				WS:	if (LoadNewDSC | LoadLowerDSC | LoadUpperDSC) DLMachine<=CSS;
						else if (AccessError) DLMachine<=AERSS;
								else DLMachine<=WS;
				// checking selector value
				CSS: if (DLSelector==24'd0) DLMachine<=ZSS;
						else if (DLSelector>=DTLIMIT) DLMachine<=INVSS;
							else DLMachine<=LBS;
				// load base address and access rights
				LBS: if (~ACT | NEXT) DLMachine<=LLSS;
						else DLMachine<=LBS;
				// load link selectors and check descriptor type
				LLSS: if (~ACT | NEXT) DLMachine<=LSLS;
						else DLMachine<=LLSS;
				// load segment limits
				LSLS: if (~ACT | NEXT) DLMachine<=RWS;
						else DLMachine<=LSLS;
				// waiting for transaction retry condition
				RWS: if (~RetryTransactionFlag) DLMachine<=RWS;
						else if (~ValidDescriptor) DLMachine<=INVOBS;
								else DLMachine<=WS;
				// zero selector reporting
				ZSS: DLMachine<=STS;
				// invalid selector reporting
				INVSS: DLMachine<=STS;
				// invalid object reporting (CPL or type)
				INVOBS: DLMachine<=STS;
				// access error
				AERSS: DLMachine<=STS;
				// skip transaction after error reporting
				STS: if (CheckTag[6] & CheckTag[5] & ~CheckTag[4] & ~CheckTag[3] & ~CheckNext) DLMachine<=STS;
						else DLMachine<=WS;
				endcase
	// temporary selector register
	if (DLMachine==WS) for (i=0; i<24; i=i+1) DLSelector[i]<=(LoadNewDSC & CheckSelector[i])|
															(LoadLowerDSC & CheckLowerSel[i])|
															(LoadUpperDSC & CheckUpperSel[i]);
				
	// 0 - base and AR, 1 - link selectors, 2 - limits
	for (i=0; i<8; i=i+1)
				if (DTRWriteFlag[i])
					case (TAGiReg[1:0])
						2'b00 : begin
								DTR[i][39:0]<=DTi[39:0];
								DTR[i][153:152]<=DTi[61:60];
								DTR[i][155:154]<=DTi[57:56];
								end
						2'b01 : begin
								DTR[i][127:104]<=DTi[23:0];
								DTR[i][151:128]<=DTi[55:32];
								end
						2'b10 :	begin
								DTR[i][103:40]<=DTi;
								end
						2'b11 :	DTR[i]<=156'hB000000000000FFFFFFFF000000000000000000;
						endcase
	DTRLoadedFlag<=DTRWriteFlag[8] & (TAGiReg[1:0]==2'b10);
	// flag to enable check transaction again
	RetryTransactionFlag<=DTRLoadedFlag;

	// valid descriptor flag
	if (DTRWriteFlag[8] & (TAGiReg[1:0]==2'b00))
		begin
		ValidDescriptor<=DTi[57] & (CPL<=DTi[59:58]) & ((TASKID==DTi[55:40]) | (~(|TASKID)) | (~(|DTi[55:40])));
		InvalidType<=~DTi[57];
		InvalidCPL<=(CPL>DTi[59:58]);
		InvalidTaskID<=(TASKID!=DTi[55:40]) & (|TASKID) & (|DTi[55:40]);
		InvalidSelector<=CheckSelector[23:0];
		end
	
	// skip read operation (validate flags)
	SkipDataRead<=CheckACT & CheckCMD & (DLMachine==STS) & ~CheckTag[6] & ~CheckTag[5];
	if (DLMachine==STS) STAGReg<=CheckTag[3:0];

	end

endmodule
