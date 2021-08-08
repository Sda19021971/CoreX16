module Neuro32
	(
	// control-data interface
	input wire CLK, RESET, ACT, MAERR,
	output wire NEXT,
	input wire [3:0] DSTi,
	input wire [2:0] DataSEL, CtrlSEL,
	input wire [34:0] DataOffset,
	input wire [33:0] CtrlOffset,
	output wire RDY, SIGN, ZERO, INF, NAN,
	output wire [31:0] R,
	output wire [3:0] DSTo,
	// memory interface
	input wire MemNEXT,
	output wire MemACT,
	output wire [2:0] MemSEL,
	output wire [34:0] MemOffset,
	output wire SIZE,
	output wire TAGo,
	input wire DRDY, TAGi,
	input wire [63:0] DTi
	);
	
//-----------------------------------------------
//	resources

reg ACTReg, TagReg, SizeReg, RDYFlag, StartFlag, FifoReadFlag, NextFlag, FifoFullFlag, MAERRFlag, SequModeFlag;
// address registers
reg [33:0] CtrlOffsetReg;
reg [34:0] DataOffsetReg, DataAddrReg, AddrReg;
reg [2:0] CtrlSelReg, DataSelReg, MemSELReg;
reg [3:0] DSTReg;
reg [2:0] CtrlMachine;
reg [31:0] CntReg, DeltaReg, DispReg;
reg [3:0] TCount;
parameter CWS=3'd0, CCRS=3'd1, CCWS=3'd2, CDRS=3'd3, CDWS=3'd4, CVRS=3'd5, CFWS=3'd6, CBRS=3'd7;
// data and scaler fifo's
logic [31:0] ScaleFifoBus, DataFifoBus, MulBus, OffsetFifoBus;
logic [3:0] ScaleFifoWordsBus, DataFifoWordsBus;
logic OffsetFifoEmpty, ScaleFifoEmpty, DataFifoEmpty, CtrlNext, CounterEmpty, SequMode, MulRDY, AccRDY, MulTag, AccTag;

//-----------------------------------------------
// 		Modules
sc_fifo OffsetFIFO (.data(DTi[63:32]), .wrreq(DRDY & ~TAGi & ~(CtrlMachine==CCWS) & ~SequModeFlag), .rdreq(~OffsetFifoEmpty & CtrlNext), .clock(CLK), .sclr(~RESET),
					.q(OffsetFifoBus), .empty(OffsetFifoEmpty));
defparam OffsetFIFO.LPM_WIDTH=32, ScaleFIFO.LPM_NUMWORDS=16, ScaleFIFO.LPM_WIDTHU=4;


sc_fifo ScaleFIFO (.data(DTi[31:0]), .wrreq(DRDY & ~TAGi & ~(CtrlMachine==CCWS)), .rdreq(~ScaleFifoEmpty & ~DataFifoEmpty & ~FifoReadFlag), .clock(CLK), .sclr(~RESET),
					.q(ScaleFifoBus), .empty(ScaleFifoEmpty), .usedw(ScaleFifoWordsBus));
defparam ScaleFIFO.LPM_WIDTH=32, ScaleFIFO.LPM_NUMWORDS=16, ScaleFIFO.LPM_WIDTHU=4;

sc_fifo DataFIFO (.data(DTi[31:0]), .wrreq(DRDY & TAGi), .rdreq(~ScaleFifoEmpty & ~DataFifoEmpty & ~FifoReadFlag), .clock(CLK), .sclr(~RESET),
					.q(DataFifoBus), .empty(DataFifoEmpty), .usedw(DataFifoWordsBus));
defparam DataFIFO.LPM_WIDTH=32, DataFIFO.LPM_NUMWORDS=16, DataFIFO.LPM_WIDTHU=4;

FPMUL32 Multiplier (.CLK(CLK), .RESET(RESET), .ACT(~ScaleFifoEmpty & ~DataFifoEmpty & ~FifoReadFlag), .TAGi((TCount==4'd1) & (CtrlMachine==CFWS)),
					.A(ScaleFifoBus), .B(DataFifoBus), .RDY(MulRDY), .TAGo(MulTag), .R(MulBus));

FPACC32 Accumulator (.CLK(CLK), .CLEAR(StartFlag), .ACT(MulRDY), .TAGi(MulTag), .A(MulBus), .SIGN(SIGN), .ZERO(ZERO), .INF(INF), .NAN(NAN),
					.RDY(AccRDY), .TAGo(AccTag), .R(R));

//-----------------------------------------------
//	assignments
assign DSTo=DSTReg, MemOffset=AddrReg, MemACT=ACTReg, MemSEL=MemSELReg, TAGo=TagReg, SIZE=SizeReg, RDY=RDYFlag, NEXT=NextFlag;

//-----------------------------------------------
//	asynchronous logic
always_comb
begin

// next signal for control machine
CtrlNext=~ACTReg | MemNEXT;
// sequential datafetch mode flag
SequMode=|DeltaReg;
// counter end
CounterEmpty=~(|CntReg);

end

//-----------------------------------------------
//	synchronous logic
always_ff @(posedge CLK or negedge RESET)
if (!RESET) begin
			CtrlMachine<=CWS;
			ACTReg<=1'b0;
			CntReg<=32'd0;
			DeltaReg<=32'd0;
			DispReg<=0;
			MemSELReg<=3'd0;
			TagReg<=0;
			SizeReg<=0;
			TCount<=0;
			FifoReadFlag<=0;
			NextFlag<=1;
			MAERRFlag<=0;
			StartFlag<=0;
			FifoFullFlag<=0;
			SequModeFlag<=0;
			end
else begin

// selector registers
if (ACT & NEXT)
	begin
	CtrlSelReg<=CtrlSEL;
	DataSelReg<=DataSEL;
	DataOffsetReg<=DataOffset;
	DSTReg<=DSTi;
	end
// control address register
if (ACT & NEXT) CtrlOffsetReg<=CtrlOffset;
	else if ((CtrlMachine==CCRS) | ((CtrlMachine==CDRS) & CtrlNext & ~FifoFullFlag & OffsetFifoEmpty)) CtrlOffsetReg<=CtrlOffsetReg+34'd1;
	
// data counter register
if (CtrlMachine==CCWS) CntReg<=DTi[31:0];
	else if ((CtrlMachine==CDRS) & CtrlNext & OffsetFifoEmpty) CntReg<=CntReg-32'd1;

// delta register
if (CtrlMachine==CCWS) DeltaReg<=DTi[63:32];

// registered sequential mode flag
SequModeFlag<=SequMode;

// Data diplacement register
if ((CtrlMachine==CCRS)|((CtrlMachine==CVRS) & CtrlNext))
	DispReg<=(CtrlMachine==CCRS) ? DataOffsetReg[31:0] : (DispReg+32'd1) & {32{(DispReg<DeltaReg)}};

// control machine
case (CtrlMachine)
	CWS: if (ACT & NEXT) CtrlMachine<=CCRS;
				else CtrlMachine<=CWS;
	// read control word
	CCRS: CtrlMachine<=CCWS;
	// waiting for control word
	CCWS: if (MAERR) CtrlMachine<=CBRS;
			else if (DRDY & ~TAGi) CtrlMachine<=CDRS;
					else CtrlMachine<=CCWS;
	// reading scale value and data offset
	CDRS: if (MAERRFlag) CtrlMachine<=CFWS;
			else if (~CtrlNext | FifoFullFlag | ~OffsetFifoEmpty) CtrlMachine<=CDRS;
					else if (SequMode) CtrlMachine<=CVRS;
							else CtrlMachine<=CDWS;
	// checking end of computing in non-regular address mode
	CDWS: if (CounterEmpty) CtrlMachine<=CFWS;
			else CtrlMachine<=CDRS;
	// reading data value in sequential address mode
	CVRS: if (~CtrlNext) CtrlMachine<=CVRS;
			else if (CounterEmpty) CtrlMachine<=CFWS;
					else CtrlMachine<=CDRS;
	// waiting for finish of the operation 
	CFWS: if (RDY) CtrlMachine<=CWS;
			else CtrlMachine<=CFWS;
	// break operation if read control word failed
	CBRS: CtrlMachine<=CWS;
	endcase

// transaction counter
TCount<=(TCount+{3'd0,((CtrlMachine==CDRS) & ~FifoFullFlag & ~MAERRFlag) & CtrlNext & OffsetFifoEmpty}-
		{3'd0,~ScaleFifoEmpty & ~DataFifoEmpty & ~FifoReadFlag}) & {4{(CtrlMachine!=CWS)}};

// fifo full flag
FifoFullFlag<=(TCount>4'hC);

// error flag
MAERRFlag<=(MAERRFlag | MAERR) & (CtrlMachine!=CWS);

// ready flag
RDYFlag<=(AccRDY & AccTag) | (CtrlMachine==CBRS);

// start flag
StartFlag<=(CtrlMachine==CCRS);

// fifo read flag
FifoReadFlag<=~ScaleFifoEmpty & ~DataFifoEmpty & ~FifoReadFlag;

// Next operation
NextFlag<=(NextFlag & ~ACT)| RDY;

// transaction activation
ACTReg<=(ACTReg & ~MemNEXT) | (CtrlMachine==CCRS) | ((CtrlMachine==CDRS) & ~FifoFullFlag & ~MAERRFlag) | (CtrlMachine==CVRS) | (~OffsetFifoEmpty);
// select address of transaction
if (CtrlNext)
	begin
	AddrReg<=({CtrlOffsetReg,1'b0} & {35{((CtrlMachine==CCRS)|(CtrlMachine==CDRS)) & OffsetFifoEmpty}}) |
			({3'd0,DispReg} & {35{(CtrlMachine==CVRS)}}) |
			((DataOffsetReg+{3'd0,OffsetFifoBus}) & {35{~OffsetFifoEmpty}});
	MemSELReg<=(CtrlSelReg & {3{((CtrlMachine==CCRS)|(CtrlMachine==CDRS)) & OffsetFifoEmpty}}) | (DataSelReg & {3{(CtrlMachine==CVRS)}}) |
				(DataSelReg & {3{~OffsetFifoEmpty}});
	TagReg<=(CtrlMachine==CVRS) | ~OffsetFifoEmpty;
	SizeReg<=~(CtrlMachine==CVRS) & OffsetFifoEmpty;
	end

end

endmodule
