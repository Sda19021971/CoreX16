
module sc_fifo #(parameter LPM_WIDTH=16, LPM_NUMWORDS=16, LPM_WIDTHU=4) (
	input wire [LPM_WIDTH-1:0] data,
	input wire wrreq, rdreq, clock, sclr,
	output wire [LPM_WIDTH-1:0] q,
	output wire empty, full, almost_empty,
	output wire [LPM_WIDTHU-1:0] usedw
	);

reg [LPM_WIDTH-1:0] FifoMemory [0:LPM_NUMWORDS-1];		// FIFO memory
reg [LPM_WIDTHU-1:0] MemoryWAddrReg;
reg [LPM_WIDTH-1:0] MemoryDataReg;
reg MemoryWriteFlag;
logic [LPM_WIDTH-1:0] FifoMemoryBus;
logic BypassFlag, FullFlag;

reg [LPM_WIDTHU-1:0] WPtr, RPtr;						// write pointer, read pointer
reg [LPM_WIDTH-1:0] DataReg [1:0];
reg [1:0] BusyFlag;
reg [LPM_WIDTHU-1:0] Cntr;

assign usedw=Cntr;
assign q=DataReg[0];
assign empty=~BusyFlag[0];
assign almost_empty=~Cntr[LPM_WIDTHU-1];
assign full=FullFlag;

always_comb begin
FifoMemoryBus=FifoMemory[RPtr];
BypassFlag=MemoryWriteFlag & (MemoryWAddrReg==RPtr);
FullFlag=&Cntr[LPM_WIDTHU-1:1];
end

always_ff @(posedge clock)
begin

// word counter
if (sclr) Cntr<={LPM_WIDTHU{1'b0}};
	else Cntr<=Cntr+{{LPM_WIDTHU-1{1'b0}},wrreq & ~FullFlag}-{{LPM_WIDTHU-1{1'b0}},rdreq & (|Cntr)};

// busy flags
if (sclr) BusyFlag[0]<=1'b0;
	else BusyFlag[0]<=(BusyFlag[0] & ~rdreq) | wrreq | (Cntr>1);
if (sclr) BusyFlag[1]<=1'b0;
	else BusyFlag[1]<=(BusyFlag[1] & ~rdreq) | (wrreq & BusyFlag[0] & (~rdreq | BusyFlag[1])) | (Cntr>2);
	
// data registers
if (~BusyFlag[0] | rdreq) DataReg[0]<=(|Cntr[LPM_WIDTHU-1:1]) ? DataReg[1] : data;
if (~BusyFlag[1] | rdreq) DataReg[1]<=(Cntr<=2) ? data : (BypassFlag ? MemoryDataReg : FifoMemoryBus);

// memory pointers
if (sclr) WPtr<=0;
	else if (wrreq & ~FullFlag & ((Cntr>2) | ((Cntr==2) & ~rdreq))) WPtr<=WPtr+{{LPM_WIDTHU-1{1'b0}},1'b1};
if (sclr) RPtr<=0;
	else if (rdreq & (Cntr>2)) RPtr<=RPtr+{{LPM_WIDTHU-1{1'b0}},1'b1};

// memory buffer
MemoryWriteFlag<=(wrreq & ~FullFlag & ((Cntr>2) | ((Cntr==2) & ~rdreq))) & ~sclr;
MemoryWAddrReg<=WPtr;
MemoryDataReg<=data;
if (MemoryWriteFlag) FifoMemory[MemoryWAddrReg] <= MemoryDataReg;

end

endmodule
