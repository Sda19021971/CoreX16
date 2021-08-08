
module MultichannelFIFO #(parameter Channels=16, Width=32, Depth=32) (
	input wire CLK, RESET, TCK,
	// write interface
	input wire [clogb2(Channels)-1:0] WCS,
	output reg WFULL,
	output reg WEMPTY,
	input wire WR,
	input wire [Width-1:0] DI,
	// read interface
	input wire [clogb2(Channels)-1:0] RCS,
	output reg RFULL,
	output reg REMPTY,
	input wire RD,
	output wire [Width-1:0] DO,
	// transaction skip interface
	output reg [clogb2(Channels)-1:0] STT,
	output reg SKIP
	);

localparam CW=clogb2(Channels)-1;
localparam DW=clogb2(Depth)-1;

integer i, j, k, l, m, n;
	
// array of pointers
reg [DW:0] WPtr [Channels-1:0];
reg [DW:0] RPtr [Channels-1:0];
reg [DW:0] Cntr [Channels-1:0];

// timers
reg [3:0] Tmr [Channels-1:0];
logic [CW:0] PosBus;
reg [Channels-1:0] TmrFlag;
reg [CW:0] TmrPtr;

logic [Channels-1:0] WriteWire, ReadWire;
reg [Width-1:0] FRAMBus;

// memory for fifo buffers

reg [Width-1:0] FRAM [0:Depth*Channels-1];

// output data
assign DO=FRAMBus;

//=================================================================================================
//			Asynchronous
//=================================================================================================
always_comb
begin

for (i=0;i<Channels;i=i+1)
	begin
	WriteWire[i]=WR & (WCS==i) & ~WFULL;
	ReadWire[i]=RD & (RCS==i) & ~REMPTY;
	end

PosBus[CW:0]=0;
for (n=0; n<Channels; n=n+1)
	if (TmrFlag[n]) PosBus[CW:0]=n[CW:0];

end

//=================================================================================================
//			Clocked part
//=================================================================================================
always_ff @(posedge CLK)
begin

if (WR & ~WFULL) FRAM[{WCS,WPtr[WCS]}]<=DI;
FRAMBus<=FRAM[{RCS,RPtr[RCS]}];

// state flags for input side
WFULL<=(Cntr[WCS][CW:1]=={CW{1'b1}});
WEMPTY<=(Cntr[WCS]==0);

// state flags for output side
RFULL<=(Cntr[RCS][CW:1]=={CW{1'b1}});
REMPTY<=(Cntr[RCS]==0);

// witre pointers
for (j=0;j<Channels;j=j+1)
	if (~RESET) WPtr[j]<=0;
		else if (WriteWire[j]) WPtr[j]<=WPtr[j]+{{CW{1'd0}},1'd1};

// read pointers
for (k=0;k<Channels;k=k+1)
	if (~RESET) RPtr[k]<=0;
		else if (ReadWire[k]) RPtr[k]<=RPtr[k]+{{CW{1'd0}},1'd1};

// counters
for (l=0;l<Channels;l=l+1)
	if (~RESET) Cntr[l]<=0;
		else if (WriteWire[l] | ReadWire[l]) Cntr[l]<=Cntr[l]+{{CW{1'd0}},WriteWire[l]}-{{CW{1'd0}},ReadWire[l]};

// timers
for (m=0; m<Channels; m=m+1)
	begin
	if (~RESET | ReadWire[m]) Tmr[m]<=0;
		else if (TCK & (RPtr[m] != WPtr[m]) & (TmrPtr==m[CW:0])) Tmr[m]<=Tmr[m]+4'd1;
	TmrFlag[m]<=(Tmr[m]==15) & TCK & ~ReadWire[m] & RESET & (TmrPtr==m[CW:0]);
	end
SKIP<=(|TmrFlag) & RESET;
if (|TmrFlag) STT<=PosBus;

// timer pointer
if (~RESET) TmrPtr<=0;
	else if (TCK) TmrPtr<=TmrPtr+{{CW-1{1'b0}},1'b1};
	
end
	
function integer clogb2;
    input [31:0] value;
    begin
        value = value - 1;
        for (clogb2 = 0; value > 0; clogb2 = clogb2 + 1) begin
            value = value >> 1;
        end
    end
endfunction

endmodule
