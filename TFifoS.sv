module TFifoS #(parameter Width=32) (
	input wire CLK, RESET, ACTL, NEXTH,
	output wire NEXTL,
	output reg ACTH,
	input wire [Width-1:0] DI,
	output reg [Width-1:0] DO
);

reg [Width-1:0] InReg [1:0];
reg InSelFlag, OutSelFlag;
reg [1:0] BusyFlag;
logic [1:0] ReadWire;


assign NEXTL=~(&BusyFlag);


//=================================================================================================
//				Asynchronous
//=================================================================================================
always_comb
begin

ReadWire[0]=~OutSelFlag & BusyFlag[0] & (~ACTH | NEXTH);
ReadWire[1]=OutSelFlag & BusyFlag[1] & (~ACTH | NEXTH);

end

//=================================================================================================
//				 Clocked by CLK
//=================================================================================================
always_ff @(posedge CLK)
begin

// Busy flags
BusyFlag[0]<=(BusyFlag[0] | (ACTL & ~InSelFlag)) & RESET & ~ReadWire[0];
BusyFlag[1]<=(BusyFlag[1] | (ACTL & InSelFlag)) & RESET & ~ReadWire[1];

// data registers
if (ACTL & ~InSelFlag & ~BusyFlag[0]) InReg[0]<=DI;
if (ACTL & InSelFlag & ~BusyFlag[1]) InReg[1]<=DI;

// selection flag
if (~RESET) InSelFlag<=1'b0;
	else if ((ACTL & ~InSelFlag & ~BusyFlag[0])|(ACTL & InSelFlag & ~BusyFlag[1])) InSelFlag<=~InSelFlag;

// output activation
ACTH<=((ACTH & ~NEXTH) | (BusyFlag[0] & ~OutSelFlag) | (BusyFlag[1] & OutSelFlag)) & RESET;

// output data
if (~ACTH | NEXTH) DO<=OutSelFlag ? InReg[1] : InReg[0];

// output selection
if (~RESET) OutSelFlag<=1'b0;
	else if ((~ACTH | NEXTH) & (OutSelFlag ? BusyFlag[1] : BusyFlag[0])) OutSelFlag<=~OutSelFlag;

end

endmodule
