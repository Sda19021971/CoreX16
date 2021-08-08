module Sequencer (
		input wire CLK, RESET,
		input wire IRDY,
		input wire [3:0] VF,
		input wire [63:0] IBUS,
		output wire IRD,
		input wire [15:0] GPRVF,
		output wire [15:0] GPRINVD,
		output wire EMPTY,
		output wire MOVFLAG,
		input wire FDIVRDY,
		input wire FMULACCRDY,
		input wire MEMRDY,
		// instructions to the execute units
		output wire [13:0] FADDBus,
		output wire [12:0] FMULBus,
		output wire [19:0] FMULACCBus,
		output wire [13:0] FDIVBus,
		output wire [24:0] ALUBus,
		output wire [12:0] ShiftBus,
		output wire [11:0] MiscBus,
		output wire [15:0] LoopBus,
		output wire [14:0] MovBus,
		output wire [15:0] MemBus,
		output wire [16:0] CtrlBus
		);

integer i, i1, j, j0,i4;

reg [15:0] InstReg [7:0];	// instruction register
reg [10:0] TypeReg [7:0]; 	// instruction types
reg [15:0] GPRInvdR [7:0];		// invalidation flags
reg [2:0] OpReg [7:0];
reg [3:0] ExtReg;
reg InstReloadWire, HalfReloadWire;
reg [7:0] LockNode, VFReg, VFResetWire;
reg [13:0] FADDBusWire, FDIVBusWire;
reg [12:0] FMULBusWire, ShiftBusWire;
reg [19:0] FMULACCBusWire;
reg [3:0] FMULLACCCMDWire;
reg [7:0] FDIVCMDWire;
reg [24:0] ALUBusWire;
reg [11:0] MiscBusWire;
reg [15:0] LoopBusWire;
reg [14:0] MovBusWire;
reg [15:0] MemBusWire;
reg [16:0] CtrlBusWire;
reg [1:0] ALUCMDWire [7:0];
reg [3:0] SHTCMDWire [7:0];
reg [2:0] MiscCMDWire [7:0];
reg [1:0] MovCMDWire [7:0];
reg [3:0] MemCMDWire [7:0];
reg [15:0] InvdBus;

reg I1SRC1_I0DST, I1SRC2_I0DST, I1DST_I0SRC1, I1DST_I0SRC2, I1DST_I0DST;

reg I2SRC1_I1DST, I2SRC2_I1DST, I2DST_I1SRC1, I2DST_I1SRC2, I2DST_I1DST,
	I2SRC1_I0DST, I2SRC2_I0DST, I2DST_I0SRC1, I2DST_I0SRC2, I2DST_I0DST;

reg	I3SRC1_I2DST, I3SRC2_I2DST, I3DST_I2SRC1, I3DST_I2SRC2, I3DST_I2DST,
	I3SRC1_I1DST, I3SRC2_I1DST, I3DST_I1SRC1, I3DST_I1SRC2, I3DST_I1DST,
	I3SRC1_I0DST, I3SRC2_I0DST, I3DST_I0SRC1, I3DST_I0SRC2, I3DST_I0DST;

reg	I4SRC1_I3DST, I4SRC2_I3DST, I4DST_I3SRC1, I4DST_I3SRC2, I4DST_I3DST,
	I4SRC1_I2DST, I4SRC2_I2DST, I4DST_I2SRC1, I4DST_I2SRC2, I4DST_I2DST,
	I4SRC1_I1DST, I4SRC2_I1DST, I4DST_I1SRC1, I4DST_I1SRC2, I4DST_I1DST,
	I4SRC1_I0DST, I4SRC2_I0DST, I4DST_I0SRC1, I4DST_I0SRC2, I4DST_I0DST;

reg	I5SRC1_I4DST, I5SRC2_I4DST, I5DST_I4SRC1, I5DST_I4SRC2, I5DST_I4DST,
	I5SRC1_I3DST, I5SRC2_I3DST, I5DST_I3SRC1, I5DST_I3SRC2, I5DST_I3DST,
	I5SRC1_I2DST, I5SRC2_I2DST, I5DST_I2SRC1, I5DST_I2SRC2, I5DST_I2DST,
	I5SRC1_I1DST, I5SRC2_I1DST, I5DST_I1SRC1, I5DST_I1SRC2, I5DST_I1DST,
	I5SRC1_I0DST, I5SRC2_I0DST, I5DST_I0SRC1, I5DST_I0SRC2, I5DST_I0DST;

reg	I6SRC1_I5DST, I6SRC2_I5DST, I6DST_I5SRC1, I6DST_I5SRC2, I6DST_I5DST,
	I6SRC1_I4DST, I6SRC2_I4DST, I6DST_I4SRC1, I6DST_I4SRC2, I6DST_I4DST,
	I6SRC1_I3DST, I6SRC2_I3DST, I6DST_I3SRC1, I6DST_I3SRC2, I6DST_I3DST,
	I6SRC1_I2DST, I6SRC2_I2DST, I6DST_I2SRC1, I6DST_I2SRC2, I6DST_I2DST,
	I6SRC1_I1DST, I6SRC2_I1DST, I6DST_I1SRC1, I6DST_I1SRC2, I6DST_I1DST,
	I6SRC1_I0DST, I6SRC2_I0DST, I6DST_I0SRC1, I6DST_I0SRC2, I6DST_I0DST;

reg	I7SRC1_I6DST, I7SRC2_I6DST, I7DST_I6SRC1, I7DST_I6SRC2, I7DST_I6DST,
	I7SRC1_I5DST, I7SRC2_I5DST, I7DST_I5SRC1, I7DST_I5SRC2, I7DST_I5DST,
	I7SRC1_I4DST, I7SRC2_I4DST, I7DST_I4SRC1, I7DST_I4SRC2, I7DST_I4DST,
	I7SRC1_I3DST, I7SRC2_I3DST, I7DST_I3SRC1, I7DST_I3SRC2, I7DST_I3DST, 
	I7SRC1_I2DST, I7SRC2_I2DST, I7DST_I2SRC1, I7DST_I2SRC2, I7DST_I2DST,
	I7SRC1_I1DST, I7SRC2_I1DST, I7DST_I1SRC1, I7DST_I1SRC2, I7DST_I1DST,
	I7SRC1_I0DST, I7SRC2_I0DST, I7DST_I0SRC1, I7DST_I0SRC2, I7DST_I0DST;


/*
====================================================================================================
				 	Asynchronous part
====================================================================================================
*/

assign FADDBus=FADDBusWire;
assign FMULBus=FMULBusWire;
assign FMULACCBus=FMULACCBusWire;
assign FDIVBus=FDIVBusWire;
assign ALUBus=ALUBusWire;
assign ShiftBus=ShiftBusWire;
assign MiscBus=MiscBusWire;
assign LoopBus=LoopBusWire;
assign MovBus=MovBusWire;
assign MemBus=MemBusWire;
assign CtrlBus=CtrlBusWire;
assign GPRINVD=InvdBus;
assign IRD=InstReloadWire | HalfReloadWire;
// empty flag
assign EMPTY=~(|VFReg);
// movement present
assign MOVFLAG=TypeReg[0][7] | TypeReg[1][7] | TypeReg[2][7] | TypeReg[3][7] | TypeReg[4][7] | TypeReg[5][7] | TypeReg[6][7] | TypeReg[7][7];

always @*
	begin

	// Lock nodes
	LockNode[0]=(~GPRVF[InstReg[0][7:4]] & OpReg[0][0])|
				(~GPRVF[InstReg[0][11:8]] & OpReg[0][1])|
				(~GPRVF[InstReg[0][15:12]] & OpReg[0][2]);

	LockNode[1]=(~GPRVF[InstReg[1][7:4]] & OpReg[1][0])|
				(~GPRVF[InstReg[1][11:8]] & OpReg[1][1])|
				(~GPRVF[InstReg[1][15:12]] & OpReg[1][2])|				
				(~GPRVF[InstReg[0][11:8]] & OpReg[0][1] & ExtReg[0])|
				(~GPRVF[InstReg[0][15:12]] & OpReg[0][2] & ExtReg[0])|
				(I1SRC1_I0DST & OpReg[1][0] & OpReg[0][2] & VFReg[0])|
				(I1SRC2_I0DST & OpReg[1][1] & OpReg[0][2] & VFReg[0])|
				(I1DST_I0SRC1 & OpReg[1][2] & OpReg[0][0] & VFReg[0])|
				(I1DST_I0SRC2 & OpReg[1][2] & OpReg[0][1] & VFReg[0])|
				(I1DST_I0DST & OpReg[1][2] & OpReg[0][2] & VFReg[0]);

	LockNode[2]=(~GPRVF[InstReg[2][7:4]] & OpReg[2][0])|
				(~GPRVF[InstReg[2][11:8]] & OpReg[2][1])|
				(~GPRVF[InstReg[2][15:12]] & OpReg[2][2])|
				(I2SRC1_I1DST & OpReg[2][0] & OpReg[1][2] & VFReg[1])|
				(I2SRC2_I1DST & OpReg[2][1] & OpReg[1][2] & VFReg[1])|
				(I2DST_I1SRC1 & OpReg[2][2] & OpReg[1][0] & VFReg[1])|
				(I2DST_I1SRC2 & OpReg[2][2] & OpReg[1][1] & VFReg[1])|
				(I2DST_I1DST & OpReg[2][2] & OpReg[1][2] & VFReg[1])|
				(I2SRC1_I0DST & OpReg[2][0] & OpReg[0][2] & VFReg[0])|
				(I2SRC2_I0DST & OpReg[2][1] & OpReg[0][2] & VFReg[0])|
				(I2DST_I0SRC1 & OpReg[2][2] & OpReg[0][0] & VFReg[0])|
				(I2DST_I0SRC2 & OpReg[2][2] & OpReg[0][1] & VFReg[0])|
				(I2DST_I0DST & OpReg[2][2] & OpReg[0][2] & VFReg[0]);

	LockNode[3]=(~GPRVF[InstReg[3][7:4]] & OpReg[3][0])|
				(~GPRVF[InstReg[3][11:8]] & OpReg[3][1])|
				(~GPRVF[InstReg[3][15:12]] & OpReg[3][2])|
				(~GPRVF[InstReg[2][11:8]] & OpReg[2][1] & ExtReg[1])|
				(~GPRVF[InstReg[2][15:12]] & OpReg[2][2] & ExtReg[1])|
				(I3SRC1_I2DST & OpReg[3][0] & OpReg[2][2] & VFReg[2])|
				(I3SRC2_I2DST & OpReg[3][1] & OpReg[2][2] & VFReg[2])|
				(I3DST_I2SRC1 & OpReg[3][2] & OpReg[2][0] & VFReg[2])|
				(I3DST_I2SRC2 & OpReg[3][2] & OpReg[2][1] & VFReg[2])|
				(I3DST_I2DST & OpReg[3][2] & OpReg[2][2] & VFReg[2])|
				(I3SRC1_I1DST & OpReg[3][0] & OpReg[1][2] & VFReg[1])|
				(I3SRC2_I1DST & OpReg[3][1] & OpReg[1][2] & VFReg[1])|
				(I3DST_I1SRC1 & OpReg[3][2] & OpReg[1][0] & VFReg[1])|
				(I3DST_I1SRC2 & OpReg[3][2] & OpReg[1][1] & VFReg[1])|
				(I3DST_I1DST & OpReg[3][2] & OpReg[1][2] & VFReg[1])|
				(I3SRC1_I0DST & OpReg[3][0] & OpReg[0][2] & VFReg[0])|
				(I3SRC2_I0DST & OpReg[3][1] & OpReg[0][2] & VFReg[0])|
				(I3DST_I0SRC1 & OpReg[3][2] & OpReg[0][0] & VFReg[0])|
				(I3DST_I0SRC2 & OpReg[3][2] & OpReg[0][1] & VFReg[0])|
				(I3DST_I0DST & OpReg[3][2] & OpReg[0][2] & VFReg[0]);

	LockNode[4]=(~GPRVF[InstReg[4][7:4]] & OpReg[4][0])|
				(~GPRVF[InstReg[4][11:8]] & OpReg[4][1])|
				(~GPRVF[InstReg[4][15:12]] & OpReg[4][2])|
				(I4SRC1_I3DST & OpReg[4][0] & OpReg[3][2] & VFReg[3])|
				(I4SRC2_I3DST & OpReg[4][1] & OpReg[3][2] & VFReg[3])|
				(I4DST_I3SRC1 & OpReg[4][2] & OpReg[3][0] & VFReg[3])|
				(I4DST_I3SRC2 & OpReg[4][2] & OpReg[3][1] & VFReg[3])|
				(I4DST_I3DST & OpReg[4][2] & OpReg[3][2] & VFReg[3])|
				(I4SRC1_I2DST & OpReg[4][0] & OpReg[2][2] & VFReg[2])|
				(I4SRC2_I2DST & OpReg[4][1] & OpReg[2][2] & VFReg[2])|
				(I4DST_I2SRC1 & OpReg[4][2] & OpReg[2][0] & VFReg[2])|
				(I4DST_I2SRC2 & OpReg[4][2] & OpReg[2][1] & VFReg[2])|
				(I4DST_I2DST & OpReg[4][2] & OpReg[2][2] & VFReg[2])|
				(I4SRC1_I1DST & OpReg[4][0] & OpReg[1][2] & VFReg[1])|
				(I4SRC2_I1DST & OpReg[4][1] & OpReg[1][2] & VFReg[1])|
				(I4DST_I1SRC1 & OpReg[4][2] & OpReg[1][0] & VFReg[1])|
				(I4DST_I1SRC2 & OpReg[4][2] & OpReg[1][1] & VFReg[1])|
				(I4DST_I1DST & OpReg[4][2] & OpReg[1][2] & VFReg[1])|
				(I4SRC1_I0DST & OpReg[4][0] & OpReg[0][2] & VFReg[0])|
				(I4SRC2_I0DST & OpReg[4][1] & OpReg[0][2] & VFReg[0])|
				(I4DST_I0SRC1 & OpReg[4][2] & OpReg[0][0] & VFReg[0])|
				(I4DST_I0SRC2 & OpReg[4][2] & OpReg[0][1] & VFReg[0])|
				(I4DST_I0DST & OpReg[4][2] & OpReg[0][2] & VFReg[0]);

	LockNode[5]=(~GPRVF[InstReg[5][7:4]] & OpReg[5][0])|
				(~GPRVF[InstReg[5][11:8]] & OpReg[5][1])|
				(~GPRVF[InstReg[5][15:12]] & OpReg[5][2])|
				(~GPRVF[InstReg[4][11:8]] & OpReg[4][1] & ExtReg[2])|
				(~GPRVF[InstReg[4][15:12]] & OpReg[4][2] & ExtReg[2])|
				(I5SRC1_I4DST & OpReg[5][0] & OpReg[4][2] & VFReg[4])|
				(I5SRC2_I4DST & OpReg[5][1] & OpReg[4][2] & VFReg[4])|
				(I5DST_I4SRC1 & OpReg[5][2] & OpReg[4][0] & VFReg[4])|
				(I5DST_I4SRC2 & OpReg[5][2] & OpReg[4][1] & VFReg[4])|
				(I5DST_I4DST & OpReg[5][2] & OpReg[4][2] & VFReg[4])|
				(I5SRC1_I3DST & OpReg[5][0] & OpReg[3][2] & VFReg[3])|
				(I5SRC2_I3DST & OpReg[5][1] & OpReg[3][2] & VFReg[3])|
				(I5DST_I3SRC1 & OpReg[5][2] & OpReg[3][0] & VFReg[3])|
				(I5DST_I3SRC2 & OpReg[5][2] & OpReg[3][1] & VFReg[3])|
				(I5DST_I3DST & OpReg[5][2] & OpReg[3][2] & VFReg[3])|
				(I5SRC1_I2DST & OpReg[5][0] & OpReg[2][2] & VFReg[2])|
				(I5SRC2_I2DST & OpReg[5][1] & OpReg[2][2] & VFReg[2])|
				(I5DST_I2SRC1 & OpReg[5][2] & OpReg[2][0] & VFReg[2])|
				(I5DST_I2SRC2 & OpReg[5][2] & OpReg[2][1] & VFReg[2])|
				(I5DST_I2DST & OpReg[5][2] & OpReg[2][2] & VFReg[2])|
				(I5SRC1_I1DST & OpReg[5][0] & OpReg[1][2] & VFReg[1])|
				(I5SRC2_I1DST & OpReg[5][1] & OpReg[1][2] & VFReg[1])|
				(I5DST_I1SRC1 & OpReg[5][2] & OpReg[1][0] & VFReg[1])|
				(I5DST_I1SRC2 & OpReg[5][2] & OpReg[1][1] & VFReg[1])|
				(I5DST_I1DST & OpReg[5][2] & OpReg[1][2] & VFReg[1])|
				(I5SRC1_I0DST & OpReg[5][0] & OpReg[0][2] & VFReg[0])|
				(I5SRC2_I0DST & OpReg[5][1] & OpReg[0][2] & VFReg[0])|
				(I5DST_I0SRC1 & OpReg[5][2] & OpReg[0][0] & VFReg[0])|
				(I5DST_I0SRC2 & OpReg[5][2] & OpReg[0][1] & VFReg[0])|
				(I5DST_I0DST & OpReg[5][2] & OpReg[0][2] & VFReg[0]);

	LockNode[6]=(~GPRVF[InstReg[6][7:4]] & OpReg[6][0])|
				(~GPRVF[InstReg[6][11:8]] & OpReg[6][1])|
				(~GPRVF[InstReg[6][15:12]] & OpReg[6][2])|
				(I6SRC1_I5DST & OpReg[6][0] & OpReg[5][2] & VFReg[5])|
				(I6SRC2_I5DST & OpReg[6][1] & OpReg[5][2] & VFReg[5])|
				(I6DST_I5SRC1 & OpReg[6][2] & OpReg[5][0] & VFReg[5])|
				(I6DST_I5SRC2 & OpReg[6][2] & OpReg[5][1] & VFReg[5])|
				(I6DST_I5DST & OpReg[6][2] & OpReg[5][2] & VFReg[5])|
				(I6SRC1_I4DST & OpReg[6][0] & OpReg[4][2] & VFReg[4])|
				(I6SRC2_I4DST & OpReg[6][1] & OpReg[4][2] & VFReg[4])|
				(I6DST_I4SRC1 & OpReg[6][2] & OpReg[4][0] & VFReg[4])|
				(I6DST_I4SRC2 & OpReg[6][2] & OpReg[4][1] & VFReg[4])|
				(I6DST_I4DST & OpReg[6][2] & OpReg[4][2] & VFReg[4])|
				(I6SRC1_I3DST & OpReg[6][0] & OpReg[3][2] & VFReg[3])|
				(I6SRC2_I3DST & OpReg[6][1] & OpReg[3][2] & VFReg[3])|
				(I6DST_I3SRC1 & OpReg[6][2] & OpReg[3][0] & VFReg[3])|
				(I6DST_I3SRC2 & OpReg[6][2] & OpReg[3][1] & VFReg[3])|
				(I6DST_I3DST & OpReg[6][2] & OpReg[3][2] & VFReg[3])|
				(I6SRC1_I2DST & OpReg[6][0] & OpReg[2][2] & VFReg[2])|
				(I6SRC2_I2DST & OpReg[6][1] & OpReg[2][2] & VFReg[2])|
				(I6DST_I2SRC1 & OpReg[6][2] & OpReg[2][0] & VFReg[2])|
				(I6DST_I2SRC2 & OpReg[6][2] & OpReg[2][1] & VFReg[2])|
				(I6DST_I2DST & OpReg[6][2] & OpReg[2][2] & VFReg[2])|
				(I6SRC1_I1DST & OpReg[6][0] & OpReg[1][2] & VFReg[1])|
				(I6SRC2_I1DST & OpReg[6][1] & OpReg[1][2] & VFReg[1])|
				(I6DST_I1SRC1 & OpReg[6][2] & OpReg[1][0] & VFReg[1])|
				(I6DST_I1SRC2 & OpReg[6][2] & OpReg[1][1] & VFReg[1])|
				(I6DST_I1DST & OpReg[6][2] & OpReg[1][2] & VFReg[1])|
				(I6SRC1_I0DST & OpReg[6][0] & OpReg[0][2] & VFReg[0])|
				(I6SRC2_I0DST & OpReg[6][1] & OpReg[0][2] & VFReg[0])|
				(I6DST_I0SRC1 & OpReg[6][2] & OpReg[0][0] & VFReg[0])|
				(I6DST_I0SRC2 & OpReg[6][2] & OpReg[0][1] & VFReg[0])|
				(I6DST_I0DST & OpReg[6][2] & OpReg[0][2] & VFReg[0]);

	LockNode[7]=(~GPRVF[InstReg[7][7:4]] & OpReg[7][0])|
				(~GPRVF[InstReg[7][11:8]] & OpReg[7][1])|
				(~GPRVF[InstReg[7][15:12]] & OpReg[7][2])|
				(~GPRVF[InstReg[6][11:8]] & OpReg[6][1] & ExtReg[3])|
				(~GPRVF[InstReg[6][15:12]] & OpReg[6][2] & ExtReg[3])|
				(I7SRC1_I6DST & OpReg[7][0] & OpReg[6][2] & VFReg[6])|
				(I7SRC2_I6DST & OpReg[7][1] & OpReg[6][2] & VFReg[6])|
				(I7DST_I6SRC1 & OpReg[7][2] & OpReg[6][0] & VFReg[6])|
				(I7DST_I6SRC2 & OpReg[7][2] & OpReg[6][1] & VFReg[6])|
				(I7DST_I6DST & OpReg[7][2] & OpReg[6][2] & VFReg[6])|
				(I7SRC1_I5DST & OpReg[7][0] & OpReg[5][2] & VFReg[5])|
				(I7SRC2_I5DST & OpReg[7][1] & OpReg[5][2] & VFReg[5])|
				(I7DST_I5SRC1 & OpReg[7][2] & OpReg[5][0] & VFReg[5])|
				(I7DST_I5SRC2 & OpReg[7][2] & OpReg[5][1] & VFReg[5])|
				(I7DST_I5DST & OpReg[7][2] & OpReg[5][2] & VFReg[5])|
				(I7SRC1_I4DST & OpReg[7][0] & OpReg[4][2] & VFReg[4])|
				(I7SRC2_I4DST & OpReg[7][1] & OpReg[4][2] & VFReg[4])|
				(I7DST_I4SRC1 & OpReg[7][2] & OpReg[4][0] & VFReg[4])|
				(I7DST_I4SRC2 & OpReg[7][2] & OpReg[4][1] & VFReg[4])|
				(I7DST_I4DST & OpReg[7][2] & OpReg[4][2] & VFReg[4])|
				(I7SRC1_I3DST & OpReg[7][0] & OpReg[3][2] & VFReg[3])|
				(I7SRC2_I3DST & OpReg[7][1] & OpReg[3][2] & VFReg[3])|
				(I7DST_I3SRC1 & OpReg[7][2] & OpReg[3][0] & VFReg[3])|
				(I7DST_I3SRC2 & OpReg[7][2] & OpReg[3][1] & VFReg[3])|
				(I7DST_I3DST & OpReg[7][2] & OpReg[3][2] & VFReg[3])|
				(I7SRC1_I2DST & OpReg[7][0] & OpReg[2][2] & VFReg[2])|
				(I7SRC2_I2DST & OpReg[7][1] & OpReg[2][2] & VFReg[2])|
				(I7DST_I2SRC1 & OpReg[7][2] & OpReg[2][0] & VFReg[2])|
				(I7DST_I2SRC2 & OpReg[7][2] & OpReg[2][1] & VFReg[2])|
				(I7DST_I2DST & OpReg[7][2] & OpReg[2][2] & VFReg[2])|
				(I7SRC1_I1DST & OpReg[7][0] & OpReg[1][2] & VFReg[1])|
				(I7SRC2_I1DST & OpReg[7][1] & OpReg[1][2] & VFReg[1])|
				(I7DST_I1SRC1 & OpReg[7][2] & OpReg[1][0] & VFReg[1])|
				(I7DST_I1SRC2 & OpReg[7][2] & OpReg[1][1] & VFReg[1])|
				(I7DST_I1DST & OpReg[7][2] & OpReg[1][2] & VFReg[1])|
				(I7SRC1_I0DST & OpReg[7][0] & OpReg[0][2] & VFReg[0])|
				(I7SRC2_I0DST & OpReg[7][1] & OpReg[0][2] & VFReg[0])|
				(I7DST_I0SRC1 & OpReg[7][2] & OpReg[0][0] & VFReg[0])|
				(I7DST_I0SRC2 & OpReg[7][2] & OpReg[0][1] & VFReg[0])|
				(I7DST_I0DST & OpReg[7][2] & OpReg[0][2] & VFReg[0]);

	// Instruction patch for FADD unit
	FADDBusWire[0]=(TypeReg[0][0] & ~LockNode[0])|(TypeReg[1][0] & ~LockNode[1])|
					(TypeReg[2][0] & ~LockNode[2])|(TypeReg[3][0] & ~LockNode[3])|
					(TypeReg[4][0] & ~LockNode[4])|(TypeReg[5][0] & ~LockNode[5])|
					(TypeReg[6][0] & ~LockNode[6])|(TypeReg[7][0] & ~LockNode[7]);
	FADDBusWire[1]=(InstReg[0][0] & TypeReg[0][0] & ~LockNode[0])|
					(InstReg[1][0] & TypeReg[1][0] & ~LockNode[1] & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[2][0] & TypeReg[2][0] & ~LockNode[2] & 
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[3][0] & TypeReg[3][0] & ~LockNode[3] & (~TypeReg[2][0] | LockNode[2]) &
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[4][0] & TypeReg[4][0] & ~LockNode[4] & (~TypeReg[3][0] | LockNode[3]) &
							(~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) & 
							(~TypeReg[0][0] | LockNode[0]))|
					(InstReg[5][0] & TypeReg[5][0] & ~LockNode[5] & (~TypeReg[4][0] | LockNode[4]) &
							(~TypeReg[3][0] | LockNode[3]) & (~TypeReg[2][0] | LockNode[2]) & 
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[6][0] & TypeReg[6][0] & ~LockNode[6] & (~TypeReg[5][0] | LockNode[5]) &
							(~TypeReg[4][0] | LockNode[4]) & (~TypeReg[3][0] | LockNode[3]) & 
							(~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) & 
							(~TypeReg[0][0] | LockNode[0]))|
					(InstReg[7][0] & TypeReg[7][0] & ~LockNode[7] & (~TypeReg[6][0] | LockNode[6]) &
							(~TypeReg[5][0] | LockNode[5]) & (~TypeReg[4][0] | LockNode[4]) &
							(~TypeReg[3][0] | LockNode[3]) & (~TypeReg[2][0] | LockNode[2]) & 
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]));
	for (i=0; i<12; i=i+1)
		begin
		FADDBusWire[i+2]=(InstReg[0][i+4] & TypeReg[0][0] & ~LockNode[0])|
					(InstReg[1][i+4] & TypeReg[1][0] & ~LockNode[1] & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[2][i+4] & TypeReg[2][0] & ~LockNode[2] & 
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[3][i+4] & TypeReg[3][0] & ~LockNode[3] & (~TypeReg[2][0] | LockNode[2]) &
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[4][i+4] & TypeReg[4][0] & ~LockNode[4] & (~TypeReg[3][0] | LockNode[3]) &
							(~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) & 
							(~TypeReg[0][0] | LockNode[0]))|
					(InstReg[5][i+4] & TypeReg[5][0] & ~LockNode[5] & (~TypeReg[4][0] | LockNode[4]) &
							(~TypeReg[3][0] | LockNode[3]) & (~TypeReg[2][0] | LockNode[2]) & 
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(InstReg[6][i+4] & TypeReg[6][0] & ~LockNode[6] & (~TypeReg[5][0] | LockNode[5]) &
							(~TypeReg[4][0] | LockNode[4]) & (~TypeReg[3][0] | LockNode[3]) & 
							(~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) & 
							(~TypeReg[0][0] | LockNode[0]))|
					(InstReg[7][i+4] & TypeReg[7][0] & ~LockNode[7] & (~TypeReg[6][0] | LockNode[6]) &
							(~TypeReg[5][0] | LockNode[5]) & (~TypeReg[4][0] | LockNode[4]) &
							(~TypeReg[3][0] | LockNode[3]) & (~TypeReg[2][0] | LockNode[2]) & 
							(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]));
		end

	//
	// Instruction patch for FMUL unit
	//
	FMULBusWire[0]=(TypeReg[0][1] & ~LockNode[0])|(TypeReg[1][1] & ~LockNode[1])|(TypeReg[2][1] & ~LockNode[2])|
					(TypeReg[3][1] & ~LockNode[3])|(TypeReg[4][1] & ~LockNode[4])|(TypeReg[5][1] & ~LockNode[5])|
					(TypeReg[6][1] & ~LockNode[6])|(TypeReg[7][1] & ~LockNode[7]);
	for (i=0; i<12; i=i+1)
		FMULBusWire[i+1]=(InstReg[0][i+4] & TypeReg[0][1] & ~LockNode[0])|
					(InstReg[1][i+4] & TypeReg[1][1] & ~LockNode[1] & (~TypeReg[0][1] | LockNode[0]))|
					(InstReg[2][i+4] & TypeReg[2][1] & ~LockNode[2] & (~TypeReg[1][1] | LockNode[1]) &
						(~TypeReg[0][1] | LockNode[0]))|
					(InstReg[3][i+4] & TypeReg[3][1] & ~LockNode[3] & (~TypeReg[2][1] | LockNode[2]) &
						(~TypeReg[1][1] | LockNode[1]) & (~TypeReg[0][1] | LockNode[0]))|
					(InstReg[4][i+4] & TypeReg[4][1] & ~LockNode[4] & (~TypeReg[3][1] | LockNode[3]) &
						(~TypeReg[2][1] | LockNode[2]) & (~TypeReg[1][1] | LockNode[1]) & 
						(~TypeReg[0][1] | LockNode[0]))|
					(InstReg[5][i+4] & TypeReg[5][1] & ~LockNode[5] & (~TypeReg[4][1] | LockNode[4]) &
						(~TypeReg[3][1] | LockNode[3]) & (~TypeReg[2][1] | LockNode[2]) &
						(~TypeReg[1][1] | LockNode[1]) & (~TypeReg[0][1] | LockNode[0]))|
					(InstReg[6][i+4] & TypeReg[6][1] & ~LockNode[6] & (~TypeReg[5][1] | LockNode[5]) &
						(~TypeReg[4][1] | LockNode[4]) & (~TypeReg[3][1] | LockNode[3]) &
						(~TypeReg[2][1] | LockNode[2]) & (~TypeReg[1][1] | LockNode[1]) &
						(~TypeReg[0][1] | LockNode[0]))|
					(InstReg[7][i+4] & TypeReg[7][1] & ~LockNode[7] & (~TypeReg[6][1] | LockNode[6]) &
						(~TypeReg[5][1] | LockNode[5]) & (~TypeReg[4][1] | LockNode[4]) &
						(~TypeReg[3][1] | LockNode[3]) & (~TypeReg[2][1] | LockNode[2]) &
						(~TypeReg[1][1] | LockNode[1]) & (~TypeReg[0][1] | LockNode[0]));

	//=============================================================================================
	//
	// Instruction patch for FMULACC or FFT unit
	// 
	FMULACCBusWire[0]=(TypeReg[0][9] & ~LockNode[0] & ~LockNode[1])|(TypeReg[2][9] & ~LockNode[2] & ~LockNode[3])|
					(TypeReg[4][9] & ~LockNode[4] & ~LockNode[5])|(TypeReg[6][9] & ~LockNode[6] & ~LockNode[7]);
	// creating CMD Nodes
	FMULLACCCMDWire[0]=(InstReg[1][3:0]==4'd7);
	FMULLACCCMDWire[1]=(InstReg[3][3:0]==4'd7);
	FMULLACCCMDWire[2]=(InstReg[5][3:0]==4'd7);
	FMULLACCCMDWire[3]=(InstReg[7][3:0]==4'd7);

	for (i4=0; i4<4; i4=i4+1)
		begin
		// DST field
		FMULACCBusWire[i4+4]=(InstReg[0][i4+12] & TypeReg[0][9] & ~LockNode[0] & ~LockNode[1])|
								(InstReg[2][i4+12] & TypeReg[2][9] & ~LockNode[2] & ~LockNode[3] & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[4][i4+12] & TypeReg[4][9] & ~LockNode[4] & ~LockNode[5] &
									(~TypeReg[2][9] | LockNode[2] | LockNode[3]) & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[6][i4+12] & TypeReg[6][9] & ~LockNode[6] & ~LockNode[7] &
									(~TypeReg[4][9] | LockNode[4] | LockNode[5]) & (~TypeReg[2][9] | LockNode[2] |LockNode[3]) &
									(~TypeReg[0][9] | LockNode[0] | LockNode[1]));
		// CtrlOffset register field
		FMULACCBusWire[i4+8]=(InstReg[1][i4+4] & TypeReg[0][9] & ~LockNode[0] & ~LockNode[1])|
								(InstReg[3][i4+4] & TypeReg[2][9] & ~LockNode[2] & ~LockNode[3] & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[5][i4+4] & TypeReg[4][9] & ~LockNode[4] & ~LockNode[5] &
									(~TypeReg[2][9] | LockNode[2] | LockNode[3]) & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[7][i4+4] & TypeReg[6][9] & ~LockNode[6] & ~LockNode[7] &
									(~TypeReg[4][9] | LockNode[4] | LockNode[5]) & (~TypeReg[2][9] | LockNode[2] |LockNode[3]) &
									(~TypeReg[0][9] | LockNode[0] | LockNode[1]));
		// DataOffset register field
		FMULACCBusWire[i4+12]=(InstReg[1][i4+8] & TypeReg[0][9] & ~LockNode[0] & ~LockNode[1])|
								(InstReg[3][i4+8] & TypeReg[2][9] & ~LockNode[2] & ~LockNode[3] & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[5][i4+8] & TypeReg[4][9] & ~LockNode[4] & ~LockNode[5] &
									(~TypeReg[2][9] | LockNode[2] | LockNode[3]) & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[7][i4+8] & TypeReg[6][9] & ~LockNode[6] & ~LockNode[7] &
									(~TypeReg[4][9] | LockNode[4] | LockNode[5]) & (~TypeReg[2][9] | LockNode[2] |LockNode[3]) &
									(~TypeReg[0][9] | LockNode[0] | LockNode[1]));
		if (i4<3)
			begin
			// CTRLSEL field
			FMULACCBusWire[i4+1]=(InstReg[0][i4+8] & TypeReg[0][9] & ~LockNode[0] & ~LockNode[1])|
								(InstReg[2][i4+8] & TypeReg[2][9] & ~LockNode[2] & ~LockNode[3] & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[4][i4+8] & TypeReg[4][9] & ~LockNode[4] & ~LockNode[5] &
									(~TypeReg[2][9] | LockNode[2] | LockNode[3]) & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[6][i4+8] & TypeReg[6][9] & ~LockNode[6] & ~LockNode[7] &
									(~TypeReg[4][9] | LockNode[4] | LockNode[5]) & (~TypeReg[2][9] | LockNode[2] |LockNode[3]) &
									(~TypeReg[0][9] | LockNode[0] | LockNode[1]));
			// DATASEL field
			FMULACCBusWire[i4+16]=(InstReg[1][i4+12] & TypeReg[0][9] & ~LockNode[0] & ~LockNode[1])|
								(InstReg[3][i4+12] & TypeReg[2][9] & ~LockNode[2] & ~LockNode[3] & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[5][i4+12] & TypeReg[4][9] & ~LockNode[4] & ~LockNode[5] &
									(~TypeReg[2][9] | LockNode[2] | LockNode[3]) & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
								(InstReg[7][i4+12] & TypeReg[6][9] & ~LockNode[6] & ~LockNode[7] &
									(~TypeReg[4][9] | LockNode[4] | LockNode[5]) & (~TypeReg[2][9] | LockNode[2] |LockNode[3]) &
									(~TypeReg[0][9] | LockNode[0] | LockNode[1]));
			end
		end
	// CMD Field
	FMULACCBusWire[19]=(FMULLACCCMDWire[0] & TypeReg[0][9] & ~LockNode[0] & ~LockNode[1])|
						(FMULLACCCMDWire[1] & TypeReg[2][9] & ~LockNode[2] & ~LockNode[3] & (~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
						(FMULLACCCMDWire[2] & TypeReg[4][9] & ~LockNode[4] & ~LockNode[5] & (~TypeReg[2][9] | LockNode[2] | LockNode[3]) &
											(~TypeReg[0][9] | LockNode[0] | LockNode[1]))|
						(FMULLACCCMDWire[3] & TypeReg[6][9] & ~LockNode[6] & ~LockNode[7] & (~TypeReg[4][9] | LockNode[4] | LockNode[5]) &
											(~TypeReg[2][9] | LockNode[2] |LockNode[3]) & (~TypeReg[0][9] | LockNode[0] | LockNode[1]));
	
	//
	// Instruction patch for FDIV unit
	// 
	FDIVBusWire[0]=(TypeReg[0][2] & ~LockNode[0])|(TypeReg[1][2] & ~LockNode[1])|(TypeReg[2][2] & ~LockNode[2])|
					(TypeReg[3][2] & ~LockNode[3])|(TypeReg[4][2] & ~LockNode[4])|(TypeReg[5][2] & ~LockNode[5])|
					(TypeReg[6][2] & ~LockNode[6])|(TypeReg[7][2] & ~LockNode[7]);
	for (i=0; i<8; i=i+1) FDIVCMDWire[i]=(InstReg[i][7:0]==8'h7C);
	FDIVBusWire[13]=(FDIVCMDWire[0] & TypeReg[0][2] & ~LockNode[0])|
					(FDIVCMDWire[1] & TypeReg[1][2] & ~LockNode[1] & (~TypeReg[0][2] | LockNode[0]))|
					(FDIVCMDWire[2] & TypeReg[2][2] & ~LockNode[2] & (~TypeReg[1][2] | LockNode[1]) &
						(~TypeReg[0][2] | LockNode[0]))|
					(FDIVCMDWire[3] & TypeReg[3][2] & ~LockNode[3] & (~TypeReg[2][2] | LockNode[2]) &
						(~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(FDIVCMDWire[4] & TypeReg[4][2] & ~LockNode[4] & (~TypeReg[3][2] | LockNode[3]) &
						(~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) & 
						(~TypeReg[0][2] | LockNode[0]))|
					(FDIVCMDWire[5] & TypeReg[5][2] & ~LockNode[5] & (~TypeReg[4][2] | LockNode[4]) &
						(~TypeReg[3][2] | LockNode[3]) & (~TypeReg[2][2] | LockNode[2]) &
						(~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(FDIVCMDWire[6] & TypeReg[6][2] & ~LockNode[6] & (~TypeReg[5][2] | LockNode[5]) &
						(~TypeReg[4][2] | LockNode[4]) & (~TypeReg[3][2] | LockNode[3]) &
						(~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) &
						(~TypeReg[0][2] | LockNode[0]))|
					(FDIVCMDWire[7] & TypeReg[7][2] & ~LockNode[7] & (~TypeReg[6][2] | LockNode[6]) &
						(~TypeReg[5][2] | LockNode[5]) & (~TypeReg[4][2] | LockNode[4]) &
						(~TypeReg[3][2] | LockNode[3]) & (~TypeReg[2][2] | LockNode[2]) &
						(~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]));
	for (i=0; i<12; i=i+1)
		FDIVBusWire[i+1]=(InstReg[0][i+4] & TypeReg[0][2] & ~LockNode[0])|
					(InstReg[1][i+4] & TypeReg[1][2] & ~LockNode[1] & (~TypeReg[0][2] | LockNode[0]))|
					(InstReg[2][i+4] & TypeReg[2][2] & ~LockNode[2] & (~TypeReg[1][2] | LockNode[1]) &
						(~TypeReg[0][2] | LockNode[0]))|
					(InstReg[3][i+4] & TypeReg[3][2] & ~LockNode[3] & (~TypeReg[2][2] | LockNode[2]) &
						(~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(InstReg[4][i+4] & TypeReg[4][2] & ~LockNode[4] & (~TypeReg[3][2] | LockNode[3]) &
						(~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) & 
						(~TypeReg[0][2] | LockNode[0]))|
					(InstReg[5][i+4] & TypeReg[5][2] & ~LockNode[5] & (~TypeReg[4][2] | LockNode[4]) &
						(~TypeReg[3][2] | LockNode[3]) & (~TypeReg[2][2] | LockNode[2]) &
						(~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(InstReg[6][i+4] & TypeReg[6][2] & ~LockNode[6] & (~TypeReg[5][2] | LockNode[5]) &
						(~TypeReg[4][2] | LockNode[4]) & (~TypeReg[3][2] | LockNode[3]) &
						(~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) &
						(~TypeReg[0][2] | LockNode[0]))|
					(InstReg[7][i+4] & TypeReg[7][2] & ~LockNode[7] & (~TypeReg[6][2] | LockNode[6]) &
						(~TypeReg[5][2] | LockNode[5]) & (~TypeReg[4][2] | LockNode[4]) &
						(~TypeReg[3][2] | LockNode[3]) & (~TypeReg[2][2] | LockNode[2]) &
						(~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]));

	//
	// Instruction patch for ALU
	// 0 - ACT
	// [2:1] - CMD
	// [14:3] - DST, SRC1, SRC2 or Parameter 1, Parameter 2, SRC3, SRC4
	// 15 - operand selection for FIELDCOPY instruction 1 - from instruction register
	// [23:16] - DST and SRC2 for FIELDCOPY and BITCOPY
	// 24 - EXT flag
	for (i=0; i<4; i=i+1)
		begin
		ALUCMDWire[i<<1][0]=InstReg[i<<1][0];
		ALUCMDWire[i<<1][1]=InstReg[i<<1][3] | ~InstReg[i<<1][2];
		ALUCMDWire[1+(i<<1)][0]=ExtReg[i] ? InstReg[1+(i<<1)][1] : InstReg[1+(i<<1)][0];
		ALUCMDWire[1+(i<<1)][1]=(InstReg[1+(i<<1)][3] | ~InstReg[1+(i<<1)][2]) & ~ExtReg[i];
		end
	ALUBusWire[0]=(TypeReg[0][3] & ~LockNode[0])|(TypeReg[1][3] & ~LockNode[1] & (~LockNode[0] | ~ExtReg[0]))|
					(TypeReg[2][3] & ~LockNode[2])|(TypeReg[3][3] & ~LockNode[3] & (~LockNode[2] | ~ExtReg[1]))|
					(TypeReg[4][3] & ~LockNode[4])|(TypeReg[5][3] & ~LockNode[5] & (~LockNode[4] | ~ExtReg[2]))|
					(TypeReg[6][3] & ~LockNode[6])|(TypeReg[7][3] & ~LockNode[7] & (~LockNode[6] | ~ExtReg[3]));

	for (i=0; i<2; i=i+1)
		ALUBusWire[i+1]=(ALUCMDWire[0][i] & TypeReg[0][3] & ~LockNode[0])|
					(ALUCMDWire[1][i] & TypeReg[1][3] & ~LockNode[1] & (~LockNode[0] | ~ExtReg[0]) & (~TypeReg[0][3] | LockNode[0]))|
					(ALUCMDWire[2][i] & TypeReg[2][3] & ~LockNode[2] & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) &
						(~TypeReg[0][3] | LockNode[0]))|
					(ALUCMDWire[3][i] & TypeReg[3][3] & ~LockNode[3] & (~LockNode[2] | ~ExtReg[1]) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]))|
					(ALUCMDWire[4][i] & TypeReg[4][3] & ~LockNode[4] & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) &
						(~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) &
						(~TypeReg[0][3] | LockNode[0]))|
					(ALUCMDWire[5][i] & TypeReg[5][3] & ~LockNode[5] & (~LockNode[4] | ~ExtReg[2]) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]))|
					(ALUCMDWire[6][i] & TypeReg[6][3] & ~LockNode[6] & (~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) &
						(~TypeReg[4][3] | LockNode[4]) & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) &
						(~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & 
						(~TypeReg[0][3] | LockNode[0]))|
					(ALUCMDWire[7][i] & TypeReg[7][3] & ~LockNode[7] & (~LockNode[6] | ~ExtReg[3]) & (~TypeReg[6][3] | LockNode[6]) &
						(~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) & 
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]));
	for (i=0; i<12; i=i+1)
		ALUBusWire[i+3]=(InstReg[0][i+4] & TypeReg[0][3] & ~LockNode[0])|
					(InstReg[1][i+4] & TypeReg[1][3] & ~LockNode[1] & (~LockNode[0] | ~ExtReg[0]) & (~TypeReg[0][3] | LockNode[0]))|
					(InstReg[2][i+4] & TypeReg[2][3] & ~LockNode[2] & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) &
						(~TypeReg[0][3] | LockNode[0]))|
					(InstReg[3][i+4] & TypeReg[3][3] & ~LockNode[3] & (~LockNode[2] | ~ExtReg[1]) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]))|
					(InstReg[4][i+4] & TypeReg[4][3] & ~LockNode[4] & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) &
						(~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) &
						(~TypeReg[0][3] | LockNode[0]))|
					(InstReg[5][i+4] & TypeReg[5][3] & ~LockNode[5] & (~LockNode[4] | ~ExtReg[2]) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]))|
					(InstReg[6][i+4] & TypeReg[6][3] & ~LockNode[6] & (~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) &
						(~TypeReg[4][3] | LockNode[4]) & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) &
						(~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & 
						(~TypeReg[0][3] | LockNode[0]))|
					(InstReg[7][i+4] & TypeReg[7][3] & ~LockNode[7] & (~LockNode[6] | ~ExtReg[3]) & (~TypeReg[6][3] | LockNode[6]) &
						(~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) & 
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]));
	ALUBusWire[15]=(~InstReg[1][1] & ~InstReg[1][0] & TypeReg[1][3] & ExtReg[0] & ~LockNode[1] & ~LockNode[0] & (~TypeReg[0][3] | LockNode[0])) |
					(~InstReg[3][1] & ~InstReg[3][0] & TypeReg[3][3] & ExtReg[1] & ~LockNode[3] & ~LockNode[2] & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) |
					(~InstReg[5][1] & ~InstReg[5][0] & TypeReg[5][3] & ExtReg[2] & ~LockNode[5] & ~LockNode[4] & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) |
					(~InstReg[7][1] & ~InstReg[7][0] & TypeReg[7][3] & ExtReg[3] & ~LockNode[7] & ~LockNode[6] & (~TypeReg[6][3] | LockNode[6]) &
						(~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]));
	for (i=0; i<8; i=i+1)
		ALUBusWire[i+16]=(InstReg[0][i+8] & TypeReg[1][3] & ExtReg[0] & ~LockNode[1] & ~LockNode[0] & (~TypeReg[0][3] | LockNode[0])) |
					(InstReg[2][i+8] & TypeReg[3][3] & ExtReg[1] & ~LockNode[3] & ~LockNode[2] & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) |
					(InstReg[4][i+8] & TypeReg[5][3] & ExtReg[2] & ~LockNode[5] & ~LockNode[4] & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) |
					(InstReg[6][i+8] & TypeReg[7][3] & ExtReg[3] & ~LockNode[7] & ~LockNode[6] & (~TypeReg[6][3] | LockNode[6]) &
						(~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]));
	ALUBusWire[24]=(ExtReg[0] & TypeReg[1][3] & ~LockNode[1] & ~LockNode[0]) |
					(ExtReg[1] & TypeReg[3][3] & ~LockNode[3] & ~LockNode[2] & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) &
						(~TypeReg[0][3] | LockNode[0])) |
					(ExtReg[2] & TypeReg[5][3] & ~LockNode[5] & ~LockNode[4] & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) &
						(~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) |
					(ExtReg[3] & TypeReg[7][3] & ~LockNode[7] & ~LockNode[6] & (~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) &
						(~TypeReg[4][3] | LockNode[4]) & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) &
						(~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) &
						(~TypeReg[0][3] | LockNode[0]));

	//
	// Instruction patch for shifter
	//
	for (i=0; i<8; i=i+1) 
		begin
		SHTCMDWire[i][0]=InstReg[i][4];
		SHTCMDWire[i][1]=InstReg[i][5];
		SHTCMDWire[i][2]=InstReg[i][6];
		SHTCMDWire[i][3]=InstReg[i][7];
		end
	ShiftBusWire[0]=(TypeReg[0][4] & ~LockNode[0])|(TypeReg[1][4] & ~LockNode[1])|
					(TypeReg[2][4] & ~LockNode[2])|(TypeReg[3][4] & ~LockNode[3])|
					(TypeReg[4][4] & ~LockNode[4])|(TypeReg[5][4] & ~LockNode[5])|
					(TypeReg[6][4] & ~LockNode[6])|(TypeReg[7][4] & ~LockNode[7]);
	for (i=0; i<4; i=i+1)
		ShiftBusWire[i+1]=(SHTCMDWire[0][i] & TypeReg[0][4] & ~LockNode[0])|
					(SHTCMDWire[1][i] & TypeReg[1][4] & ~LockNode[1] & (~TypeReg[0][4] | LockNode[0]))|
					(SHTCMDWire[2][i] & TypeReg[2][4] & ~LockNode[2] & (~TypeReg[1][4] | LockNode[1]) &
							(~TypeReg[0][4] | LockNode[0]))|
					(SHTCMDWire[3][i] & TypeReg[3][4] & ~LockNode[3] & (~TypeReg[2][4] | LockNode[2]) &
							(~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(SHTCMDWire[4][i] & TypeReg[4][4] & ~LockNode[4] & (~TypeReg[3][4] | LockNode[3]) &
							(~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) &
							(~TypeReg[0][4] | LockNode[0]))|
					(SHTCMDWire[5][i] & TypeReg[5][4] & ~LockNode[5] & (~TypeReg[4][4] | LockNode[4]) &
							(~TypeReg[3][4] | LockNode[3]) & (~TypeReg[2][4] | LockNode[2]) &
							(~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(SHTCMDWire[6][i] & TypeReg[6][4] & ~LockNode[6] & (~TypeReg[5][4] | LockNode[5]) &
							(~TypeReg[4][4] | LockNode[4]) & (~TypeReg[3][4] | LockNode[3]) &
							(~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) &
							(~TypeReg[0][4] | LockNode[0]))|
					(SHTCMDWire[7][i] & TypeReg[7][4] & ~LockNode[7] & (~TypeReg[6][4] | LockNode[6]) &
							(~TypeReg[5][4] | LockNode[5]) & (~TypeReg[4][4] | LockNode[4]) &
							(~TypeReg[3][4] | LockNode[3]) & (~TypeReg[2][4] | LockNode[2]) & 
							(~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]));
	for (i=0; i<8; i=i+1)
		ShiftBusWire[i+5]=(InstReg[0][i+8] & TypeReg[0][4] & ~LockNode[0])|
					(InstReg[1][i+8] & TypeReg[1][4] & ~LockNode[1] & (~TypeReg[0][4] | LockNode[0]))|
					(InstReg[2][i+8] & TypeReg[2][4] & ~LockNode[2] & (~TypeReg[1][4] | LockNode[1]) &
							(~TypeReg[0][4] | LockNode[0]))|
					(InstReg[3][i+8] & TypeReg[3][4] & ~LockNode[3] & (~TypeReg[2][4] | LockNode[2]) &
							(~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(InstReg[4][i+8] & TypeReg[4][4] & ~LockNode[4] & (~TypeReg[3][4] | LockNode[3]) &
							(~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) &
							(~TypeReg[0][4] | LockNode[0]))|
					(InstReg[5][i+8] & TypeReg[5][4] & ~LockNode[5] & (~TypeReg[4][4] | LockNode[4]) &
							(~TypeReg[3][4] | LockNode[3]) & (~TypeReg[2][4] | LockNode[2]) &
							(~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(InstReg[6][i+8] & TypeReg[6][4] & ~LockNode[6] & (~TypeReg[5][4] | LockNode[5]) &
							(~TypeReg[4][4] | LockNode[4]) & (~TypeReg[3][4] | LockNode[3]) &
							(~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) &
							(~TypeReg[0][4] | LockNode[0]))|
					(InstReg[7][i+8] & TypeReg[7][4] & ~LockNode[7] & (~TypeReg[6][4] | LockNode[6]) &
							(~TypeReg[5][4] | LockNode[5]) & (~TypeReg[4][4] | LockNode[4]) &
							(~TypeReg[3][4] | LockNode[3]) & (~TypeReg[2][4] | LockNode[2]) &
							(~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]));
	
	//
	// Instruction patch for miscellaneous operations
	//
	/*
	0010 1111 0000	BSWAP	010
	0011 1111 0000	NEG		011
	0100 1111 0000	DAA		000
	0101 1111 0000	DAS		001
	xxxx 1100 0000	POS		110
	xxxx 0001 1100	CFZ		100
	xxxx 0011 1100	CFN		101
	xxxx xxxx 0001	LOOP	111
	*/
	for (i=0; i<8; i=i+1)
		begin
		MiscCMDWire[i][0]=(InstReg[i][8] & InstReg[i][7] & InstReg[i][6] & InstReg[i][5] & InstReg[i][4]) |
							(~InstReg[i][7] & ~InstReg[i][6] & InstReg[i][5] & InstReg[i][4])|
							(~InstReg[i][3] & ~InstReg[i][2] & ~InstReg[i][1] & InstReg[i][0]);
		MiscCMDWire[i][1]=(InstReg[i][9] & ~InstReg[i][3])|(InstReg[i][7] & InstReg[i][6] & ~InstReg[i][5] & ~InstReg[i][4])|
							(~InstReg[i][3] & ~InstReg[i][2] & ~InstReg[i][1] & InstReg[i][0]);
		MiscCMDWire[i][2]=InstReg[i][3]|(InstReg[i][7] & InstReg[i][6] & ~InstReg[i][5] & ~InstReg[i][4])|
							(~InstReg[i][3] & ~InstReg[i][2] & ~InstReg[i][1] & InstReg[i][0]);
		end
	MiscBusWire[0]=(TypeReg[0][5] & ~LockNode[0])|(TypeReg[1][5] & ~LockNode[1])|
					(TypeReg[2][5] & ~LockNode[2])|(TypeReg[3][5] & ~LockNode[3])|
					(TypeReg[4][5] & ~LockNode[4])|(TypeReg[5][5] & ~LockNode[5])|
					(TypeReg[6][5] & ~LockNode[6])|(TypeReg[7][5] & ~LockNode[7]);
	for (i=0; i<3; i=i+1)
		MiscBusWire[i+1]=(MiscCMDWire[0][i] & TypeReg[0][5] & ~LockNode[0])|
					(MiscCMDWire[1][i] & TypeReg[1][5] & ~LockNode[1] & (~TypeReg[0][5] | LockNode[0]))|
					(MiscCMDWire[2][i] & TypeReg[2][5] & ~LockNode[2] & (~TypeReg[1][5] | LockNode[1]) &
							(~TypeReg[0][5] | LockNode[0]))|
					(MiscCMDWire[3][i] & TypeReg[3][5] & ~LockNode[3] & (~TypeReg[2][5] | LockNode[2]) &
							(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(MiscCMDWire[4][i] & TypeReg[4][5] & ~LockNode[4] & (~TypeReg[3][5] | LockNode[3]) &
							(~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) & 
							(~TypeReg[0][5] | LockNode[0]))|
					(MiscCMDWire[5][i] & TypeReg[5][5] & ~LockNode[5] & (~TypeReg[4][5] | LockNode[4]) &
							(~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) &
							(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(MiscCMDWire[6][i] & TypeReg[6][5] & ~LockNode[6] & (~TypeReg[5][5] | LockNode[5]) &
							(~TypeReg[4][5] | LockNode[4]) & (~TypeReg[3][5] | LockNode[3]) & 
							(~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) &
							(~TypeReg[0][5] | LockNode[0]))|
					(MiscCMDWire[7][i] & TypeReg[7][5] & ~LockNode[7] & (~TypeReg[6][5] | LockNode[6]) &
							(~TypeReg[5][5] | LockNode[5]) & (~TypeReg[4][5] | LockNode[4]) &
							(~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) & 
							(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]));
	for (i=0; i<8; i=i+1)
		MiscBusWire[i+4]=(InstReg[0][i+8] & TypeReg[0][5] & ~LockNode[0])|
					(InstReg[1][i+8] & TypeReg[1][5] & ~LockNode[1] & (~TypeReg[0][5] | LockNode[0]))|
					(InstReg[2][i+8] & TypeReg[2][5] & ~LockNode[2] & (~TypeReg[1][5] | LockNode[1]) &
							(~TypeReg[0][5] | LockNode[0]))|
					(InstReg[3][i+8] & TypeReg[3][5] & ~LockNode[3] & (~TypeReg[2][5] | LockNode[2]) &
							(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(InstReg[4][i+8] & TypeReg[4][5] & ~LockNode[4] & (~TypeReg[3][5] | LockNode[3]) &
							(~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) &
							(~TypeReg[0][5] | LockNode[0]))|
					(InstReg[5][i+8] & TypeReg[5][5] & ~LockNode[5] & (~TypeReg[4][5] | LockNode[4]) &
							(~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) &
							(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(InstReg[6][i+8] & TypeReg[6][5] & ~LockNode[6] & (~TypeReg[5][5] | LockNode[5]) &
							(~TypeReg[4][5] | LockNode[4]) & (~TypeReg[3][5] | LockNode[3]) &
							(~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) &
							(~TypeReg[0][5] | LockNode[0]))|
					(InstReg[7][i+8] & TypeReg[7][5] & ~LockNode[7] & (~TypeReg[6][5] | LockNode[6]) &
							(~TypeReg[5][5] | LockNode[5]) & (~TypeReg[4][5] | LockNode[4]) &
							(~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) &
							(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]));
	//
	// LOOP instruction patch
	//
	for (i=0; i<16; i=i+1)
		LoopBusWire[i]=(InstReg[0][i] & TypeReg[0][5] & ~LockNode[0])|
						(InstReg[1][i] & TypeReg[1][5] & ~LockNode[1] & (~TypeReg[0][5] | LockNode[0]))|
						(InstReg[2][i] & TypeReg[2][5] & ~LockNode[2] & (~TypeReg[1][5] | LockNode[1]) &
								(~TypeReg[0][5] | LockNode[0]))|
						(InstReg[3][i] & TypeReg[3][5] & ~LockNode[3] & (~TypeReg[2][5] | LockNode[2]) &
								(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
						(InstReg[4][i] & TypeReg[4][5] & ~LockNode[4] & (~TypeReg[3][5] | LockNode[3]) &
								(~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) &
								(~TypeReg[0][5] | LockNode[0]))|
						(InstReg[5][i] & TypeReg[5][5] & ~LockNode[5] & (~TypeReg[4][5] | LockNode[4]) &
								(~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) &
								(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
						(InstReg[6][i] & TypeReg[6][5] & ~LockNode[6] & (~TypeReg[5][5] | LockNode[5]) &
								(~TypeReg[4][5] | LockNode[4]) & (~TypeReg[3][5] | LockNode[3]) &
								(~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) &
								(~TypeReg[0][5] | LockNode[0]))|
						(InstReg[7][i] & TypeReg[7][5] & ~LockNode[7] & (~TypeReg[6][5] | LockNode[6]) &
								(~TypeReg[5][5] | LockNode[5]) & (~TypeReg[4][5] | LockNode[4]) &
								(~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) &
								(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]));

	
	//
	// Instruction patch for register movement operations
	//
	/*
	0100 0000	COPY 			00
	1110 0000	SETSIZE/AMODE 	01
	     1111	LI	 			11
	*/
	for (i=0; i<8; i=i+1)
		begin
		MovCMDWire[i][0]=InstReg[i][5] | InstReg[i][3];
		MovCMDWire[i][1]=InstReg[i][3];
		end
	MovBusWire[0]=(TypeReg[0][6] & ~LockNode[0])|(TypeReg[1][6] & ~LockNode[1])|
					(TypeReg[2][6] & ~LockNode[2])|(TypeReg[3][6] & ~LockNode[3])|
					(TypeReg[4][6] & ~LockNode[4])|(TypeReg[5][6] & ~LockNode[5])|
					(TypeReg[6][6] & ~LockNode[6])|(TypeReg[7][6] & ~LockNode[7]);
	for (i=0; i<2; i=i+1)
		MovBusWire[i+1]=(MovCMDWire[0][i] &  TypeReg[0][6] & ~LockNode[0])|
					(MovCMDWire[1][i] & TypeReg[1][6] & ~LockNode[1] & (~TypeReg[0][6] | LockNode[0]))|
					(MovCMDWire[2][i] & TypeReg[2][6] & ~LockNode[2] & (~TypeReg[1][6] | LockNode[1]) &
							(~TypeReg[0][6] | LockNode[0]))|
					(MovCMDWire[3][i] & TypeReg[3][6] & ~LockNode[3] & (~TypeReg[2][6] | LockNode[2]) &
							(~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(MovCMDWire[4][i] & TypeReg[4][6] & ~LockNode[4] & (~TypeReg[3][6] | LockNode[3]) &
							(~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) &
							(~TypeReg[0][6] | LockNode[0]))|
					(MovCMDWire[5][i] & TypeReg[5][6] & ~LockNode[5] & (~TypeReg[4][6] | LockNode[4]) &
							(~TypeReg[3][6] | LockNode[3]) & (~TypeReg[2][6] | LockNode[2]) &
							(~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(MovCMDWire[6][i] & TypeReg[6][6] & ~LockNode[6] & (~TypeReg[5][6] | LockNode[5]) &
							(~TypeReg[4][6] | LockNode[4]) & (~TypeReg[3][6] | LockNode[3]) &
							(~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) & 
							(~TypeReg[0][6] | LockNode[0]))|
					(MovCMDWire[7][i] & TypeReg[7][6] & ~LockNode[7] & (~TypeReg[6][6] | LockNode[6]) &
							(~TypeReg[5][6] | LockNode[5]) & (~TypeReg[4][6] | LockNode[4]) &
							(~TypeReg[3][6] | LockNode[3]) & (~TypeReg[2][6] | LockNode[2]) &
							(~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]));
	for (i=0; i<12; i=i+1)
		MovBusWire[i+3]=(InstReg[0][i+4] & TypeReg[0][6] & ~LockNode[0])|
					(InstReg[1][i+4] & TypeReg[1][6] & ~LockNode[1] & (~TypeReg[0][6] | LockNode[0]))|
					(InstReg[2][i+4] & TypeReg[2][6] & ~LockNode[2] & (~TypeReg[1][6] | LockNode[1]) &
							(~TypeReg[0][6] | LockNode[0]))|
					(InstReg[3][i+4] & TypeReg[3][6] & ~LockNode[3] & (~TypeReg[2][6] | LockNode[2]) &
							(~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(InstReg[4][i+4] & TypeReg[4][6] & ~LockNode[4] & (~TypeReg[3][6] | LockNode[3]) &
							(~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) &
							(~TypeReg[0][6] | LockNode[0]))|
					(InstReg[5][i+4] & TypeReg[5][6] & ~LockNode[5] & (~TypeReg[4][6] | LockNode[4]) &
							(~TypeReg[3][6] | LockNode[3]) & (~TypeReg[2][6] | LockNode[2]) &
							(~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(InstReg[6][i+4] & TypeReg[6][6] & ~LockNode[6] & (~TypeReg[5][6] | LockNode[5]) &
							(~TypeReg[4][6] | LockNode[4]) & (~TypeReg[3][6] | LockNode[3]) &
							(~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) &
							(~TypeReg[0][6] | LockNode[0]))|
					(InstReg[7][i+4] & TypeReg[7][6] & ~LockNode[7] & (~TypeReg[6][6] | LockNode[6]) &
							(~TypeReg[5][6] | LockNode[5]) & (~TypeReg[4][6] | LockNode[4]) &
							(~TypeReg[3][6] | LockNode[3]) & (~TypeReg[2][6] | LockNode[2]) &
							(~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]));
	
	//
	// LD/ST/POP/PUSH instructions
	//
	/*
		00101 LDB  	1000
		01101 LDW	1001
		10101 LDD   1010
		11101 LDQ   1011
		10100 LDO   1100
		00100 ST    0000
	011110000 PUSH  0111
	111110000 POP   1111
	 01100000 LAR	0110
	 01110000 SAR	0101
	*/
	for (i=0; i<8; i=i+1)
		begin
		MemCMDWire[i][0]=(InstReg[i][3] & InstReg[i][2])|(InstReg[i][4] & ~InstReg[i][2]);
		MemCMDWire[i][1]=(InstReg[i][4] & InstReg[i][0] & InstReg[i][2])| (~InstReg[i][2] & (InstReg[i][7] | ~InstReg[i][4]));
		MemCMDWire[i][2]=~InstReg[i][2] | (InstReg[i][4] & ~InstReg[i][0]);
		MemCMDWire[i][3]=InstReg[i][0] | (InstReg[i][4] & InstReg[i][2]) | (~InstReg[i][2] & InstReg[i][7] & InstReg[i][8]);
		end
	MemBusWire[0]=(TypeReg[0][7] & ~LockNode[0])|
					(TypeReg[1][7] & ~LockNode[1] & ~TypeReg[0][7])|
					(TypeReg[2][7] & ~LockNode[2] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[3][7] & ~LockNode[3] & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[4][7] & ~LockNode[4] & ~TypeReg[3][7] & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[5][7] & ~LockNode[5] & ~TypeReg[4][7] & ~TypeReg[3][7] & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[6][7] & ~LockNode[6] & ~TypeReg[5][7] & ~TypeReg[4][7] & ~TypeReg[3][7] & ~TypeReg[2][7] & ~TypeReg[1][7] &
									~TypeReg[0][7])|
					(TypeReg[7][7] & ~LockNode[7] & ~TypeReg[6][7] & ~TypeReg[5][7] & ~TypeReg[4][7] & ~TypeReg[3][7] & ~TypeReg[2][7] & 
									~TypeReg[1][7] & ~TypeReg[0][7]);
	for (i=0; i<4; i=i+1)
		MemBusWire[1+i]=(MemCMDWire[0][i] & TypeReg[0][7] & ~LockNode[0])|
					(MemCMDWire[1][i] & TypeReg[1][7] & ~LockNode[1] & (~TypeReg[0][7] | LockNode[0]))|
					(MemCMDWire[2][i] & TypeReg[2][7] & ~LockNode[2] & (~TypeReg[1][7] | LockNode[1]) &
							(~TypeReg[0][7] | LockNode[0]))|
					(MemCMDWire[3][i] & TypeReg[3][7] & ~LockNode[3] & (~TypeReg[2][7] | LockNode[2]) &
							(~TypeReg[1][7] | LockNode[1]) & (~TypeReg[0][7] | LockNode[0]))|
					(MemCMDWire[4][i] & TypeReg[4][7] & ~LockNode[4] & (~TypeReg[3][7] | LockNode[3]) &
							(~TypeReg[2][7] | LockNode[2]) & (~TypeReg[1][7] | LockNode[1]) &
							(~TypeReg[0][7] | LockNode[0]))|
					(MemCMDWire[5][i] & TypeReg[5][7] & ~LockNode[5] & (~TypeReg[4][7] | LockNode[4]) &
							(~TypeReg[3][7] | LockNode[3]) & (~TypeReg[2][7] | LockNode[2]) & 
							(~TypeReg[1][7] | LockNode[1]) & (~TypeReg[0][7] | LockNode[0]))|
					(MemCMDWire[6][i] & TypeReg[6][7] & ~LockNode[6] & (~TypeReg[5][7] | LockNode[5]) &
							(~TypeReg[4][7] | LockNode[4]) & (~TypeReg[3][7] | LockNode[3]) &
							(~TypeReg[2][7] | LockNode[2]) & (~TypeReg[1][7] | LockNode[1]) &
							(~TypeReg[0][7] | LockNode[0]))|
					(MemCMDWire[7][i] & TypeReg[7][7] & ~LockNode[7] & (~TypeReg[6][7] | LockNode[6]) &
							(~TypeReg[5][7] | LockNode[5]) & (~TypeReg[4][7] | LockNode[4]) &
							(~TypeReg[3][7] | LockNode[3]) & (~TypeReg[2][7] | LockNode[2]) &
							(~TypeReg[1][7] | LockNode[1]) & (~TypeReg[0][7] | LockNode[0]));
	for (i=0; i<11; i=i+1)
		MemBusWire[i+5]=(InstReg[0][i+5] & TypeReg[0][7] & ~LockNode[0])|
					(InstReg[1][i+5] & TypeReg[1][7] & ~LockNode[1] & (~TypeReg[0][7] | LockNode[0]))|
					(InstReg[2][i+5] & TypeReg[2][7] & ~LockNode[2] & (~TypeReg[1][7] | LockNode[1]) &
							(~TypeReg[0][7] | LockNode[0]))|
					(InstReg[3][i+5] & TypeReg[3][7] & ~LockNode[3] & (~TypeReg[2][7] | LockNode[2]) &
							(~TypeReg[1][7] | LockNode[1]) & (~TypeReg[0][7] | LockNode[0]))|
					(InstReg[4][i+5] & TypeReg[4][7] & ~LockNode[4] & (~TypeReg[3][7] | LockNode[3]) &
							(~TypeReg[2][7] | LockNode[2]) & (~TypeReg[1][7] | LockNode[1]) &
							(~TypeReg[0][7] | LockNode[0]))|
					(InstReg[5][i+5] & TypeReg[5][7] & ~LockNode[5] & (~TypeReg[4][7] | LockNode[4]) &
							(~TypeReg[3][7] | LockNode[3]) & (~TypeReg[2][7] | LockNode[2]) &
							(~TypeReg[1][7] | LockNode[1]) & (~TypeReg[0][7] | LockNode[0]))|
					(InstReg[6][i+5] & TypeReg[6][7] & ~LockNode[6] & (~TypeReg[5][7] | LockNode[5]) &
							(~TypeReg[4][7] | LockNode[4]) & (~TypeReg[3][7] | LockNode[3]) &
							(~TypeReg[2][7] | LockNode[2]) & (~TypeReg[1][7] | LockNode[1]) &
							(~TypeReg[0][7] | LockNode[0]))|
					(InstReg[7][i+5] & TypeReg[7][7] & ~LockNode[7] & (~TypeReg[6][7] | LockNode[6]) &
							(~TypeReg[5][7] | LockNode[5]) & (~TypeReg[4][7] | LockNode[4]) &
							(~TypeReg[3][7] | LockNode[3]) & (~TypeReg[2][7] | LockNode[2]) &
							(~TypeReg[1][7] | LockNode[1]) & (~TypeReg[0][7] | LockNode[0]));

	//
	// Flow control instructions
	//
	CtrlBusWire[0]=(TypeReg[0][8] & ~LockNode[0])|
				(TypeReg[1][8] & ~LockNode[1] & ~TypeReg[0][8])|
				(TypeReg[2][8] & ~LockNode[2] & ~TypeReg[1][8] & ~TypeReg[0][8])|
				(TypeReg[3][8] & ~LockNode[3] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8])|
				(TypeReg[4][8] & ~LockNode[4] & ~TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8])|
				(TypeReg[5][8] & ~LockNode[5] & ~TypeReg[4][8] & ~TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8])|
				(TypeReg[6][8] & ~LockNode[6] & ~TypeReg[5][8] & ~TypeReg[4][8] & ~TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] &
												~TypeReg[0][8])|
				(TypeReg[7][8] & ~LockNode[7] & ~TypeReg[6][8] & ~TypeReg[5][8] & ~TypeReg[4][8] & ~TypeReg[3][8] & ~TypeReg[2][8] &
												~TypeReg[1][8] & ~TypeReg[0][8]);
	for (i=0; i<16; i=i+1)
		CtrlBusWire[i+1]=(InstReg[0][i] & TypeReg[0][8] & ~LockNode[0])|
					(InstReg[1][i] & TypeReg[1][8] & ~LockNode[1] & (~TypeReg[0][8] | LockNode[0]))|
					(InstReg[2][i] & TypeReg[2][8] & ~LockNode[2] & (~TypeReg[1][8] | LockNode[1]) &
							(~TypeReg[0][8] | LockNode[0]))|
					(InstReg[3][i] & TypeReg[3][8] & ~LockNode[3] & (~TypeReg[2][8] | LockNode[2]) &
							(~TypeReg[1][8] | LockNode[1]) & (~TypeReg[0][8] | LockNode[0]))|
					(InstReg[4][i] & TypeReg[4][8] & ~LockNode[4] & (~TypeReg[3][8] | LockNode[3]) &
							(~TypeReg[2][8] | LockNode[2]) & (~TypeReg[1][8] | LockNode[1]) &
							(~TypeReg[0][8] | LockNode[0]))|
					(InstReg[5][i] & TypeReg[5][8] & ~LockNode[5] & (~TypeReg[4][8] | LockNode[4]) &
							(~TypeReg[3][8] | LockNode[3]) & (~TypeReg[2][8] | LockNode[2]) &
							(~TypeReg[1][8] | LockNode[1]) & (~TypeReg[0][8] | LockNode[0]))|
					(InstReg[6][i] & TypeReg[6][8] & ~LockNode[6] & (~TypeReg[5][8] | LockNode[5]) &
							(~TypeReg[4][8] | LockNode[4]) & (~TypeReg[3][8] | LockNode[3]) &
							(~TypeReg[2][8] | LockNode[2]) & (~TypeReg[1][8] | LockNode[1]) & 
							(~TypeReg[0][8] | LockNode[0]))|
					(InstReg[7][i] & TypeReg[7][8] & ~LockNode[7] & (~TypeReg[6][8] | LockNode[6]) &
							(~TypeReg[5][8] | LockNode[5]) & (~TypeReg[4][8] | LockNode[4]) &
							(~TypeReg[3][8] | LockNode[3]) & (~TypeReg[2][8] | LockNode[2]) &
							(~TypeReg[1][8] | LockNode[1]) & (~TypeReg[0][8] | LockNode[0]));
	
	// VF reset nodes
	VFResetWire[0]=((TypeReg[0][0] | TypeReg[0][1] | (TypeReg[0][2] & FDIVRDY) | TypeReg[0][3] |
				TypeReg[0][4] | TypeReg[0][5] | TypeReg[0][6] | (TypeReg[0][7] & MEMRDY) | TypeReg[0][8] |
				(TypeReg[1][3] & ExtReg[0] & ~LockNode[1]) | (TypeReg[0][9] & ~LockNode[1] & FMULACCRDY)) & ~LockNode[0]) | TypeReg[0][10];

	VFResetWire[1]=(((TypeReg[1][0] & (~TypeReg[0][0] | LockNode[0]))|
					(TypeReg[1][1] & (~TypeReg[0][1] | LockNode[0]))|
					(TypeReg[1][2] & FDIVRDY & (~TypeReg[0][2] | LockNode[0]))|
					(TypeReg[1][3] & (~TypeReg[0][3] | LockNode[0]) & (~LockNode[0] | ~ExtReg[0]))|
					(TypeReg[1][4] & (~TypeReg[0][4] | LockNode[0]))|
					(TypeReg[1][5] & (~TypeReg[0][5] | LockNode[0]))|
					(TypeReg[1][6] & (~TypeReg[0][6] | LockNode[0]))|
					(TypeReg[1][7] & MEMRDY & ~TypeReg[0][7])|
					(TypeReg[1][8] & ~TypeReg[0][8]) |
					(TypeReg[0][9] & FMULACCRDY & ~LockNode[0])) & ~LockNode[1])| TypeReg[1][10];

	VFResetWire[2]=(((TypeReg[2][0] & (~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(TypeReg[2][1] & (~TypeReg[1][1] | LockNode[1]) & (~TypeReg[0][1] | LockNode[0]))|
					(TypeReg[2][2] & FDIVRDY & (~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(TypeReg[2][3] & (~TypeReg[1][3] | LockNode[1]) & (~TypeReg[0][3] | LockNode[0]))|
					(TypeReg[2][4] & (~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(TypeReg[2][5] & (~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(TypeReg[2][6] & (~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(TypeReg[2][7] & MEMRDY & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8]) |
					(TypeReg[2][9] & FMULACCRDY & ~LockNode[3] & (~TypeReg[0][9] | LockNode[0] | LockNode[1])) |
					((TypeReg[3][3] & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) & ExtReg[1] & ~LockNode[3])) & ~LockNode[2])| TypeReg[2][10];

	VFResetWire[3]=(((TypeReg[3][0] & (~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(TypeReg[3][1] & (~TypeReg[2][1] | LockNode[2]) & (~TypeReg[1][1] | LockNode[1]) & (~TypeReg[0][1] | LockNode[0]))|
					(TypeReg[3][2] & FDIVRDY & (~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(TypeReg[3][3] & (~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]) &
							(~LockNode[2] | ~ExtReg[1]))|
					(TypeReg[3][4] & (~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(TypeReg[3][5] & (~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(TypeReg[3][6] & (~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(TypeReg[3][7] & MEMRDY & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[2][9] & FMULACCRDY & (~TypeReg[0][9] | LockNode[0] | LockNode[1]) & ~LockNode[2])|
					(TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8])) & ~LockNode[3]) | TypeReg[3][10];

	VFResetWire[4]=(((TypeReg[4][0] & (~TypeReg[3][0] | LockNode[3]) & (~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) &
						(~TypeReg[0][0] | LockNode[0]))|
					(TypeReg[4][1] & (~TypeReg[3][1] | LockNode[3]) & (~TypeReg[2][1] | LockNode[2]) & (~TypeReg[1][1] | LockNode[1]) &
						(~TypeReg[0][1] | LockNode[0]))|
					(TypeReg[4][2] & FDIVRDY & (~TypeReg[3][2] | LockNode[3]) & (~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) & 
						(~TypeReg[0][2] | LockNode[0]))|
					(TypeReg[4][3] & (~TypeReg[3][3] | LockNode[3]) & (~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1]) &
						(~TypeReg[0][3] | LockNode[0]))|
					(TypeReg[4][4] & (~TypeReg[3][4] | LockNode[3]) & (~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) &
						(~TypeReg[0][4] | LockNode[0]))|
					(TypeReg[4][5] & (~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) &
						(~TypeReg[0][5] | LockNode[0]))|
					(TypeReg[4][6] & (~TypeReg[3][6] | LockNode[3]) & (~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) &
						(~TypeReg[0][6] | LockNode[0]))|
					(TypeReg[4][7] & MEMRDY & ~TypeReg[3][7] & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[4][8] & ~TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8]) |
					(TypeReg[4][9] & FMULACCRDY & ~LockNode[5] & (~TypeReg[2][9] | LockNode[2] | LockNode[3]) & (~TypeReg[0][9] | LockNode[1] | LockNode[0]))|
					((TypeReg[5][3] & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) & ExtReg[2] & ~LockNode[5])) & ~LockNode[4]) | TypeReg[4][10];

	VFResetWire[5]=(((TypeReg[5][0] & (~TypeReg[4][0] | LockNode[4]) & (~TypeReg[3][0] | LockNode[3]) & (~TypeReg[2][0] | LockNode[2]) &
						(~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(TypeReg[5][1] & (~TypeReg[4][1] | LockNode[4]) & (~TypeReg[3][1] | LockNode[3]) & (~TypeReg[2][1] | LockNode[2]) &
						(~TypeReg[1][1] | LockNode[1]) & (~TypeReg[0][1] | LockNode[0]))|
					(TypeReg[5][2] & FDIVRDY & (~TypeReg[4][2] | LockNode[4]) & (~TypeReg[3][2] | LockNode[3]) &
						(~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(TypeReg[5][3] & (~TypeReg[4][3] | LockNode[4]) & (~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0]) & (~LockNode[4] | ~ExtReg[2]))|
					(TypeReg[5][4] & (~TypeReg[4][4] | LockNode[4]) & (~TypeReg[3][4] | LockNode[3]) & (~TypeReg[2][4] | LockNode[2]) &
						(~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(TypeReg[5][5] & (~TypeReg[4][5] | LockNode[4]) & (~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) &
						(~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(TypeReg[5][6] & (~TypeReg[4][6] | LockNode[4]) & (~TypeReg[3][6] | LockNode[3]) & (~TypeReg[2][6] | LockNode[2]) &
						(~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(TypeReg[5][7] & MEMRDY & ~TypeReg[4][7] & ~TypeReg[3][7] & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[5][8] & ~TypeReg[4][8] & ~TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8])|
					(TypeReg[4][9] & ~LockNode[4] & FMULACCRDY & (~TypeReg[2][9] | LockNode[2] | LockNode[3]) & (~TypeReg[0][9] | LockNode[1] | LockNode[0]))) & ~LockNode[5])| TypeReg[5][10];

	VFResetWire[6]=(((TypeReg[6][0] & (~TypeReg[5][0] | LockNode[5]) & (~TypeReg[4][0] | LockNode[4]) & (~TypeReg[3][0] | LockNode[3]) &
						(~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) & (~TypeReg[0][0] | LockNode[0]))|
					(TypeReg[6][1] & (~TypeReg[5][1] | LockNode[5]) & (~TypeReg[4][1] | LockNode[4]) & (~TypeReg[3][1] | LockNode[3]) &
						(~TypeReg[2][1] | LockNode[2]) & (~TypeReg[1][1] | LockNode[1]) & (~TypeReg[0][1] | LockNode[0]))|
					(TypeReg[6][2] & FDIVRDY & (~TypeReg[5][2] | LockNode[5]) & (~TypeReg[4][2] | LockNode[4]) & (~TypeReg[3][2] | LockNode[3]) &
						(~TypeReg[2][2] | LockNode[2]) & (~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(TypeReg[6][3] & (~TypeReg[5][3] | LockNode[5]) & (~TypeReg[4][3] | LockNode[4]) & (~TypeReg[3][3] | LockNode[3]) &
						(~TypeReg[2][3] | LockNode[2]) & (~TypeReg[1][3] | LockNode[1]) & (~TypeReg[0][3] | LockNode[0]))|
					(TypeReg[6][4] & (~TypeReg[5][4] | LockNode[5]) & (~TypeReg[4][4] | LockNode[4]) & (~TypeReg[3][4] | LockNode[3]) &
						(~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) & (~TypeReg[0][4] | LockNode[0]))|
					(TypeReg[6][5] & (~TypeReg[5][5] | LockNode[5]) & (~TypeReg[4][5] | LockNode[4]) & (~TypeReg[3][5] | LockNode[3]) &
						(~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) & (~TypeReg[0][5] | LockNode[0]))|
					(TypeReg[6][6] & (~TypeReg[5][6] | LockNode[5]) & (~TypeReg[4][6] | LockNode[4]) & (~TypeReg[3][6] | LockNode[3]) &
						(~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) & (~TypeReg[0][6] | LockNode[0]))|
					(TypeReg[6][7] & MEMRDY & ~TypeReg[5][7] & ~TypeReg[4][7] & ~TypeReg[3][7] & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
					(TypeReg[6][8] & ~TypeReg[5][8] & ~TypeReg[4][8] & ~TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8]) |
					(TypeReg[6][9] & FMULACCRDY & ~LockNode[7] & (~TypeReg[4][9] | LockNode[5] | LockNode[4]) &
									(~TypeReg[2][9] | LockNode[3] | LockNode[2]) & (~TypeReg[0][9] | LockNode[1] | LockNode[0]))|
					((TypeReg[7][3] & (~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[0])) & (~TypeReg[0][3] | LockNode[0])) & ExtReg[3] & ~LockNode[7])) & ~LockNode[6])| TypeReg[6][10];

	VFResetWire[7]=(((TypeReg[7][0] & (~TypeReg[6][0] | LockNode[6]) & (~TypeReg[5][0] | LockNode[5]) & (~TypeReg[4][0] | LockNode[4]) &
						(~TypeReg[3][0] | LockNode[3]) & (~TypeReg[2][0] | LockNode[2]) & (~TypeReg[1][0] | LockNode[1]) &
						(~TypeReg[0][0] | LockNode[0]))|
					(TypeReg[7][1] & (~TypeReg[6][1] | LockNode[6]) & (~TypeReg[5][1] | LockNode[5]) & (~TypeReg[4][1] | LockNode[4]) &
						(~TypeReg[3][1] | LockNode[3]) & (~TypeReg[2][1] | LockNode[2]) & (~TypeReg[1][1] | LockNode[1]) &
						(~TypeReg[0][1] | LockNode[0]))|
					(TypeReg[7][2] & FDIVRDY & (~TypeReg[6][2] | LockNode[6]) & (~TypeReg[5][2] | LockNode[5]) & 
						(~TypeReg[4][2] | LockNode[4]) & (~TypeReg[3][2] | LockNode[3]) & (~TypeReg[2][2] | LockNode[2]) & 
						(~TypeReg[1][2] | LockNode[1]) & (~TypeReg[0][2] | LockNode[0]))|
					(TypeReg[7][3] & (~TypeReg[6][3] | LockNode[6]) & (~TypeReg[5][3] | LockNode[5] | (LockNode[4] & ExtReg[2])) & (~TypeReg[4][3] | LockNode[4]) &
						(~TypeReg[3][3] | LockNode[3] | (LockNode[2] & ExtReg[1])) & (~TypeReg[2][3] | LockNode[2]) &
						(~TypeReg[1][3] | LockNode[1] | (LockNode[0] & ExtReg[1])) & (~TypeReg[0][3] | LockNode[0]) & (~LockNode[6] | ~ExtReg[3]))|
					(TypeReg[7][4] & (~TypeReg[6][4] | LockNode[6]) & (~TypeReg[5][4] | LockNode[5]) & (~TypeReg[4][4] | LockNode[4]) &
						(~TypeReg[3][4] | LockNode[3]) & (~TypeReg[2][4] | LockNode[2]) & (~TypeReg[1][4] | LockNode[1]) &
						(~TypeReg[0][4] | LockNode[0]))|
					(TypeReg[7][5] & (~TypeReg[6][5] | LockNode[6]) & (~TypeReg[5][5] | LockNode[5]) & (~TypeReg[4][5] | LockNode[4]) &
						(~TypeReg[3][5] | LockNode[3]) & (~TypeReg[2][5] | LockNode[2]) & (~TypeReg[1][5] | LockNode[1]) &
						(~TypeReg[0][5] | LockNode[0]))|
					(TypeReg[7][6] & (~TypeReg[6][6] | LockNode[6]) & (~TypeReg[5][6] | LockNode[5]) & (~TypeReg[4][6] | LockNode[4]) &
						(~TypeReg[3][6] | LockNode[3]) & (~TypeReg[2][6] | LockNode[2]) & (~TypeReg[1][6] | LockNode[1]) &
						(~TypeReg[0][6] | LockNode[0]))|
			(TypeReg[7][7] & MEMRDY & ~TypeReg[6][7] & ~TypeReg[5][7] & ~TypeReg[4][7] & ~TypeReg[3][7] & ~TypeReg[2][7] & ~TypeReg[1][7] & ~TypeReg[0][7])|
			(TypeReg[7][8] & ~TypeReg[6][8] & ~TypeReg[5][8] & ~TypeReg[4][8] & ~TypeReg[3][8] & ~TypeReg[2][8] & ~TypeReg[1][8] & ~TypeReg[0][8])|
			(TypeReg[6][9] & ~LockNode[6] & FMULACCRDY & (~TypeReg[4][9] | LockNode[5] | LockNode[4]) & (~TypeReg[2][9] | LockNode[3] | LockNode[2]) &
					(~TypeReg[0][9] | LockNode[1] | LockNode[0]))) & 
					~LockNode[7]) | TypeReg[7][10];
	
	// invalidation bus
	for (i=0; i<16; i=i+1)
		InvdBus[i]=(GPRInvdR[0][i] | LockNode[0] | ~VFReg[0] | ~VFResetWire[0]) &
					(GPRInvdR[1][i] | LockNode[1] | ~VFReg[1] | ~VFResetWire[1]) &
					(GPRInvdR[2][i] | LockNode[2] | ~VFReg[2] | ~VFResetWire[2]) &
					(GPRInvdR[3][i] | LockNode[3] | ~VFReg[3] | ~VFResetWire[3]) &
					(GPRInvdR[4][i] | LockNode[4] | ~VFReg[4] | ~VFResetWire[4]) &
					(GPRInvdR[5][i] | LockNode[5] | ~VFReg[5] | ~VFResetWire[5]) &
					(GPRInvdR[6][i] | LockNode[6] | ~VFReg[6] | ~VFResetWire[6]) &
					(GPRInvdR[7][i] | LockNode[7] | ~VFReg[7] | ~VFResetWire[7]);

	// reloading instruction registers
	InstReloadWire=(~VFReg[3] | VFResetWire[3])  & (~VFReg[2] | VFResetWire[2]) & (~VFReg[1] | VFResetWire[1]) & (~VFReg[0] | VFResetWire[0]);
	HalfReloadWire=(~VFReg[7] | VFResetWire[7])  & (~VFReg[6] | VFResetWire[6]) & (~VFReg[5] | VFResetWire[5]) & (~VFReg[4] | VFResetWire[4]) & ~InstReloadWire;

	end


/*
====================================================================================================
					Synchronous part
====================================================================================================
*/
always @(negedge RESET or posedge CLK)
	begin
	if (!RESET) begin
					InstReg[0]<=0;
					InstReg[1]<=0;
					InstReg[2]<=0;
					InstReg[3]<=0;
					InstReg[4]<=0;
					InstReg[5]<=0;
					InstReg[6]<=0;
					InstReg[7]<=0;
					VFReg<=8'd0;
					TypeReg[0]<=0;
					TypeReg[1]<=0;
					TypeReg[2]<=0;
					TypeReg[3]<=0;
					TypeReg[4]<=0;
					TypeReg[5]<=0;
					TypeReg[6]<=0;
					TypeReg[7]<=0;
					GPRInvdR[0]<=16'hFFFF;
					GPRInvdR[1]<=16'hFFFF;
					GPRInvdR[2]<=16'hFFFF;
					GPRInvdR[3]<=16'hFFFF;
					GPRInvdR[4]<=16'hFFFF;
					GPRInvdR[5]<=16'hFFFF;
					GPRInvdR[6]<=16'hFFFF;
					GPRInvdR[7]<=16'hFFFF;
					OpReg[0]<=0;
					OpReg[1]<=0;
					OpReg[2]<=0;
					OpReg[3]<=0;
					OpReg[4]<=0;
					OpReg[5]<=0;
					OpReg[6]<=0;
					OpReg[7]<=0;
					end
	else begin
	// instruction register
	if (InstReloadWire)
		begin
		InstReg[0]<=InstReg[4];
		InstReg[1]<=InstReg[5];
		InstReg[2]<=InstReg[6];
		InstReg[3]<=InstReg[7];

		I1SRC1_I0DST<=I5SRC1_I4DST;
		I1SRC2_I0DST<=I5SRC2_I4DST;
		I1DST_I0SRC1<=I5DST_I4SRC1;
		I1DST_I0SRC2<=I5DST_I4SRC2;
		I1DST_I0DST<=I5DST_I4DST;

		I2SRC1_I1DST<=I6SRC1_I5DST;
		I2SRC2_I1DST<=I6SRC2_I5DST;
		I2DST_I1SRC1<=I6DST_I5SRC1;
		I2DST_I1SRC2<=I6DST_I5SRC2;
		I2DST_I1DST<=I6DST_I5DST;
		I2SRC1_I0DST<=I6SRC1_I4DST;
		I2SRC2_I0DST<=I6SRC2_I4DST;
		I2DST_I0SRC1<=I6DST_I4SRC1;
		I2DST_I0SRC2<=I6DST_I4SRC2;
		I2DST_I0DST<=I6DST_I4DST;

		I3SRC1_I2DST<=I7SRC1_I6DST;
		I3SRC2_I2DST<=I7SRC2_I6DST;
		I3DST_I2SRC1<=I7DST_I6SRC1;
		I3DST_I2SRC2<=I7DST_I6SRC2;
		I3DST_I2DST<=I7DST_I6DST;
		I3SRC1_I1DST<=I7SRC1_I5DST;
		I3SRC2_I1DST<=I7SRC2_I5DST;
		I3DST_I1SRC1<=I7DST_I5SRC1;
		I3DST_I1SRC2<=I7DST_I5SRC2;
		I3DST_I1DST<=I7DST_I5DST;
		I3SRC1_I0DST<=I7SRC1_I4DST;
		I3SRC2_I0DST<=I7SRC2_I4DST;
		I3DST_I0SRC1<=I7DST_I4SRC1;
		I3DST_I0SRC2<=I7DST_I4SRC2;
		I3DST_I0DST<=I7DST_I4DST;
		end

	if (InstReloadWire | HalfReloadWire)
		begin
		InstReg[4]<=IBUS[15:0];
		InstReg[5]<=IBUS[31:16];
		InstReg[6]<=IBUS[47:32];
		InstReg[7]<=IBUS[63:48];

		I4SRC1_I3DST<=HalfReloadWire ? (IBUS[7:4]==InstReg[3][15:12]) : (IBUS[7:4]==InstReg[7][15:12]);
		I4SRC2_I3DST<=HalfReloadWire ? (IBUS[11:8]==InstReg[3][15:12]) : (IBUS[11:8]==InstReg[7][15:12]);
		I4DST_I3SRC1<=HalfReloadWire ? (IBUS[15:12]==InstReg[3][7:4]) : (IBUS[15:12]==InstReg[7][7:4]);
		I4DST_I3SRC2<=HalfReloadWire ? (IBUS[15:12]==InstReg[3][11:8]) : (IBUS[15:12]==InstReg[7][11:8]);
		I4DST_I3DST<=HalfReloadWire ? (IBUS[15:12]==InstReg[3][15:12]) : (IBUS[15:12]==InstReg[7][15:12]);
		I4SRC1_I2DST<=HalfReloadWire ? (IBUS[7:4]==InstReg[2][15:12]) : (IBUS[7:4]==InstReg[6][15:12]);
		I4SRC2_I2DST<=HalfReloadWire ? (IBUS[11:8]==InstReg[2][15:12]) : (IBUS[11:8]==InstReg[6][15:12]);
		I4DST_I2SRC1<=HalfReloadWire ? (IBUS[15:12]==InstReg[2][7:4]) : (IBUS[15:12]==InstReg[6][7:4]);
		I4DST_I2SRC2<=HalfReloadWire ? (IBUS[15:12]==InstReg[2][11:8]) : (IBUS[15:12]==InstReg[6][11:8]);
		I4DST_I2DST<=HalfReloadWire ? (IBUS[15:12]==InstReg[2][15:12]) : (IBUS[15:12]==InstReg[6][15:12]);
		I4SRC1_I1DST<=HalfReloadWire ? (IBUS[7:4]==InstReg[1][15:12]) : (IBUS[7:4]==InstReg[5][15:12]);
		I4SRC2_I1DST<=HalfReloadWire ? (IBUS[11:8]==InstReg[1][15:12]) : (IBUS[11:8]==InstReg[5][15:12]);
		I4DST_I1SRC1<=HalfReloadWire ? (IBUS[15:12]==InstReg[1][7:4]) : (IBUS[15:12]==InstReg[5][7:4]);
		I4DST_I1SRC2<=HalfReloadWire ? (IBUS[15:12]==InstReg[1][11:8]) : (IBUS[15:12]==InstReg[5][11:8]);
		I4DST_I1DST<=HalfReloadWire ? (IBUS[15:12]==InstReg[1][15:12]) : (IBUS[15:12]==InstReg[5][15:12]);
		I4SRC1_I0DST<=HalfReloadWire ? (IBUS[7:4]==InstReg[0][15:12]) : (IBUS[7:4]==InstReg[4][15:12]);
		I4SRC2_I0DST<=HalfReloadWire ? (IBUS[11:8]==InstReg[0][15:12]) : (IBUS[11:8]==InstReg[4][15:12]);
		I4DST_I0SRC1<=HalfReloadWire ? (IBUS[15:12]==InstReg[0][7:4]) : (IBUS[15:12]==InstReg[4][7:4]);
		I4DST_I0SRC2<=HalfReloadWire ? (IBUS[15:12]==InstReg[0][11:8]) : (IBUS[15:12]==InstReg[4][11:8]);
		I4DST_I0DST<=HalfReloadWire ? (IBUS[15:12]==InstReg[0][15:12]) : (IBUS[15:12]==InstReg[4][15:12]);

		I5SRC1_I4DST<=(IBUS[23:20]==IBUS[15:12]) & (IBUS[7:0] != 8'hFC);
		I5SRC2_I4DST<=(IBUS[27:24]==IBUS[15:12]) & (IBUS[7:0] != 8'hFC);
		I5DST_I4SRC1<=(IBUS[31:28]==IBUS[7:4]) & (IBUS[7:0] != 8'hFC);
		I5DST_I4SRC2<=(IBUS[31:28]==IBUS[11:8]) & (IBUS[7:0] != 8'hFC);
		I5DST_I4DST<=(IBUS[31:28]==IBUS[15:12]) & (IBUS[7:0] != 8'hFC);
		I5SRC1_I3DST<=HalfReloadWire ? (IBUS[23:20]==InstReg[3][15:12]) : (IBUS[23:20]==InstReg[7][15:12]);
		I5SRC2_I3DST<=HalfReloadWire ? (IBUS[27:24]==InstReg[3][15:12]) : (IBUS[27:24]==InstReg[7][15:12]);
		I5DST_I3SRC1<=HalfReloadWire ? (IBUS[31:28]==InstReg[3][7:4]) : (IBUS[31:28]==InstReg[7][7:4]);
		I5DST_I3SRC2<=HalfReloadWire ? (IBUS[31:28]==InstReg[3][11:8]) : (IBUS[31:28]==InstReg[7][11:8]);
		I5DST_I3DST<=HalfReloadWire ? (IBUS[31:28]==InstReg[3][15:12]) : (IBUS[31:28]==InstReg[7][15:12]);
		I5SRC1_I2DST<=HalfReloadWire ? (IBUS[23:20]==InstReg[2][15:12]) : (IBUS[23:20]==InstReg[6][15:12]);
		I5SRC2_I2DST<=HalfReloadWire ? (IBUS[27:24]==InstReg[2][15:12]) : (IBUS[27:24]==InstReg[6][15:12]);
		I5DST_I2SRC1<=HalfReloadWire ? (IBUS[31:28]==InstReg[2][7:4]) : (IBUS[31:28]==InstReg[6][7:4]);
		I5DST_I2SRC2<=HalfReloadWire ? (IBUS[31:28]==InstReg[2][11:8]) : (IBUS[31:28]==InstReg[6][11:8]);
		I5DST_I2DST<=HalfReloadWire ? (IBUS[31:28]==InstReg[2][15:12]) : (IBUS[31:28]==InstReg[6][15:12]);
		I5SRC1_I1DST<=HalfReloadWire ? (IBUS[23:20]==InstReg[1][15:12]) : (IBUS[23:20]==InstReg[5][15:12]);
		I5SRC2_I1DST<=HalfReloadWire ? (IBUS[27:24]==InstReg[1][15:12]) : (IBUS[27:24]==InstReg[5][15:12]);
		I5DST_I1SRC1<=HalfReloadWire ? (IBUS[31:28]==InstReg[1][7:4]) : (IBUS[31:28]==InstReg[5][7:4]);
		I5DST_I1SRC2<=HalfReloadWire ? (IBUS[31:28]==InstReg[1][11:8]) : (IBUS[31:28]==InstReg[5][11:8]);
		I5DST_I1DST<=HalfReloadWire ? (IBUS[31:28]==InstReg[1][15:12]) : (IBUS[31:28]==InstReg[5][15:12]);
		I5SRC1_I0DST<=HalfReloadWire ? (IBUS[23:20]==InstReg[0][15:12]) : (IBUS[23:20]==InstReg[4][15:12]);
		I5SRC2_I0DST<=HalfReloadWire ? (IBUS[27:24]==InstReg[0][15:12]) : (IBUS[27:24]==InstReg[4][15:12]);
		I5DST_I0SRC1<=HalfReloadWire ? (IBUS[31:28]==InstReg[0][7:4]) : (IBUS[31:28]==InstReg[4][7:4]);
		I5DST_I0SRC2<=HalfReloadWire ? (IBUS[31:28]==InstReg[0][11:8]) : (IBUS[31:28]==InstReg[4][11:8]);
		I5DST_I0DST<=HalfReloadWire ? (IBUS[31:28]==InstReg[0][15:12]) : (IBUS[31:28]==InstReg[4][15:12]);

		I6SRC1_I5DST<=(IBUS[39:36]==IBUS[31:28]);
		I6SRC2_I5DST<=(IBUS[43:40]==IBUS[31:28]);
		I6DST_I5SRC1<=(IBUS[47:44]==IBUS[23:20]);
		I6DST_I5SRC2<=(IBUS[47:44]==IBUS[27:24]);
		I6DST_I5DST<=(IBUS[47:44]==IBUS[31:28]);
		I6SRC1_I4DST<=(IBUS[39:36]==IBUS[15:12]);
		I6SRC2_I4DST<=(IBUS[43:40]==IBUS[15:12]);
		I6DST_I4SRC1<=(IBUS[47:44]==IBUS[7:4]);
		I6DST_I4SRC2<=(IBUS[47:44]==IBUS[11:8]);
		I6DST_I4DST<=(IBUS[47:44]==IBUS[15:12]);
		I6SRC1_I3DST<=HalfReloadWire ? (IBUS[39:36]==InstReg[3][15:12]) : (IBUS[39:36]==InstReg[7][15:12]);
		I6SRC2_I3DST<=HalfReloadWire ? (IBUS[43:40]==InstReg[3][15:12]) : (IBUS[43:40]==InstReg[7][15:12]);
		I6DST_I3SRC1<=HalfReloadWire ? (IBUS[47:44]==InstReg[3][7:4]) : (IBUS[47:44]==InstReg[7][7:4]);
		I6DST_I3SRC2<=HalfReloadWire ? (IBUS[47:44]==InstReg[3][11:8]) : (IBUS[47:44]==InstReg[7][11:8]);
		I6DST_I3DST<=HalfReloadWire ? (IBUS[47:44]==InstReg[3][15:12]) : (IBUS[47:44]==InstReg[7][15:12]);
		I6SRC1_I2DST<=HalfReloadWire ? (IBUS[39:36]==InstReg[2][15:12]) : (IBUS[39:36]==InstReg[6][15:12]);
		I6SRC2_I2DST<=HalfReloadWire ? (IBUS[43:40]==InstReg[2][15:12]) : (IBUS[43:40]==InstReg[6][15:12]);
		I6DST_I2SRC1<=HalfReloadWire ? (IBUS[47:44]==InstReg[2][7:4]) : (IBUS[47:44]==InstReg[6][7:4]);
		I6DST_I2SRC2<=HalfReloadWire ? (IBUS[47:44]==InstReg[2][11:8]) : (IBUS[47:44]==InstReg[6][11:8]);
		I6DST_I2DST<=HalfReloadWire ? (IBUS[47:44]==InstReg[2][15:12]) : (IBUS[47:44]==InstReg[6][15:12]);
		I6SRC1_I1DST<=HalfReloadWire ? (IBUS[39:36]==InstReg[1][15:12]) : (IBUS[39:36]==InstReg[5][15:12]);
		I6SRC2_I1DST<=HalfReloadWire ? (IBUS[43:40]==InstReg[1][15:12]) : (IBUS[43:40]==InstReg[5][15:12]);
		I6DST_I1SRC1<=HalfReloadWire ? (IBUS[47:44]==InstReg[1][7:4]) : (IBUS[47:44]==InstReg[5][7:4]);
		I6DST_I1SRC2<=HalfReloadWire ? (IBUS[47:44]==InstReg[1][11:8]) : (IBUS[47:44]==InstReg[5][11:8]);
		I6DST_I1DST<=HalfReloadWire ? (IBUS[47:44]==InstReg[1][15:12]) : (IBUS[47:44]==InstReg[5][15:12]);
		I6SRC1_I0DST<=HalfReloadWire ? (IBUS[39:36]==InstReg[0][15:12]) : (IBUS[39:36]==InstReg[4][15:12]);
		I6SRC2_I0DST<=HalfReloadWire ? (IBUS[43:40]==InstReg[0][15:12]) : (IBUS[43:40]==InstReg[4][15:12]);
		I6DST_I0SRC1<=HalfReloadWire ? (IBUS[47:44]==InstReg[0][7:4]) : (IBUS[47:44]==InstReg[4][7:4]);
		I6DST_I0SRC2<=HalfReloadWire ? (IBUS[47:44]==InstReg[0][11:8]) : (IBUS[47:44]==InstReg[4][11:8]);
		I6DST_I0DST<=HalfReloadWire ? (IBUS[47:44]==InstReg[0][15:12]) : (IBUS[47:44]==InstReg[4][15:12]);

		I7SRC1_I6DST<=(IBUS[55:52]==IBUS[47:44]) & (IBUS[39:32] != 8'hFC);
		I7SRC2_I6DST<=(IBUS[59:56]==IBUS[47:44]) & (IBUS[39:32] != 8'hFC);
		I7DST_I6SRC1<=(IBUS[63:60]==IBUS[39:36]) & (IBUS[39:32] != 8'hFC);
		I7DST_I6SRC2<=(IBUS[63:60]==IBUS[43:40]) & (IBUS[39:32] != 8'hFC);
		I7DST_I6DST<=(IBUS[63:60]==IBUS[47:44]) & (IBUS[39:32] != 8'hFC);
		I7SRC1_I5DST<=(IBUS[55:52]==IBUS[31:28]);
		I7SRC2_I5DST<=(IBUS[59:56]==IBUS[31:28]);
		I7DST_I5SRC1<=(IBUS[63:60]==IBUS[23:20]);
		I7DST_I5SRC2<=(IBUS[63:60]==IBUS[27:24]);
		I7DST_I5DST<=(IBUS[63:60]==IBUS[31:28]);
		I7SRC1_I4DST<=(IBUS[55:52]==IBUS[15:12]);
		I7SRC2_I4DST<=(IBUS[59:56]==IBUS[15:12]);
		I7DST_I4SRC1<=(IBUS[63:60]==IBUS[7:4]);
		I7DST_I4SRC2<=(IBUS[63:60]==IBUS[11:8]);
		I7DST_I4DST<=(IBUS[63:60]==IBUS[15:12]);
		I7SRC1_I3DST<=HalfReloadWire ? (IBUS[55:52]==InstReg[3][15:12]) : (IBUS[55:52]==InstReg[7][15:12]);
		I7SRC2_I3DST<=HalfReloadWire ? (IBUS[59:56]==InstReg[3][15:12]) : (IBUS[59:56]==InstReg[7][15:12]);
		I7DST_I3SRC1<=HalfReloadWire ? (IBUS[63:60]==InstReg[3][7:4]) : (IBUS[63:60]==InstReg[7][7:4]);
		I7DST_I3SRC2<=HalfReloadWire ? (IBUS[63:60]==InstReg[3][11:8]) : (IBUS[63:60]==InstReg[7][11:8]);
		I7DST_I3DST<=HalfReloadWire ? (IBUS[63:60]==InstReg[3][15:12]) : (IBUS[63:60]==InstReg[7][15:12]);
		I7SRC1_I2DST<=HalfReloadWire ? (IBUS[55:52]==InstReg[2][15:12]) : (IBUS[55:52]==InstReg[6][15:12]);
		I7SRC2_I2DST<=HalfReloadWire ? (IBUS[59:56]==InstReg[2][15:12]) : (IBUS[59:56]==InstReg[6][15:12]);
		I7DST_I2SRC1<=HalfReloadWire ? (IBUS[63:60]==InstReg[2][7:4]) : (IBUS[63:60]==InstReg[6][7:4]);
		I7DST_I2SRC2<=HalfReloadWire ? (IBUS[63:60]==InstReg[2][11:8]) : (IBUS[63:60]==InstReg[6][11:8]);
		I7DST_I2DST<=HalfReloadWire ? (IBUS[63:60]==InstReg[2][15:12]) : (IBUS[63:60]==InstReg[6][15:12]);
		I7SRC1_I1DST<=HalfReloadWire ? (IBUS[55:52]==InstReg[1][15:12]) : (IBUS[55:52]==InstReg[5][15:12]);
		I7SRC2_I1DST<=HalfReloadWire ? (IBUS[59:56]==InstReg[1][15:12]) : (IBUS[59:56]==InstReg[5][15:12]);
		I7DST_I1SRC1<=HalfReloadWire ? (IBUS[63:60]==InstReg[1][7:4]) : (IBUS[63:60]==InstReg[5][7:4]);
		I7DST_I1SRC2<=HalfReloadWire ? (IBUS[63:60]==InstReg[1][11:8]) : (IBUS[63:60]==InstReg[5][11:8]);
		I7DST_I1DST<=HalfReloadWire ? (IBUS[63:60]==InstReg[1][15:12]) : (IBUS[63:60]==InstReg[5][15:12]);
		I7SRC1_I0DST<=HalfReloadWire ? (IBUS[55:52]==InstReg[0][15:12]) : (IBUS[55:52]==InstReg[4][15:12]);
		I7SRC2_I0DST<=HalfReloadWire ? (IBUS[59:56]==InstReg[0][15:12]) : (IBUS[59:56]==InstReg[4][15:12]);
		I7DST_I0SRC1<=HalfReloadWire ? (IBUS[63:60]==InstReg[0][7:4]) : (IBUS[63:60]==InstReg[4][7:4]);
		I7DST_I0SRC2<=HalfReloadWire ? (IBUS[63:60]==InstReg[0][11:8]) : (IBUS[63:60]==InstReg[4][11:8]);
		I7DST_I0DST<=HalfReloadWire ? (IBUS[63:60]==InstReg[0][15:12]) : (IBUS[63:60]==InstReg[4][15:12]);
		end	
	// instruction valid flags
	for (i=0; i<4; i=i+1)
			if (InstReloadWire)
					begin
					VFReg[i]<=VFReg[i+4] & ~VFResetWire[i+4];
					VFReg[i+4]<=IRDY & VF[i];
					end
					else begin
					if (HalfReloadWire)
							begin
							VFReg[i]<=VFReg[i] & ~VFResetWire[i];
							VFReg[i+4]<=IRDY & VF[i];						
							end
							else begin
								VFReg[i]<=VFReg[i] & ~VFResetWire[i];
								VFReg[i+4]<=VFReg[i+4] & ~VFResetWire[i+4];
								end
					end
	// instruction type flags and operand verify flags
	for (i=0; i<4; i=i+1) 
				if (InstReloadWire)
						begin
						// Invalidation register
						GPRInvdR[i]<=GPRInvdR[i+4];
						// type regs
						TypeReg[i]<=TypeReg[i+4] & {11{~VFResetWire[i+4]}};
						// Operand flags
						OpReg[i]<=OpReg[i+4];
						// extension flags
						if (i[0]==1'b0)	ExtReg[i>>1]<=ExtReg[(i+4)>>1];

						if ((i==0) | (i==2))
							begin
							for (j=0; j<16; j=j+1)
							GPRInvdR[i+4][j]<=~IRDY | ~VF[i] | ~({IBUS[15+(i<<4)],IBUS[14+(i<<4)],IBUS[13+(i<<4)],IBUS[12+(i<<4)]}==j) |
											((~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | ~IBUS[0+(i<<4)]) &
											 (IBUS[6+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[i<<4]) &
											 (~IBUS[3+(i<<4)] | IBUS[2+(i<<4)]) &
											 (~IBUS[4+(i<<4)] | IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (~IBUS[2+(i<<4)] | ~(IBUS[1+(i<<4)] | IBUS[0+(i<<4)])) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)] |
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]) |
													((IBUS[11+(i<<4)] | (~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] &  IBUS[5+(i<<4)] & IBUS[4+(i<<4)])));
							// extension flags
							ExtReg[(i+4)>>1]<=IRDY & VF[i] & ~(IBUS[1+((i+1)<<4)] & IBUS[(i+1)<<4]) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
												IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)];
							// FADD Flags
							TypeReg[i+4][0]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)];
							// FMUL Flags
							TypeReg[i+4][1]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4];
							// FDIV Flags
							TypeReg[i+4][2]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4]) |
												(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]));
							// ALU Flags
							TypeReg[i+4][3]<=IRDY & VF[i] & IBUS[1+(i<<4)] & ((~IBUS[3+(i<<4)] & IBUS[2+(i<<4)]) | 
										(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[i<<4]) |
										(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4]));
							// Shift Flags
							TypeReg[i+4][4]<=IRDY & VF[i] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
										(~IBUS[6+(i<<4)] | (IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)]));
							// Miscellaneous flags
							TypeReg[i+4][5]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
										((~IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
										(IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]))) |
										(~IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
										(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[0+(i<<4)]));
							// Movement flags COPY, SETSIZE/AMODE, LI
							TypeReg[i+4][6]<=IRDY & VF[i] & ((~(IBUS[7+(i<<4)] ^ IBUS[5+(i<<4)]) & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)] &
													~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
													(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4]));
							// LD/ST/PUSH/POP/LAR/SAR operations
							TypeReg[i+4][7]<=IRDY & VF[i] & ((IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[i<<4])|
												(IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & ~IBUS[3+(i<<4)])|
												(((~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)])) &
												  ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4]));
							//Flow control instructions
							TypeReg[i+4][8]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4])|
												(IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
												(IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & 
												~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
													((~IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])|
													 (~IBUS[15+(i<<4)] & ~IBUS[14+(i<<4)] & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)])))));
							// FMULACC instruction
							TypeReg[i+4][9]<=IRDY & VF[i] & IBUS[1+((i+1)<<4)] & IBUS[(i+1)<<4] & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
												IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)];
							// Invalid instruction
							TypeReg[i+4][10]<=IRDY & VF[i] & (IBUS[15+(i<<4)] | IBUS[14+(i<<4)]) & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)] &
												IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4];
							// verification request for SRC_A  operand
							OpReg[i+4][0]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)]) |
									(IBUS[1+(i<<4)] & (~IBUS[3+(i<<4)] | (IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4]))));
							// verification request for SRC B operand
							OpReg[i+4][1]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4])|
											(IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)] &
												(~IBUS[7+(i<<4)] | ~IBUS[1+((i+1)<<4)] | ~IBUS[(i+1)<<4])) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] ^ IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
											((IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)])|
											  (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)])|
											  (~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)]))));
							// verification request for DST field as destination
							OpReg[i+4][2]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[0+(i<<4)])|
											(IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] | IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)] &
											   ((~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | (IBUS[7+(i<<4)] ^ IBUS[4+(i<<4)])) |
											   (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
											       (~IBUS[11+(i<<4)] | (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)])))))));
							end
							else begin
							for (j=0; j<16; j=j+1)
							GPRInvdR[i+4][j]<=~IRDY | ~VF[i] | ~({IBUS[15+(i<<4)],IBUS[14+(i<<4)],IBUS[13+(i<<4)],IBUS[12+(i<<4)]}==j) |
											(VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] & IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]) |
											((IBUS[7+(i<<4)] | ~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | ~IBUS[0+(i<<4)]) &
											 (IBUS[6+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[i<<4]) &
											 (~IBUS[3+(i<<4)] | IBUS[2+(i<<4)]) &
											 (~IBUS[4+(i<<4)] | IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (~IBUS[2+(i<<4)] | ~(IBUS[1+(i<<4)] | IBUS[0+(i<<4)])) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)] |
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]) |
													((IBUS[11+(i<<4)] | (~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] &  IBUS[5+(i<<4)] & IBUS[4+(i<<4)])));

							// FADD Flags
							TypeReg[i+4][0]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							// FMUL Flags
							TypeReg[i+4][1]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4] &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							// FDIV Flags
							TypeReg[i+4][2]<=IRDY & VF[i] & (((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4]) |
												(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])) &
												(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] |
													~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]));
							// ALU Flags
							TypeReg[i+4][3]<=IRDY & VF[i] & ((IBUS[1+(i<<4)] & ((~IBUS[3+(i<<4)] & IBUS[2+(i<<4)]) | 
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[i<<4]) |
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)])) |
									(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~(IBUS[1+(i<<4)] & IBUS[0+(i<<4)]) & VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] &
										IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]));
							// Shift Flags
							TypeReg[i+4][4]<=IRDY & VF[i] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
									(~IBUS[6+(i<<4)] | (IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							// Miscellaneous flags
							TypeReg[i+4][5]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
									((~IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
									 (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]))) |
									(~IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
									(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[0+(i<<4)])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							// Movement flags COPY, SETSIZE/AMODE, LI
							TypeReg[i+4][6]<=IRDY & VF[i] & ((~(IBUS[7+(i<<4)] ^ IBUS[5+(i<<4)]) & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)] &
												~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
												(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							// LD/ST/PUSH/POP/LAR/SAR operations
							TypeReg[i+4][7]<=IRDY & VF[i] & ((IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[i<<4])|
												(IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & ~IBUS[3+(i<<4)])|
												(((~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)])) &
												  ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							//Flow control instructions
							TypeReg[i+4][8]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4])|
												(IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
												(IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & 
												~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
													((~IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])|
													 (~IBUS[15+(i<<4)] & ~IBUS[14+(i<<4)] & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)]))))) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							TypeReg[i+4][9]<=1'b0;
							// Invalid instruction
							TypeReg[i+4][10]<=IRDY & VF[i] & (IBUS[15+(i<<4)] | IBUS[14+(i<<4)]) & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)] &
												IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
												(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							// verification request for SRC_A  operand
							OpReg[i+4][0]<=IRDY & VF[i] & ((((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)]) |
									(IBUS[1+(i<<4)] & (~IBUS[3+(i<<4)] | (IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4])))) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] |
										~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)])) | 
									((IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) & VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] &
											IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]));
							// verification request for SRC B operand
							OpReg[i+4][1]<=IRDY & VF[i] & ((((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4])|
											(IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] ^ IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
											((IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)])|
											  (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)])|
											  (~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)])))) &
											(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] |
													~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)])) |
											((IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) & VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] &
											IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]));
							// verification request for DST field as destination
							OpReg[i+4][2]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[0+(i<<4)])|
											(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] | IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)] &
											   ((~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | (IBUS[7+(i<<4)] ^ IBUS[4+(i<<4)])) |
											   (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
											       (~IBUS[11+(i<<4)] | (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]))))))) &
											(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] |
													~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
							end
						end
					else begin
						if (HalfReloadWire)
							begin
							TypeReg[i]<=TypeReg[i] & {11{~VFResetWire[i]}};

							if ((i==0) | (i==2))
								begin
								for (j=0; j<16; j=j+1)
								GPRInvdR[i+4][j]<=~IRDY | ~VF[i] | ~({IBUS[15+(i<<4)],IBUS[14+(i<<4)],IBUS[13+(i<<4)],IBUS[12+(i<<4)]}==j) |
											((~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | ~IBUS[0+(i<<4)]) &
											 (IBUS[6+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[i<<4]) &
											 (~IBUS[3+(i<<4)] | IBUS[2+(i<<4)]) &
											 (~IBUS[4+(i<<4)] | IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (~IBUS[2+(i<<4)] | ~(IBUS[1+(i<<4)] | IBUS[0+(i<<4)])) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)] |
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]) |
													((IBUS[11+(i<<4)] | (~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] &  IBUS[5+(i<<4)] & IBUS[4+(i<<4)])));
								// extension flags
								ExtReg[(i+4)>>1]<=IRDY & VF[i] & ~(IBUS[1+((i+1)<<4)] & IBUS[(i+1)<<4]) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
												IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)];
								// FADD Flags
								TypeReg[i+4][0]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)];
								// FMUL Flags
								TypeReg[i+4][1]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4];
								// FDIV Flags
								TypeReg[i+4][2]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4]) |
												(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]));
								// ALU Flags
								TypeReg[i+4][3]<=IRDY & VF[i] & IBUS[1+(i<<4)] & ((~IBUS[3+(i<<4)] & IBUS[2+(i<<4)]) | 
										(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[i<<4]) |
										(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4]));
								// Shift Flags
								TypeReg[i+4][4]<=IRDY & VF[i] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
										(~IBUS[6+(i<<4)] | (IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)]));
								// Miscellaneous flags
								TypeReg[i+4][5]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
										((~IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
										(IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]))) |
										(~IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
										(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[0+(i<<4)]));
								// Movement flags COPY, SETSIZE/AMODE, LI
								TypeReg[i+4][6]<=IRDY & VF[i] & ((~(IBUS[7+(i<<4)] ^ IBUS[5+(i<<4)]) & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)] &
													~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
													(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4]));
								// LD/ST/PUSH/POP/LAR/SAR operations
								TypeReg[i+4][7]<=IRDY & VF[i] & ((IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[i<<4])|
												(IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & ~IBUS[3+(i<<4)])|
												(((~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)])) &
												  ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4]));
								//Flow control instructions
								TypeReg[i+4][8]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4])|
												(IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
												(IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & 
												~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
													((~IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])|
													 (~IBUS[15+(i<<4)] & ~IBUS[14+(i<<4)] & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)])))));
								// FMULACC instruction
								TypeReg[i+4][9]<=IRDY & VF[i] & IBUS[1+((i+1)<<4)] & IBUS[(i+1)<<4] & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
												IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)];
								// Invalid instruction
								TypeReg[i+4][10]<=IRDY & VF[i] & (IBUS[15+(i<<4)] | IBUS[14+(i<<4)]) & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)] &
												IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4];
								// verification request for SRC_A  operand
								OpReg[i+4][0]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)]) |
									(IBUS[1+(i<<4)] & (~IBUS[3+(i<<4)] | (IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4]))));
								// verification request for SRC B operand
								OpReg[i+4][1]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4])|
											(IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)] &
												(~IBUS[7+(i<<4)] | ~IBUS[1+((i+1)<<4)] | ~IBUS[(i+1)<<4])) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] ^ IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
											((IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)])|
											  (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)])|
											  (~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)]))));
								// verification request for DST field as destination
								OpReg[i+4][2]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[0+(i<<4)])|
											(IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] | IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)] &
											   ((~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | (IBUS[7+(i<<4)] ^ IBUS[4+(i<<4)])) |
											   (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
											       (~IBUS[11+(i<<4)] | (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)])))))));
								end
							else begin
								for (j=0; j<16; j=j+1)
								GPRInvdR[i+4][j]<=~IRDY | ~VF[i] | ~({IBUS[15+(i<<4)],IBUS[14+(i<<4)],IBUS[13+(i<<4)],IBUS[12+(i<<4)]}==j) |
											(VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] & IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]) |
											((IBUS[7+(i<<4)] | ~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | ~IBUS[0+(i<<4)]) &
											 (IBUS[6+(i<<4)] | ~IBUS[4+(i<<4)] | ~IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[i<<4]) &
											 (~IBUS[3+(i<<4)] | IBUS[2+(i<<4)]) &
											 (~IBUS[4+(i<<4)] | IBUS[3+(i<<4)] | ~IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) &
											 (~IBUS[2+(i<<4)] | ~(IBUS[1+(i<<4)] | IBUS[0+(i<<4)])) &
											 (IBUS[3+(i<<4)] | IBUS[2+(i<<4)] | IBUS[1+(i<<4)] | IBUS[0+(i<<4)] |
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]) |
													((IBUS[11+(i<<4)] | (~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] &  IBUS[5+(i<<4)] & IBUS[4+(i<<4)])));

								// FADD Flags
								TypeReg[i+4][0]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								// FMUL Flags
								TypeReg[i+4][1]<=IRDY & VF[i] & IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4] &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								// FDIV Flags
								TypeReg[i+4][2]<=IRDY & VF[i] & (((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4]) |
												(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])) &
												(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] |
													~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]));
								// ALU Flags
								TypeReg[i+4][3]<=IRDY & VF[i] & ((IBUS[1+(i<<4)] & ((~IBUS[3+(i<<4)] & IBUS[2+(i<<4)]) | 
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[i<<4]) |
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)])) |
									(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~(IBUS[1+(i<<4)] & IBUS[0+(i<<4)]) & VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] &
										IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]));
								// Shift Flags
								TypeReg[i+4][4]<=IRDY & VF[i] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
									(~IBUS[6+(i<<4)] | (IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								// Miscellaneous flags
								TypeReg[i+4][5]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
									((~IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
									 (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & ~IBUS[4+(i<<4)]))) |
									(~IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
									(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[0+(i<<4)])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								// Movement flags COPY, SETSIZE/AMODE, LI
								TypeReg[i+4][6]<=IRDY & VF[i] & ((~(IBUS[7+(i<<4)] ^ IBUS[5+(i<<4)]) & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)] &
												~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])|
												(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								// LD/ST/PUSH/POP/LAR/SAR operations
								TypeReg[i+4][7]<=IRDY & VF[i] & ((IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & IBUS[i<<4])|
												(IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & ~IBUS[3+(i<<4)])|
												(((~IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)])|
													(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)])) &
												  ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4])) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								//Flow control instructions
								TypeReg[i+4][8]<=IRDY & VF[i] & ((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & ~IBUS[i<<4])|
												(IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
												(IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & 
												~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] & 
													((~IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]) |
													 (IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & ~IBUS[9+(i<<4)] & ~IBUS[8+(i<<4)])|
													 (~IBUS[15+(i<<4)] & ~IBUS[14+(i<<4)] & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)]))))) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								TypeReg[i+4][9]<=1'b0;
								// Invalid instruction
								TypeReg[i+4][10]<=IRDY & VF[i] & (IBUS[15+(i<<4)] | IBUS[14+(i<<4)]) & IBUS[11+(i<<4)] & IBUS[10+(i<<4)] & IBUS[9+(i<<4)] & IBUS[8+(i<<4)] &
												IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & ~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
												(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								// verification request for SRC_A  operand
								OpReg[i+4][0]<=IRDY & VF[i] & ((((IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)]) |
									(IBUS[1+(i<<4)] & (~IBUS[3+(i<<4)] | (IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[i<<4])))) &
									(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] | ~IBUS[3+((i-1)<<4)] |
										~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)])) | 
									((IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) & VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] &
											IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]));
								// verification request for SRC B operand
								OpReg[i+4][1]<=IRDY & VF[i] & ((((~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[1+(i<<4)] & IBUS[i<<4])|
											(IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] ^ IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[i<<4] &
											((IBUS[7+(i<<4)] & ~IBUS[6+(i<<4)])|
											  (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[5+(i<<4)])|
											  (~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & ~IBUS[4+(i<<4)])))) &
											(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] |
													~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)])) |
											((IBUS[1+(i<<4)] | IBUS[0+(i<<4)]) & VF[i-1] & IBUS[7+((i-1)<<4)] & IBUS[6+((i-1)<<4)] & IBUS[5+((i-1)<<4)] & IBUS[4+((i-1)<<4)] &
											IBUS[3+((i-1)<<4)] & IBUS[2+((i-1)<<4)] & ~IBUS[1+((i-1)<<4)] & ~IBUS[0+((i-1)<<4)]));
								// verification request for DST field as destination
								OpReg[i+4][2]<=IRDY & VF[i] & ((IBUS[3+(i<<4)] ^ IBUS[2+(i<<4)])|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & IBUS[0+(i<<4)])|
											(~IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)]) |
											(~IBUS[6+(i<<4)] & IBUS[4+(i<<4)] & IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)])|
											(IBUS[3+(i<<4)] & IBUS[2+(i<<4)] & (IBUS[1+(i<<4)] | IBUS[0+(i<<4)]))|
											(~IBUS[3+(i<<4)] & ~IBUS[2+(i<<4)] & ~IBUS[1+(i<<4)] & ~IBUS[0+(i<<4)] &
											   ((~IBUS[6+(i<<4)] | ~IBUS[5+(i<<4)] | (IBUS[7+(i<<4)] ^ IBUS[4+(i<<4)])) |
											   (IBUS[7+(i<<4)] & IBUS[6+(i<<4)] & IBUS[5+(i<<4)] & IBUS[4+(i<<4)] &
											       (~IBUS[11+(i<<4)] | (IBUS[11+(i<<4)] & (IBUS[10+(i<<4)] ^ IBUS[9+(i<<4)]))))))) &
											(~VF[i-1] | ~IBUS[7+((i-1)<<4)] | ~IBUS[6+((i-1)<<4)] | ~IBUS[5+((i-1)<<4)] | ~IBUS[4+((i-1)<<4)] |
													~IBUS[3+((i-1)<<4)] | ~IBUS[2+((i-1)<<4)] | IBUS[1+((i-1)<<4)] | IBUS[0+((i-1)<<4)]);
								end
							end
							else begin
								TypeReg[i]<=TypeReg[i] & {11{~VFResetWire[i]}};
								TypeReg[i+4]<=TypeReg[i+4] & {11{~VFResetWire[i+4]}};
								end
						end
				
	end
	end

endmodule
