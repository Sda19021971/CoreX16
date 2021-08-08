module Messenger (
	input wire CLK, RESETn,
	output reg RST,
	// message interface from EU, error system, interrupt system and network system
	input wire EUREQ, ERRORREQ, INTREQ, NETREQ,
	input wire [63:0] EUPARAM,		// Message index for import table in the current PSO and parameter dword
	input wire [15:0] INTPARAM,		// Interrupt index
	input wire [121:0] NETPARAM,	// BUS(msb:lsb): CPL[121:120], Target PSO[119:96], TaskID[95:80], ProcINDX[79:64], Parameter[63:32], SourcePSO[31:0]
	output reg NETMSGRD,
	output reg INTACK,
	// interface to memory system
	input wire NEXT,
	output wire ACT, CMD,
	output wire [44:0] ADDR,
	output wire [1:0] SIZE,
	output wire [31:0] DATO,
	input wire DRDY,
	input wire [31:0] DATI,
	// interface to the context controller (interrupt, error or procedure)
	input wire ContextRDY,
	output reg ContextREQ, ContextCHK,
	output wire [95:0] ContextMSG,
	// interface to network controller
	input wire NETRDY,
	output reg NETSEND,
	output reg NETTYPE,
	output wire [79:0] NETMSG,
	output reg [4:0] NETSTAT,
	// DTR and DTL registers
	input wire [39:0] DTBASE,
	input wire [23:0] DTLIMIT,
	input wire [7:0] CPUNUM,
	input wire [23:0] CPSR,
	input logic [23:0] INTSR,			// selector IT
	input logic [15:0] INTCR,			// Length of IT
	input logic [31:0] CSR,
	// core control lines 
	output reg EUCONTINUE,
	output reg EUHALT,
	input wire BKPT,
	// error reporting interface
	output reg ESTB,
	output reg [63:0] ERRC
	);

extern module Microcontroller #(parameter Code) 
(	input wire CLK, RESET, CF,
	// IO Bus
	output reg [3:0] IOWSTB,
	output reg [3:0] IORSTB,
	input wire [31:0] IOSRC [3:0],
	// memory interface
	input wire NEXT,
	output reg ACT, CMD,
	output reg [1:0] SIZE,
	output reg [44:0] ADDRESS,
	output reg [31:0] DTo,
	input wire DRDY,
	input wire [31:0] DTi,
	// external requests
	input wire [7:0] REQ,
	input wire [7:0] IMASK,
	// external base sources
	input wire [39:0] BASE [1:0]
);

reg [95:0] ContextMSGReg;
reg [79:0] NETMSGReg;

logic [3:0] MPUWSTB;
logic [3:0] MPURSTB;
logic [7:0] MPUReqBus;
wire [39:0] MPUBaseBus[1:0];
logic [31:0] IOSRCBus[3:0];
logic [31:0] MPUDOUTBus;
reg ExtFlag, NETSENDFlag, ContextREQFlag, INTREQFlag, INTACKFlag;
reg [31:0] DMuxReg;
reg [2:0] DMuxSelReg, CondSelReg;

Microcontroller MPU (.CLK(CLK),.RESET(RESETn), .CF(ExtFlag), .IOWSTB(MPUWSTB),.IORSTB(MPURSTB),.IOSRC(IOSRCBus),
					.NEXT(NEXT),.ACT(ACT),.CMD(CMD),.SIZE(SIZE),.ADDRESS(ADDR),.DTo(MPUDOUTBus),.DRDY(DRDY),
					.DTi(DATI), .REQ(MPUReqBus), .IMASK(8'hFF), .BASE(MPUBaseBus));

defparam MPU.Code="Messenger.mif";

assign DATO=MPUDOUTBus;
assign ContextMSG=ContextMSGReg;
assign NETMSG=NETMSGReg;
// base address
assign MPUBaseBus[0]=DTBASE;
assign MPUBaseBus[1]=40'hFFFFFFFFFC;		// base to the control registers block


/*
===================================================================================================
		Asynchronous functions
===================================================================================================
*/
always_comb
begin


// request from network interface
MPUReqBus[0]=NETREQ;
// request from error processing system
MPUReqBus[1]=ERRORREQ;
// request from interrupt processing system
MPUReqBus[2]=INTREQ & ~INTREQFlag;
// request from EU
MPUReqBus[3]=EUREQ;
// request breakpoint
MPUReqBus[4]=BKPT;
// reserved requests
MPUReqBus[7:5]=3'd0;

// IO source bus
IOSRCBus[0]=DMuxReg;
IOSRCBus[1]={6'd0,CSR[19:18],CPSR};						// CPL and bits [23:0] of PSO selector
IOSRCBus[2]=CSR;									// CSR register content
IOSRCBus[3]={16'd0,INTCR};									// interrupt table length

end

/*
===================================================================================================
		Registered functions
===================================================================================================
*/
always_ff @(posedge CLK)
begin
// generate reset signal when fatal error occurs
RST<=(RST | ~RESETn) & (~MPUWSTB[0] | ~MPUDOUTBus[7]);
end

always_ff @(negedge RESETn or posedge CLK)
begin
if (!RESETn) begin
				ContextREQ<=0;
				ContextREQFlag<=0;
				ContextCHK<=0;
				NETSEND<=0;
				NETSENDFlag<=0;
				EUCONTINUE<=0;
				EUHALT<=0;
				ESTB<=0;
				INTREQFlag<=0;
				INTACKFlag<=0;
				end
else begin
/*
		control register fields MPUWSTB[0]
[2:0] 	- DMuxSelReg
[3]		- enable write to the DMuxSelReg
[4]		- EUCONTINUE signal generation
[5]		- EUHALT signal generation
[6]		- confirmation to the network controller
[7]		- reset signal generation
[10:8]	- condition flag selection
[11]	- enable write to the condselreg
[12]	- ContextCHK generation
*/

// Input data mux
case (DMuxSelReg)
	3'd0:	DMuxReg<=EUPARAM[31:0];			// message index
	3'd1:	DMuxReg<=EUPARAM[63:32];		// message parameter
	3'd2:	DMuxReg<={16'd0,INTPARAM};		// interrupt index 
	3'd3:	DMuxReg<={8'd0,INTSR};			// interrupt table selector
	3'd4:	DMuxReg<=NETPARAM[63:32];		// message parameter
	3'd5:	DMuxReg<=NETPARAM[95:64];		// Task ID and Proc Index
	3'd6:	DMuxReg<={6'd0, NETPARAM[121:120], NETPARAM[119:96]};	// target PSO selector and CPL
	3'd7:	DMuxReg<={CPUNUM,DTLIMIT};
	endcase
if (MPUWSTB[0] & MPUDOUTBus[3]) DMuxSelReg<=MPUDOUTBus[2:0];
	else if (MPUWSTB[1] | MPURSTB[0]) DMuxSelReg<=DMuxSelReg+3'd1;		// autoincrement

// message read confirmation to the network controller
NETMSGRD<=MPUWSTB[0] & MPUDOUTBus[6];

// Channel to the context controller
// if ContextMSG==0 request for check message queues in the current process
if ((DMuxSelReg==3'd0) & MPUWSTB[1]) ContextMSGReg[31:0]<=MPUDOUTBus;		// subaddress 0 - procedure offset
if ((DMuxSelReg==3'd1) & MPUWSTB[1]) ContextMSGReg[63:32]<=MPUDOUTBus;		// subaddress 1 - procedure selector and control byte
if ((DMuxSelReg==3'd2) & MPUWSTB[1]) ContextMSGReg[95:64]<=MPUDOUTBus;		// subaddress 2 - PSO (for interrupts) or message parameter (for procedure calls)
ContextREQ<=(DMuxSelReg==3'd2) & MPUWSTB[1];							// request to the context controller
ContextREQFlag<=(ContextREQFlag | ((DMuxSelReg==3'd2) & MPUWSTB[1])) & ~ContextRDY;
// generation ContextCHK signal
ContextCHK<=MPUWSTB[0] & MPUDOUTBus[12];

// Channel to the network controller
if (((DMuxSelReg==3'd3) & MPUWSTB[1]) | MPUWSTB[2]) NETMSGReg[31:0]<=MPUWSTB[2] ? NETPARAM[31:0]:MPUDOUTBus;			// subaddress 2 - PSO selector and CPU index
if ((DMuxSelReg==3'd4) & MPUWSTB[1]) NETMSGReg[47:32]<=MPUDOUTBus[15:0];	// subaddress 3 - procedure index
if ((DMuxSelReg==3'd5) & MPUWSTB[1]) NETMSGReg[79:48]<=MPUDOUTBus;			// subaddress 4 - message parameter
NETSEND<=((DMuxSelReg==3'd5) & MPUWSTB[1]) | MPUWSTB[2];
NETSENDFlag<=(NETSENDFlag | (((DMuxSelReg==3'd5) & MPUWSTB[1]) | MPUWSTB[2])) & ~NETRDY;

// network reporting
if (((DMuxSelReg==3'd5) & MPUWSTB[1]) | MPUWSTB[2]) NETTYPE<=MPUWSTB[2];
if (MPUWSTB[2]) NETSTAT<=MPUDOUTBus[4:0];

// EU control
EUCONTINUE<=MPUWSTB[0] & MPUDOUTBus[4];
EUHALT<=MPUWSTB[0] & MPUDOUTBus[5];

// Error reporting interface
ESTB<=(DMuxSelReg==3'd7) & MPUWSTB[1];
if ((DMuxSelReg==3'd6) & MPUWSTB[1]) ERRC[31:0]<=MPUDOUTBus;			// subaddress 6 - low dword of error code
if ((DMuxSelReg==3'd7) & MPUWSTB[1]) ERRC[63:32]<=MPUDOUTBus;			// subaddress 7 - high dword of error code

// interrupt acknowledgement
INTACK<=(DMuxSelReg==3'd2) & MPURSTB[0];

// External condition flag
case (CondSelReg)
	3'd0:	ExtFlag<=ContextREQFlag;
	3'd1:	ExtFlag<=NETSENDFlag;
	3'd2:	ExtFlag<=1'b0;
	3'd3:	ExtFlag<=1'b0;
	3'd4:	ExtFlag<=1'b0;
	3'd5:	ExtFlag<=1'b0;
	3'd6:	ExtFlag<=1'b0;
	3'd7:	ExtFlag<=1'b0;
	endcase
// condition selection register
if (MPUWSTB[0] & MPUDOUTBus[11]) CondSelReg<=MPUDOUTBus[10:8];

// External IRQ processing
INTREQFlag<=(INTREQFlag | INTREQ) & ~INTACKFlag;
INTACKFlag<=INTACK;

end
end

endmodule: Messenger
