module Flash #(parameter Code=0) (
input wire CLKH, RESET,
// internal interface
output wire NEXT,
input wire ACT, CMD,
input wire [31:0] ADDR,
input wire [7:0] BE,
input wire [63:0] DTI,
input wire [20:0] TAGI,
output reg DRDY,
output reg [63:0] DTO,
output reg [20:0] TAGO,
// interface to the external memory
output reg FLCK,
output reg FLRST,
output reg FLCS,
output wire FLSO,
input wire FLSI
);

int i0, i1, i2;

//=================================================================================================
// Address space 
// 3FFFFFFFh - 20000000h	- SPI data read interface
// 01000000h				- SPI control register
// 00007FFFh - 00000000h 	- internal memory buffer RAM-BIOS, last 1024 bytes is a play buffer for SPI

// internal memory buffers for RAM-BIOS
reg [7:0] IntMemA [0:4095];
reg [7:0] IntMemB [0:4095];
reg [7:0] IntMemC [0:4095];
reg [7:0] IntMemD [0:4095];
reg [7:0] IntMemE [0:4095];
reg [7:0] IntMemF [0:4095];
reg [7:0] IntMemG [0:4095];
reg [7:0] IntMemH [0:4095];
reg [11:0] IntAReg;
reg [63:0] IntDReg;
reg [7:0] IntWEReg;

// shift registers and counters
reg [7:0] OutShiftReg;
reg [6:0] InShiftReg;
reg [2:0] InBitReg, OutBitReg;
reg [63:0] ShiDataReg;
reg [55:0] ShoDataReg;

// sequence registers
reg [20:0] InTagReg;
reg InDRDYReg;
reg [9:0] SCntReg, SendReg;
reg [7:0] InstReg;
reg [2:0] TLengthReg;
reg [2:0] BCnt, ShCaseReg;

// input registers
reg [31:0] InAddrReg;
wire [7:0] ReadCodeBus;

logic [7:0] ShoDataBus;

// state machine
reg [3:0] SPIMachine, StackReg;
parameter Idle=4'd0, PreRead=4'd1, SetRead=4'd2, SetAddrHigh=4'd3, SetAddrMedium=4'd4, SetAddrLow=4'd5, SetDummy=4'd6, WaitByte=4'd7, ReadQword=4'd8,
			OutData=4'd9, PrepareRead=4'd10, PreCmd=4'd11, ReadCmd=4'd12, TransmitCmd=4'd13, GetData=4'd14;

reg StrobeCmdFlag, WriteCmdFlag, DelayFlag, FLSIReg, LastBitFlag;

reg [5:0] BitCntReg;

initial begin
if (Code==1) begin
	$readmemh("MemoryFile10", IntMemA, 0, 4095);
	$readmemh("MemoryFile11", IntMemB, 0, 4095);
	$readmemh("MemoryFile12", IntMemC, 0, 4095);
	$readmemh("MemoryFile13", IntMemD, 0, 4095);
	$readmemh("MemoryFile14", IntMemE, 0, 4095);
	$readmemh("MemoryFile15", IntMemF, 0, 4095);
	$readmemh("MemoryFile16", IntMemG, 0, 4095);
	$readmemh("MemoryFile17", IntMemH, 0, 4095);
	end
else if (Code==2) begin
	$readmemh("MemoryFile20", IntMemA, 0, 4095);
	$readmemh("MemoryFile21", IntMemB, 0, 4095);
	$readmemh("MemoryFile22", IntMemC, 0, 4095);
	$readmemh("MemoryFile23", IntMemD, 0, 4095);
	$readmemh("MemoryFile24", IntMemE, 0, 4095);
	$readmemh("MemoryFile25", IntMemF, 0, 4095);
	$readmemh("MemoryFile26", IntMemG, 0, 4095);
	$readmemh("MemoryFile27", IntMemH, 0, 4095);
	end
else if (Code==3) begin
	$readmemh("MemoryFile30", IntMemA, 0, 4095);
	$readmemh("MemoryFile31", IntMemB, 0, 4095);
	$readmemh("MemoryFile32", IntMemC, 0, 4095);
	$readmemh("MemoryFile33", IntMemD, 0, 4095);
	$readmemh("MemoryFile34", IntMemE, 0, 4095);
	$readmemh("MemoryFile35", IntMemF, 0, 4095);
	$readmemh("MemoryFile36", IntMemG, 0, 4095);
	$readmemh("MemoryFile37", IntMemH, 0, 4095);
	end
else if (Code==4) begin
	$readmemh("MemoryFile40", IntMemA, 0, 4095);
	$readmemh("MemoryFile41", IntMemB, 0, 4095);
	$readmemh("MemoryFile42", IntMemC, 0, 4095);
	$readmemh("MemoryFile43", IntMemD, 0, 4095);
	$readmemh("MemoryFile44", IntMemE, 0, 4095);
	$readmemh("MemoryFile45", IntMemF, 0, 4095);
	$readmemh("MemoryFile46", IntMemG, 0, 4095);
	$readmemh("MemoryFile47", IntMemH, 0, 4095);
	end
else if (Code==5) begin
	$readmemh("MemoryFile50", IntMemA, 0, 4095);
	$readmemh("MemoryFile51", IntMemB, 0, 4095);
	$readmemh("MemoryFile52", IntMemC, 0, 4095);
	$readmemh("MemoryFile53", IntMemD, 0, 4095);
	$readmemh("MemoryFile54", IntMemE, 0, 4095);
	$readmemh("MemoryFile55", IntMemF, 0, 4095);
	$readmemh("MemoryFile56", IntMemG, 0, 4095);
	$readmemh("MemoryFile57", IntMemH, 0, 4095);
	end
else begin
$readmemh("MemoryFile0", IntMemA, 0, 4095);
$readmemh("MemoryFile1", IntMemB, 0, 4095);
$readmemh("MemoryFile2", IntMemC, 0, 4095);
$readmemh("MemoryFile3", IntMemD, 0, 4095);
$readmemh("MemoryFile4", IntMemE, 0, 4095);
$readmemh("MemoryFile5", IntMemF, 0, 4095);
$readmemh("MemoryFile6", IntMemG, 0, 4095);
$readmemh("MemoryFile7", IntMemH, 0, 4095);
end
end


assign NEXT=(SPIMachine==Idle) & ~DelayFlag;
assign FLSO=OutShiftReg[7];
assign ReadCodeBus=InstReg;

//=================================================================================================
//				Asynchronous part
//=================================================================================================
always_comb
begin

if (WriteCmdFlag) ShoDataBus=DTO[7:0];
	else case (BitCntReg[5:3])
	3'd0: ShoDataBus=DTO[7:0];
	3'd1: ShoDataBus=ShoDataReg[7:0];
	3'd2: ShoDataBus=ShoDataReg[15:8];
	3'd3: ShoDataBus=ShoDataReg[23:16];
	3'd4: ShoDataBus=ShoDataReg[31:24];
	3'd5: ShoDataBus=ShoDataReg[39:32];
	3'd6: ShoDataBus=ShoDataReg[47:40];
	3'd7: ShoDataBus=ShoDataReg[55:48];
	endcase
	
end

//=================================================================================================
//				Synchronous part
//=================================================================================================
always_ff @(posedge CLKH)
begin

// internal address register for internal memory
if (SPIMachine==ReadCmd) IntAReg<={5'd31, SCntReg[9:3]};
	else IntAReg<=ADDR[14:3];
IntDReg<=DTI;
for (i0=0; i0<8; i0=i0+1) IntWEReg[i0]<=(~BE[i0] & ~CMD & ACT & NEXT & ~ADDR[24]);
// write to memory
if (IntWEReg[0]) IntMemA[IntAReg]<=IntDReg[7:0];
if (IntWEReg[1]) IntMemB[IntAReg]<=IntDReg[15:8];
if (IntWEReg[2]) IntMemC[IntAReg]<=IntDReg[23:16];
if (IntWEReg[3]) IntMemD[IntAReg]<=IntDReg[31:24];
if (IntWEReg[4]) IntMemE[IntAReg]<=IntDReg[39:32];
if (IntWEReg[5]) IntMemF[IntAReg]<=IntDReg[47:40];
if (IntWEReg[6]) IntMemG[IntAReg]<=IntDReg[55:48];
if (IntWEReg[7]) IntMemH[IntAReg]<=IntDReg[63:56];

// read from memory
// data out
DTO[7:0]<=InAddrReg[29] ? ShiDataReg[7:0] : IntMemA[IntAReg];
DTO[15:8]<=InAddrReg[29] ? ShiDataReg[15:8] : IntMemB[IntAReg];
DTO[23:16]<=InAddrReg[29] ? ShiDataReg[23:16] : IntMemC[IntAReg];
DTO[31:24]<=InAddrReg[29] ? ShiDataReg[31:24] : IntMemD[IntAReg];
DTO[39:32]<=InAddrReg[29] ? ShiDataReg[39:32] : IntMemE[IntAReg];
DTO[47:40]<=InAddrReg[29] ? ShiDataReg[47:40] : IntMemF[IntAReg];
DTO[55:48]<=InAddrReg[29] ? ShiDataReg[55:48] : IntMemG[IntAReg];
DTO[63:56]<=InAddrReg[29] ? ShiDataReg[63:56] : IntMemH[IntAReg];

if (SPIMachine==Idle)
	begin
	InTagReg<=TAGI;
	InAddrReg<=ADDR;
	end
	
// tag output
TAGO<=InTagReg;

// data ready signal
InDRDYReg<=ACT & CMD & RESET & NEXT;
DRDY<=((InDRDYReg & ~InAddrReg[29] & (SPIMachine==Idle)) | (SPIMachine==GetData)) & RESET;

// data counters for command sequences
if (ACT & ~CMD & ~ADDR[29] & ADDR[24] & ~BE[0] & NEXT) SendReg[7:0]<=DTI[7:0];
if (ACT & ~CMD & ~ADDR[29] & ADDR[24] & ~BE[1] & NEXT) SendReg[9:8]<=DTI[9:8];
if (ACT & ~CMD & ~ADDR[29] & ADDR[24] & ~BE[2] & NEXT) InstReg<=DTI[23:16];

if (SPIMachine==Idle) SCntReg<=10'd0;
	else SCntReg<=SCntReg+{9'd0, (BitCntReg[2:0]==3'd6) & FLCK & ~FLCS};
	
// transaction length register
if ((SPIMachine==Idle) & ACT & CMD & ADDR[29]) TLengthReg<={TAGI[17] & TAGI[16], TAGI[17], TAGI[17] | TAGI[16]};

//---------------------------------------------------------
//	state machine wthat controls SPI transfers
if (~RESET) SPIMachine<=Idle;
	else case (SPIMachine)
			Idle: if (ACT & ((CMD & ADDR[29]) | (~CMD & ADDR[24] & ~BE[1])) & ~DelayFlag)
						begin
						case ({ADDR[29], FLCK})
							2'b00: SPIMachine<=ReadCmd;
							2'b01: SPIMachine<=PreCmd;
							2'b10: SPIMachine<=SetRead;
							2'b11: SPIMachine<=PreRead;
							endcase
						end
					else SPIMachine<=Idle;
			PreRead: SPIMachine<=SetRead;
			SetRead: SPIMachine<=WaitByte;
			SetAddrHigh: SPIMachine<=WaitByte;
			SetAddrMedium: SPIMachine<=WaitByte;
			SetAddrLow: SPIMachine<=WaitByte;
			SetDummy: SPIMachine<=WaitByte;
			PrepareRead: SPIMachine<=ReadQword;
			ReadQword: if (&BitCntReg[2:0] & ~FLCK & (BitCntReg[5:3]==TLengthReg)) SPIMachine<=OutData;
							else SPIMachine<=ReadQword;
			WaitByte: if ((BitCntReg[2:0]==3'd7)& ~FLCK) SPIMachine<=StackReg;
						else SPIMachine<=WaitByte;
			OutData: SPIMachine<=GetData;
			GetData: SPIMachine<=Idle;
			// reading sequence qword from memory
			PreCmd: SPIMachine<=ReadCmd;
			ReadCmd: SPIMachine<=TransmitCmd;
			TransmitCmd: if ((BitCntReg[2:0]==3'd7) & ~FLCK & (SCntReg==SendReg)) SPIMachine<=Idle;
							else if ((BitCntReg == 6'h3E) & ~FLCK & (SCntReg!=SendReg)) SPIMachine<=ReadCmd;
									else SPIMachine<=TransmitCmd;
			endcase
			
// Delay transaction
if (FLCK | ~RESET) DelayFlag<=(SPIMachine==OutData) & RESET;

// strobe flags
StrobeCmdFlag<=(SPIMachine==ReadCmd);
WriteCmdFlag<=StrobeCmdFlag;

// output chip select signal
FLCS<=(FLCS & (SPIMachine!=SetRead) & ~WriteCmdFlag) | (SPIMachine==Idle) | (SPIMachine==OutData);

// output reset
FLRST<=RESET;

// creating Flash clock
FLCK<=~FLCK & RESET;

//---------------------------------------------------------
//	machine stack register
// 05/35/15 - only instruction
// 03/AB/90 - instruction + address
// 0B/4B/5A/48 - instruction + address + dummy
case (SPIMachine)
	SetRead: if ((InstReg==8'h05)|(InstReg==8'h35)|(InstReg==8'h15)) StackReg<=PrepareRead;
				else StackReg<=SetAddrHigh;
	SetAddrHigh: StackReg<=SetAddrMedium;
	SetAddrMedium: StackReg<=SetAddrLow;
	SetAddrLow: if ((InstReg==8'h03)|(InstReg==8'hAB)|(InstReg==8'h90)) StackReg<=PrepareRead;
					else StackReg<=SetDummy;
	SetDummy: StackReg<=PrepareRead;
	default: StackReg<=StackReg;
	endcase

// bit counter
if ((SPIMachine==SetRead) | WriteCmdFlag | ((BitCntReg[2:0]==3'd7) & ~FLCK & (SCntReg==SendReg))) BitCntReg<=6'd0;
 else if (~FLCK) BitCntReg<=(BitCntReg+6'd1) & {{3{(SPIMachine==ReadQword) | (SPIMachine==TransmitCmd)}},{3{SPIMachine!=Idle}}};
 
// data register for transfered parameters
if (WriteCmdFlag)
	begin
	ShoDataReg<=DTO[63:8];
	end

//---------------------------------------------------------
//	output shift register
for (i1=0; i1<8; i1=i1+1)
	if (FLCK)
		begin
		if (WriteCmdFlag | ((BitCntReg[2:0]==3'd0) & (SPIMachine==TransmitCmd))) OutShiftReg[i1]<=ShoDataBus[i1];
			else if (SPIMachine==SetRead) OutShiftReg[i1]<=ReadCodeBus[i1];
				else if (SPIMachine==SetAddrHigh) OutShiftReg[i1]<=InAddrReg[i1+16];
					else if (SPIMachine==SetAddrMedium) OutShiftReg[i1]<=InAddrReg[i1+8];
						else if (SPIMachine==SetAddrLow) OutShiftReg[i1]<=InAddrReg[i1];
							else if (i1==0) OutShiftReg[i1]<=(SPIMachine==SetDummy);
								else OutShiftReg[i1]<=OutShiftReg[i1-1] | (SPIMachine==SetDummy);
		end


//---------------------------------------------------------
//	input shift register
FLSIReg<=FLSI;

if (FLCK) for (i2=0; i2<7; i2=i2+1)
	if (i2==0) InShiftReg[i2]<=FLSIReg;
		else InShiftReg[i2]<=InShiftReg[i2-1];
		
// input data register
//if (BitCntReg[2:0]==3'd7)
//	case (BitCntReg[5:3]+InAddrReg[2:0])
LastBitFlag<=&BitCntReg[2:0];
ShCaseReg<=BitCntReg[5:3]+InAddrReg[2:0];

if (LastBitFlag)
	case (ShCaseReg)
	3'd0: ShiDataReg[7:0]<={InShiftReg, FLSIReg};
	3'd1: ShiDataReg[15:8]<={InShiftReg, FLSIReg};
	3'd2: ShiDataReg[23:16]<={InShiftReg, FLSIReg};
	3'd3: ShiDataReg[31:24]<={InShiftReg, FLSIReg};
	3'd4: ShiDataReg[39:32]<={InShiftReg, FLSIReg};
	3'd5: ShiDataReg[47:40]<={InShiftReg, FLSIReg};
	3'd6: ShiDataReg[55:48]<={InShiftReg, FLSIReg};
	3'd7: ShiDataReg[63:56]<={InShiftReg, FLSIReg};
	endcase

end

endmodule
