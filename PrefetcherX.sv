module PrefetcherX #(parameter Mode=0)(
	input wire CLK, RESET, CSELCHG,
	// data bus from memory subsystem
	input wire DRDY,
	input wire [7:0] TAGi,
	input wire [63:0] DTi,
	// instruction bus to sequencer
	input wire IFETCH,
	output wire IRDY,
	output wire [3:0] VF,
	output wire [63:0] IBUS,
	// interface to address translation system
	input wire NEXT,
	output reg ACT,
	output wire [36:0] OFFSET,
	output wire [2:0] TAGo,
	// command interface
	input wire [16:0] CBUS,
	input wire [63:0] GPRBUS,
	input wire CC, MOVFLAG,
	// interface to message and memory control system
	output reg [3:0] GPINDX,
	output reg SMSG, MALLOC, GPAR, EMSG, BKPT, SLEEP,
	output reg [63:0] PARAM,
	output wire CALL, RET, ERST, EMPTY, HALT, SMSGBKPT,
	input wire CEXEC, ESMSG, CONTINUE,
	input wire STOP,
	output wire [35:0] RIP,
	// error reporting
	output reg CFERROR,
	// fatal error report (test purpose)
	output reg FATAL
	);

integer i,i0,i1;

// instruction RAM and his resources
//reg [64:0] RAM [0:4095];
//reg [21:0] ARAM [0:255];
reg [255:0] VFlags;
reg [64:0] RamReg;
reg [35:0] PrefIP, RealIP, RamIP;			// IP registers
reg [7:0] TMOReg;							// timeout register
reg [3:0] LoadCntr, DataCntr;				// reload counter
reg [29:0] LoadPage;
reg [3:0] VFReg, LockWire, LockReg;
reg [16:0] CBUSReg;
reg [35:0] JumpBus;
reg [15:0] LongDispReg;

reg RamValidFlag, LoadPageFlag, LockFlag, HaltFlag, DelayedCEXECFlag, CFErrorStateFlag, DRDYReg, OutRegFull, JccTrueInstFlag,
	JccFalseInstFlag, JNearInstFlag, LoopInstFlag, JumpInstFlag, CallFlag, CallInstFlag, RetInstFlag, RetConditionFlag,
	CEXECFlag, CFErrorFlag, DNearJumpFlag, ContextCEXECFlag, SMSGFlag, RetFlag, EndLoadFlag, LongJCFlag, ValidWire, EmptyFlag, IPLoadFlag,
	ContinueFlag, MemBusyFlag;

logic RestartWire, RamEnaWire;

logic [64:0] DBus;

logic [21:0] ABus;
/*
 bus format
63:0	Instructions
65:64		lowest two bits of the instruction pointer
66		fetch error flag
*/
logic [1:0] LongJC;

logic NearJumpFlag, StartLoadFlag;
	
altsyncram	ARAM (.addressstall_b(~RamEnaWire),
				.wren_a(LoadPageFlag & ~(|LoadCntr)), .clock0(CLK), .byteena_a(3'h7),
				.address_a(LoadPage[7:0]), .address_b(PrefIP[13:6]), .data_a(LoadPage[29:8]), .q_b (ABus), .aclr0 (1'b0), .aclr1 (1'b0),
				.addressstall_a (1'b0), .byteena_b (1'b1), .clock1 (1'b1), .clocken0 (1'b1), .clocken1 (1'b1), .clocken2 (1'b1),
				.clocken3 (1'b1), .data_b ({22{1'b1}}), .eccstatus (), .q_a (), .rden_a (1'b1), .rden_b (1'b1), .wren_b (1'b0));
	defparam
		ARAM.address_aclr_b = "NONE",
		ARAM.address_reg_b = "CLOCK0",
		ARAM.byte_size = 8,
		ARAM.clock_enable_input_a = "BYPASS",
		ARAM.clock_enable_input_b = "BYPASS",
		ARAM.clock_enable_output_b = "BYPASS",
		ARAM.lpm_type = "altsyncram",
		ARAM.numwords_a = 256,
		ARAM.numwords_b = 256,
		ARAM.operation_mode = "DUAL_PORT",
		ARAM.outdata_aclr_b = "NONE",
		ARAM.outdata_reg_b = "UNREGISTERED",
		ARAM.power_up_uninitialized = "FALSE",
		ARAM.read_during_write_mode_mixed_ports = "DONT_CARE",
		ARAM.widthad_a = 8,
		ARAM.widthad_b = 8,
		ARAM.width_a = 22,
		ARAM.width_b = 22;
	
altsyncram	DRAM (.addressstall_b(~RamEnaWire),
				.wren_a(DRDYReg), .clock0(CLK), .byteena_a(9'h1FF), .address_a({LoadPage[7:0],DataCntr}), .address_b(PrefIP[13:2]),
				.data_a({CFErrorFlag, DTi}), .q_b (DBus), .aclr0 (1'b0), .aclr1 (1'b0),
				.addressstall_a (1'b0), .byteena_b (1'b1), .clock1 (1'b1), .clocken0 (1'b1), .clocken1 (1'b1), .clocken2 (1'b1),
				.clocken3 (1'b1), .data_b ({22{1'b1}}), .eccstatus (), .q_a (), .rden_a (1'b1), .rden_b (1'b1), .wren_b (1'b0));
	defparam
		DRAM.address_aclr_b = "NONE",
		DRAM.address_reg_b = "CLOCK0",
		DRAM.byte_size = 8,
		DRAM.clock_enable_input_a = "BYPASS",
		DRAM.clock_enable_input_b = "BYPASS",
		DRAM.clock_enable_output_b = "BYPASS",
		DRAM.lpm_type = "altsyncram",
		DRAM.numwords_a = 4096,
		DRAM.numwords_b = 4096,
		DRAM.operation_mode = "DUAL_PORT",
		DRAM.outdata_aclr_b = "NONE",
		DRAM.outdata_reg_b = "UNREGISTERED",
		DRAM.power_up_uninitialized = "FALSE",
		DRAM.read_during_write_mode_mixed_ports = "DONT_CARE",
		DRAM.widthad_a = 12,
		DRAM.widthad_b = 12,
		DRAM.width_a = 65,
		DRAM.width_b = 65;

assign RIP=RealIP;
assign CALL=CallInstFlag;
assign RET=RetInstFlag;
assign ERST=DNearJumpFlag;
assign OFFSET[2:0]=3'd0;
// empty flag
assign EMPTY=(((RealIP==PrefIP) | CFErrorStateFlag) & ~RamValidFlag & (~(~LockFlag & ~HaltFlag)) & ~DNearJumpFlag & HaltFlag) | EmptyFlag;
assign HALT=HaltFlag;
assign SMSGBKPT=SMSGFlag;
assign OFFSET={LoadPage, LoadCntr, 3'd0};
assign TAGo={&LoadCntr,2'd0};
assign IBUS=RamReg[63:0];
assign IRDY=RamValidFlag;
assign VF[0]=VFReg[0];
assign VF[1]=VFReg[1] & ~LockReg[0];
assign VF[2]=VFReg[2] & ~LockReg[1] & ~LockReg[0];
assign VF[3]=VFReg[3] & ~LockReg[2] & ~LockReg[1] & ~LockReg[0];

/*
===================================================================================================
		Asynchronous part
===================================================================================================
*/
always @*
	begin
	
	// reading address ram
	//ABus=ARAM[PrefIP[13:6]];

	// reading data ram
	//DBus=RAM[PrefIP[13:2]];

	// instruction valid in the register
	//ValidWire=(RamValidFlag & ~IFETCH) | (MemBusyFlag & (ABus==RamIP[35:14]) & ~LockFlag & ~HaltFlag) |
										//(ContinueFlag & (|LockReg[2:0]) & MemBusyFlag & (ABus==RamIP[35:14]));
			//((~LockFlag | (DContinueFlag & LockReg[3])) & ~HaltFlag & ValidFlag & (ABus==RamIP[35:14]));
	ValidWire=(RamValidFlag & ~IFETCH) | (ContinueFlag & (|LockReg[2:0])) |
			((~LockFlag | (ContinueFlag & LockReg[3])) & MemBusyFlag & (ABus==RamIP[35:14]));
			
	// enable transaction on the RAM stage
	RamEnaWire=~MemBusyFlag | ((~RamValidFlag | IFETCH) & ~LockFlag) | (ContinueFlag & ~LockReg[2] & ~LockReg[1] & ~LockReg[0]);

	// instruction lock nodes
	LockWire[0]=(((DBus[7:0]==8'hF0) & (((&DBus[10:8]) & ~(&DBus[15:8]))|(DBus[11:8]==4'd6)|(DBus[11:9]==3'd5)|(DBus[11:8]==4'hC)))|
				(DBus[6:0]==7'h5C)|(~DBus[3] & ~DBus[2] & (DBus[1] ^ DBus[0]))) & ~DBus[64] & ~RamIP[1] & ~RamIP[0]; 
	LockWire[1]=(((DBus[23:16]==8'hF0) & (((&DBus[26:24]) & ~(&DBus[31:24]))|(DBus[27:24]==4'd6)|(DBus[27:25]==3'd5)|(DBus[27:24]==4'hC)))|
				(DBus[22:16]==7'h5C)|(~DBus[19] & ~DBus[18] & (DBus[17] ^ DBus[16]))) & ~DBus[64] & ~RamIP[1] &
				~({DBus[15:12],DBus[3:0]}==8'h12); 
	LockWire[2]=(((DBus[39:32]==8'hF0) & (((&DBus[42:40]) & ~(&DBus[47:40]))|(DBus[43:40]==4'd6)|(DBus[43:41]==3'd5)|(DBus[43:40]==4'hC)))|
				(DBus[38:32]==7'h5C)|(~DBus[35] & ~DBus[34] & (DBus[33] ^ DBus[32]))) & ~DBus[64] & (~RamIP[1] | ~RamIP[0]); 
	LockWire[3]=(((DBus[55:48]==8'hF0) & (((&DBus[58:56]) & ~(&DBus[63:56]))|(DBus[59:56]==4'd6)|(DBus[59:57]==3'd5)|(DBus[59:56]==4'hC)))|
				(DBus[54:48]==7'h5C)|(~DBus[51] & ~DBus[50] & (DBus[49] ^ DBus[48]))) & ~DBus[64] &
				~({DBus[47:44],DBus[35:32]}==8'h12); 
	
	// generation request to load page
	StartLoadFlag=(~VFlags[PrefIP[13:6]] | (ABus!=RamIP[35:14])) & ~HaltFlag;
						
	RestartWire=(|LockReg[2:0]) & ContinueFlag;
	
	// decoding long JC instruction
	LongJC[0]=VF[0] & (IBUS[3:0]==4'h2) & (IBUS[15:12]==4'h1);
	LongJC[1]=VF[2] & (IBUS[35:32]==4'h2) & (IBUS[47:44]==4'h1);
	
	end

/*
===================================================================================================
		Synchronous part
===================================================================================================
*/
always @(negedge RESET or posedge CLK)
	if (!RESET) begin
				VFReg<=0;
				LockReg<=0;
				ACT<=0;
				SMSG<=0;
				MALLOC<=0;
				GPAR<=0;
				EMSG<=0;
				BKPT<=0;
				SLEEP<=0;
				CFERROR<=0;
				RamValidFlag<=0;
				LoadPageFlag<=0;
				DRDYReg<=0;
				PrefIP<=0;
				RealIP<=0;
				RamIP<=36'hFFFFFFFFF;
				TMOReg<=0;
				LoadCntr<=0;
				DataCntr<=0;
				LockFlag<=0;
				if (Mode==0) HaltFlag<=0;
					else HaltFlag<=1'b1;
				DelayedCEXECFlag<=0;
				CFErrorStateFlag<=0;
				JccTrueInstFlag<=0;
				JccFalseInstFlag<=0;
				JNearInstFlag<=0;
				LoopInstFlag<=0;
				JumpInstFlag<=0;
				CallFlag<=0;
				CallInstFlag<=0;
				RetInstFlag<=0;
				RetConditionFlag<=0;
				CEXECFlag<=0;
				CFErrorFlag<=0;
				DNearJumpFlag<=0;
				ContextCEXECFlag<=0;
				SMSGFlag<=0;
				RetFlag<=0;
				VFlags<=0;
				EndLoadFlag<=0;
				LongJCFlag<=0;
				ContinueFlag<=0;
				NearJumpFlag<=0;
				EmptyFlag<=0;
				IPLoadFlag<=0;
				MemBusyFlag<=0;
				end
	else begin
	
	// memory stage busy flag
	MemBusyFlag<=((MemBusyFlag & ((RamValidFlag & ~IFETCH) | LockFlag) & (~ContinueFlag | (|LockReg[2:0]))) | 
				((~LockFlag | (ContinueFlag & LockReg[3])) & (VFlags[PrefIP[13:6]] | ((PrefIP[5:2]<DataCntr)&(LoadPage[7:0]==PrefIP[13:6]))) & ~HaltFlag)) &
				~NearJumpFlag;
	
	// Long JC instruction flag
	LongJCFlag<=(LongJCFlag | ((|LongJC) & IRDY & IFETCH)) & ~NearJumpFlag & ~ContinueFlag;
	
	// long offset muxing
	if ((|LongJC) & IRDY & IFETCH) LongDispReg<=(IBUS[31:16] & {16{LongJC[0]}})|(IBUS[63:48] & {16{LongJC[1]}});
	
	// valid flags 
	if (((~IRDY | IFETCH) & ~LockFlag) | ContinueFlag | NearJumpFlag)
		begin
		VFReg[0]<=(RestartWire ? 1'b0 : ~DBus[64] & ~RamIP[1] & ~RamIP[0]) & ~NearJumpFlag;
		VFReg[1]<=(RestartWire ? VFReg[1] & (VFReg[0] & LockReg[0]) : ~DBus[64] & ~RamIP[1] & ~((DBus[3:0]==4'h2) & (DBus[15:12]==4'h1))) & ~NearJumpFlag;
		VFReg[2]<=(RestartWire ? VFReg[2] & ((VFReg[0] & LockReg[0])|(VFReg[1] & LockReg[1])) : ~DBus[64] & (~RamIP[1] | ~RamIP[0])) & ~NearJumpFlag;
		VFReg[3]<=(RestartWire ? VFReg[3] & ((VFReg[0] & LockReg[0])|(VFReg[1] & LockReg[1])|(VFReg[2] & LockReg[2])) : ~DBus[64] & ~((DBus[35:32]==4'h2) & (DBus[47:44]==4'h1))) & ~NearJumpFlag;
	
		// instruction lock nodes 
		LockReg[0]<=(RestartWire ? 1'b0 : LockWire[0]) & ~NearJumpFlag; 
		LockReg[1]<=(RestartWire ? LockReg[1] & LockReg[0] : LockWire[1]) & ~NearJumpFlag; 
		LockReg[2]<=(RestartWire ? LockReg[2] & (LockReg[1] | LockReg[0]) : LockWire[2]) & ~NearJumpFlag; 
		LockReg[3]<=(RestartWire ? LockReg[3] & (LockReg[2] | LockReg[1] | LockReg[0]) : LockWire[3]) & ~NearJumpFlag; 
		end
	
	// send message state flag
	EmptyFlag<=(EmptyFlag | (CBUS[16:0]==17'h7FE1) | (CBUS[16:0]==17'h5FE1) | (CBUS[12:0]==13'h15E1)) & ~CONTINUE & ~ESMSG & ~CEXEC & ~IPLoadFlag;
	SMSGFlag<=(SMSGFlag | (CBUS[16:0]==17'h5FE1) | (CBUS[12:0]==13'h15E1) | (CBUS[16:0]==17'h3FE1) | (CBUS[12:0]==13'h0DE1) | (CBUS[12:0]==13'h0FE1)) &
				~CONTINUE & ~ESMSG & ~CEXEC & ~IPLoadFlag;

	// transaction timeout
	TMOReg<=(TMOReg+8'd1) & ~HaltFlag & (~IRDY | ~IFETCH);
	// generate FATAL error signal if instruction timeout expired
	FATAL<=&TMOReg;
	
	// instruction register and data from GPR
	if ((~CallFlag)&(~RetFlag)) CBUSReg<=CBUS;
	if (CBUS[0]) PARAM<=GPRBUS;
	
	// data ready to the instruction cache
	DRDYReg<=DRDY & ~TAGi[7] & TAGi[6] & TAGi[5] & ~TAGi[3];
	EndLoadFlag<=DRDY & ~TAGi[7] & TAGi[6] & TAGi[5] & ~TAGi[3] & TAGi[2];
	
	// Code fetch error flag
	CFErrorFlag<=TAGi[4];
	
	// Code fetch error reporting
	CFERROR<=~CFErrorStateFlag & RamValidFlag & RamReg[64] & (~LockFlag | ContinueFlag);
	CFErrorStateFlag<=(CFErrorStateFlag | CFERROR) & ~CEXEC & ~IPLoadFlag;
	
	// return condition
	RetConditionFlag<=(DRDY & ~TAGi[7] & TAGi[6] & TAGi[5] & ~TAGi[4] & TAGi[3]) | CEXEC | ESMSG;
	CEXECFlag<=CEXEC | ESMSG;
	
	// restart core by write IP from context controller
	IPLoadFlag<=DRDY & ~TAGi[7] & TAGi[6] & TAGi[5] & ~TAGi[4] & TAGi[3] & TAGi[0];

	ContextCEXECFlag<=CEXEC;
	
	DelayedCEXECFlag<=ContextCEXECFlag | IPLoadFlag;
	
	// setting the LockFlag
	if (~IRDY | IFETCH | ContinueFlag | NearJumpFlag) LockFlag<=ContinueFlag ? (~VF[3] & LockReg[3])|(~VF[2] & LockReg[2])|(~VF[1] & LockReg[1])|
									((|LockWire) & ValidWire & ~(|LockReg[2:0])) : (LockFlag | ((|LockWire) & ValidWire)) & ~NearJumpFlag;

	// stop execution flag
	HaltFlag<=(HaltFlag | STOP) & ~DelayedCEXECFlag;
	
	//===================================================================================================================================
	// reading data from instruction memory
	
	// processing pre-load program counter
	if (NearJumpFlag) PrefIP<=JumpBus;
			else if ((((~MemBusyFlag | ~RamValidFlag | IFETCH) & ~LockFlag) | (ContinueFlag & ~LockReg[2] & ~LockReg[1] & ~LockReg[0])) &
						~HaltFlag & ((VFlags[PrefIP[13:6]] & (ABus==RamIP[35:14])) | ((PrefIP[5:2]<DataCntr)&(LoadPage[7:0]==PrefIP[13:6])))) PrefIP<={PrefIP[35:2]+34'd1, 2'd0};

	if (((~RamValidFlag | IFETCH) & ~LockFlag) | (ContinueFlag & ~LockReg[2] & ~LockReg[1] & ~LockReg[0])) RamReg<=DBus;
	
	if (RamEnaWire) RamIP<=PrefIP;
		
	// creating ram valid flag
	RamValidFlag<=ValidWire;
						
	// forming jump address
	if (JccTrueInstFlag) JumpBus<=RealIP+(LongJCFlag ? {{20{LongDispReg[15]}}, LongDispReg} : {{32{CBUSReg[16]}}, CBUSReg[16:13]}) + {{35{1'b1}},~LongJCFlag};
			else if (JNearInstFlag) JumpBus<=RealIP+{{27{CBUSReg[16]}}, CBUSReg[16:8]} + {36{1'b1}};
					else if (LoopInstFlag) JumpBus<=RealIP+{{28{1'b1}},CBUSReg[12:5]} + {36{1'b1}};
							else if (JumpInstFlag | CallInstFlag) JumpBus<=PARAM[36:1];
									else if (RetConditionFlag) JumpBus<=CEXECFlag ? RealIP : DTi[36:1];
					
	
	//===================================================================================================================================
		
	
	// real program counter
	if (NearJumpFlag) RealIP<=JumpBus;
			else if (IFETCH & IRDY) RealIP<=RealIP+{35'd0, VF[0]}+{35'd0, VF[1]}+{35'd0, VF[2]}+{35'd0, VF[3]}+{35'd0, |LongJC};
	
	// delayed jump flag
	DNearJumpFlag<=NearJumpFlag;

	// Jcc instruction when condition true
	JccTrueInstFlag<=(CBUS[4:0]==5'b00101) & CC;
	// Jcc instruction when condition false
	JccFalseInstFlag<=((CBUS[4:0]==5'b00101) & ~CC)|((CBUS[4:0]==5'b00011) & ~(|GPRBUS[31:1]) & GPRBUS[0]);
	// JNEAR instruction flag
	JNearInstFlag<=(CBUS[7:0]==8'hB9);
	// LOOP Instruction flag
	LoopInstFlag<=(CBUS[4:0]==5'b00011) & ((|GPRBUS[31:1]) | ~GPRBUS[0]);
	// JUMP instruction flag
	JumpInstFlag<=(CBUS[12:0]==13'h17E1);
	// CALL instruction flag
	CallInstFlag<=CallFlag & ~MOVFLAG;
	CallFlag<=(CallFlag & MOVFLAG) | (CBUS[12:0]==13'h19E1);
	// RET instruction flag
	RetInstFlag<=RetFlag & ~MOVFLAG;
	RetFlag<=(RetFlag & MOVFLAG) | (CBUS[16:0]==17'h1FE1);
	// flag for ALL near jump's
	NearJumpFlag<=JccTrueInstFlag | JNearInstFlag | LoopInstFlag | JumpInstFlag | CallInstFlag | RetConditionFlag;
	ContinueFlag<=JccFalseInstFlag | CONTINUE;
	
	//
	// Control signals to high-level instructions
	//
	// register index
	GPINDX<=CBUS[16:13];
	// send message request
	SMSG<=(CBUS[12:0]==13'h15E1);
	// memory allocation request
	MALLOC<=(CBUS[12:0]==13'h0DE1);
	// get message params request
	GPAR<=(CBUS[12:0]==13'h0FE1);
	// end message processign
	EMSG<=(CBUS[16:0]==17'h3FE1);
	// breakpoint request
	BKPT<=(CBUS[16:0]==17'h5FE1);
	// sleep request
	SLEEP<=(CBUS[16:0]==17'h7FE1);
	
	
	//=============================================================================================================
	//				ram preload system
	
	// valid flags
	for (i=0; i<256; i=i+1)
		if (CSELCHG) VFlags[i]<=0;
			else if (i==LoadPage[7:0]) VFlags[i]<=VFlags[i] | EndLoadFlag;
	
	// reload line counter
	LoadCntr<=LoadCntr+{3'd0,ACT & NEXT};
	
	// page loading
	LoadPageFlag<=(LoadPageFlag | StartLoadFlag) & ~EndLoadFlag;
	
	// page register
	if (~LoadPageFlag) LoadPage<=PrefIP[35:6];
	
	// transaction generation
	ACT<=(ACT & ~(NEXT & (&LoadCntr))) | (StartLoadFlag & ~LoadPageFlag);
	
	// write to the address ram
	//if (LoadPageFlag & ~(|LoadCntr)) ARAM[LoadPage[7:0]]<=LoadPage[29:8];
	
	// write data to the RAM
	//if (DRDYReg) RAM[{LoadPage[7:0],DataCntr}]<={CFErrorFlag, DTi};
	
	// data counter
	DataCntr<=(DataCntr+{3'd0,DRDYReg}) & {4{LoadPageFlag}};

	
	end

endmodule
