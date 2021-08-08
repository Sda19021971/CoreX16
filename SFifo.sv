
module SFifo (
	input wire CLK, RST,
	// data input
	input wire D,
	input wire WR,
	// data output
	output reg Q,
	input wire RD,
	// status
	output wire VALID
);

reg [4:0] DataRegs;
reg [5:0] ValidFlags;
wire RDFlag;

assign VALID=ValidFlags[0], RDFlag=VALID & RD;

always_ff @(posedge CLK)
begin

// data to output register
if (RDFlag | (WR & ~(|ValidFlags))) Q<=ValidFlags[1] ? DataRegs[0] : D;
if (RDFlag | (WR & ~(|ValidFlags[5:1]))) DataRegs[0]<=ValidFlags[2] ? DataRegs[1] : D;
if (RDFlag | (WR & ~(|ValidFlags[5:2]))) DataRegs[1]<=ValidFlags[3] ? DataRegs[2] : D;
if (RDFlag | (WR & ~(|ValidFlags[5:3]))) DataRegs[2]<=ValidFlags[4] ? DataRegs[3] : D;
if (RDFlag | (WR & ~(|ValidFlags[5:4]))) DataRegs[3]<=ValidFlags[5] ? DataRegs[4] : D;
if (WR & (~ValidFlags[5] | RDFlag)) DataRegs[4]<=D;

// busy flags
ValidFlags[0]<=((ValidFlags[0] & (~RDFlag | ValidFlags[1])) | WR) & RST;
ValidFlags[1]<=((ValidFlags[1] & (~RDFlag | ValidFlags[2])) | (WR & ValidFlags[0] & (~RDFlag | ValidFlags[1]))) & RST;
ValidFlags[2]<=((ValidFlags[2] & (~RDFlag | ValidFlags[3])) | (WR & ValidFlags[1] & (~RDFlag | ValidFlags[2]))) & RST;
ValidFlags[3]<=((ValidFlags[3] & (~RDFlag | ValidFlags[4])) | (WR & ValidFlags[2] & (~RDFlag | ValidFlags[3]))) & RST;
ValidFlags[4]<=((ValidFlags[4] & (~RDFlag | ValidFlags[5])) | (WR & ValidFlags[3] & (~RDFlag | ValidFlags[4]))) & RST;
ValidFlags[5]<=((ValidFlags[5] & ~RDFlag) | (WR & ValidFlags[4] & (~RDFlag | ValidFlags[5]))) & RST;

end


endmodule
