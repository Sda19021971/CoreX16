module InternalMemory1M #(parameter TagWidth=21)(
	input wire CLK, ACT, CMD,
	input wire [16:0] ADDR,
	input wire [7:0] BE,
	input wire [63:0] DI,
	input wire [TagWidth-1:0] TI,
	output reg DRDY,
	output reg [63:0] DO,
	output reg [TagWidth-1:0] TO
	);
	
reg [7:0][7:0] ram[0:131071];
	
always_ff @(posedge CLK)
begin
if (ACT & ~CMD)
	begin
	if (~BE[0]) ram[ADDR][0]<=DI[7:0];
	if (~BE[1]) ram[ADDR][1]<=DI[15:8];
	if (~BE[2]) ram[ADDR][2]<=DI[23:16];
	if (~BE[3]) ram[ADDR][3]<=DI[31:24];
	if (~BE[4]) ram[ADDR][4]<=DI[39:32];
	if (~BE[5]) ram[ADDR][5]<=DI[47:40];
	if (~BE[6]) ram[ADDR][6]<=DI[55:48];
	if (~BE[7]) ram[ADDR][7]<=DI[63:56];
	end
DO<=ram[ADDR];
TO<=TI;
DRDY<=ACT & CMD;
end

endmodule
