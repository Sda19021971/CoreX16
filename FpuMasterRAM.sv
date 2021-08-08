module FpuMasterRAM #(parameter Mode=1, Width=32)(
					input wire CLK, RDEN, WREN,
					input wire [3:0] RADDR, WADDR,
					input wire [Width-1:0] DI,
					output wire [Width-1:0] DO
					);

generate
if (Mode==0)
begin

altsyncram	MasterRAM(
				.wren_a (WREN),
				.clock0 (CLK),
				.clock1 (CLK),
				.address_a (WADDR),
				.address_b (RADDR),
				.rden_b (RDEN),
				.data_a (DI),
				.q_b (DO),
				.aclr0 (1'b0),
				.aclr1 (1'b0),
				.addressstall_a (1'b0),
				.addressstall_b (1'b0),
				.byteena_a (1'b1),
				.byteena_b (1'b1),
				.clocken0 (1'b1),
				.clocken1 (1'b1),
				.clocken2 (1'b1),
				.clocken3 (1'b1),
				.data_b ({Width{1'b1}}),
				.eccstatus (),
				.q_a (),
				.rden_a (1'b1),
				.wren_b (1'b0));
	defparam
		MasterRAM.address_reg_b = "CLOCK1",
		MasterRAM.clock_enable_input_a = "BYPASS",
		MasterRAM.clock_enable_input_b = "BYPASS",
		MasterRAM.clock_enable_output_a = "BYPASS",
		MasterRAM.clock_enable_output_b = "BYPASS",
		MasterRAM.lpm_type = "altsyncram",
		MasterRAM.numwords_a = 16,
		MasterRAM.numwords_b = 16,
		MasterRAM.operation_mode = "DUAL_PORT",
		MasterRAM.outdata_aclr_b = "NONE",
		MasterRAM.outdata_reg_b = "UNREGISTERED",
		MasterRAM.power_up_uninitialized = "FALSE",
		MasterRAM.rdcontrol_reg_b = "CLOCK1",
		MasterRAM.widthad_a = 4,
		MasterRAM.widthad_b = 4,
		MasterRAM.width_a = Width,
		MasterRAM.width_b = Width,
		MasterRAM.width_byteena_a = 1;
	end
else begin

reg [Width-1:0] MasterRAM [0:15];
reg [Width-1:0] DOReg;

assign DO=DOReg;

always_ff @(posedge CLK)
begin

if (WREN) MasterRAM[WADDR]<=DI;

if (RDEN) DOReg<=MasterRAM[RADDR];

end

end

endgenerate

endmodule
