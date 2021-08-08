module Mult108 (
	input wire CLK,
	input wire [107:0] A,B,
	output wire [24:0] R32,
	output wire [53:0] R64,
	output wire [113:0] R128
);

// mantissa
logic [107:0] MA, MB;

// multiplication partial results
reg [35:0] A0B0, A1B0, A2B0, A3B0, A4B0, A5B0;
reg [35:0] A0B1, A1B1, A2B1, A3B1, A4B1, A5B1;
reg [35:0] A0B2, A1B2, A2B2, A3B2, A4B2, A5B2;
reg [35:0] A0B3, A1B3, A2B3, A3B3, A4B3, A5B3;
reg [35:0] A0B4, A1B4, A2B4, A3B4, A4B4, A5B4;
reg [35:0] A0B5, A1B5, A2B5, A3B5, A4B5, A5B5;

// first stage
reg [17:0] StageA_Low;
reg [54:0] StageA_0;
reg [55:0] StageA[6:1];
reg [54:0] StageA_7;
reg [72:0] StageA_8;
reg [54:0] StageA32;
reg [55:0] StageA64_Low;
reg [54:0] StageA64_Medium;
reg [35:0] StageA64_High;

// second stage
reg [35:0] StageB_Low;
reg [56:0] StageB0;
reg [74:0] StageB1;
reg [74:0] StageB2;
reg [73:0] StageB3;
reg [71:0] StageB64;

// thrid stage
reg [53:0] StageC_Low;
reg [75:0] StageC1;
reg [128:0] StageC2;

// fourth stage
reg [89:0] StageD_Low;
reg [72:0] StageD_Medium;
reg [56:0] StageD_High;

// fifth stage
reg [161:0] StageE_Low;
reg [56:0] StageE_High;


assign R32=StageA32[53:29];
assign R64=StageB64[71:18];
assign R128={StageE_High[53:0], StageE_Low[161:102]};

always_comb
begin
MA=A[107:0];
MB=B[107:0];
end


always_ff @(posedge CLK)
begin

// create partial multiplication results
A0B0<=MA[17:0]*MB[17:0];
A1B0<=MA[35:18]*MB[17:0];
A2B0<=MA[53:36]*MB[17:0];
A3B0<=MA[71:54]*MB[17:0];
A4B0<=MA[89:72]*MB[17:0];
A5B0<=MA[107:90]*MB[17:0];

A0B1<=MA[17:0]*MB[35:18];
A1B1<=MA[35:18]*MB[35:18];
A2B1<=MA[53:36]*MB[35:18];
A3B1<=MA[71:54]*MB[35:18];
A4B1<=MA[89:72]*MB[35:18];
A5B1<=MA[107:90]*MB[35:18];

A0B2<=MA[17:0]*MB[53:36];
A1B2<=MA[35:18]*MB[53:36];
A2B2<=MA[53:36]*MB[53:36];
A3B2<=MA[71:54]*MB[53:36];
A4B2<=MA[89:72]*MB[53:36];
A5B2<=MA[107:90]*MB[53:36];

A0B3<=MA[17:0]*MB[71:54];
A1B3<=MA[35:18]*MB[71:54];
A2B3<=MA[53:36]*MB[71:54];
A3B3<=MA[71:54]*MB[71:54];
A4B3<=MA[89:72]*MB[71:54];
A5B3<=MA[107:90]*MB[71:54];

A0B4<=MA[17:0]*MB[89:72];
A1B4<=MA[35:18]*MB[89:72];
A2B4<=MA[53:36]*MB[89:72];
A3B4<=MA[71:54]*MB[89:72];
A4B4<=MA[89:72]*MB[89:72];
A5B4<=MA[107:90]*MB[89:72];

A0B5<=MA[17:0]*MB[107:90];
A1B5<=MA[35:18]*MB[107:90];
A2B5<=MA[53:36]*MB[107:90];
A3B5<=MA[71:54]*MB[107:90];
A4B5<=MA[89:72]*MB[107:90];
A5B5<=MA[107:90]*MB[107:90];

// first stage adder
StageA_Low<=A0B0[17:0];
StageA_0<={1'd0,A2B0,A0B0[35:18]}+{19'd0,A1B0}+{19'd0,A0B1};
StageA[1]<={2'd0,A3B0,18'd0}+{2'd0,A4B0[17:0],A1B1}+{2'd0,A3B1[17:0],A0B2};
StageA[2]<={2'd0,A4B0[35:18],A2B1}+{2'd0,A3B1[35:18],A1B2}+{2'd0,A2B2[35:18],A0B3};
StageA[3]<={2'd0,A5B0,A2B2[17:0]}+{2'd0,A4B2[17:0],A1B3}+{2'd0,A3B3[17:0],A0B4};
StageA[4]<={2'd0,A5B2[17:0],A4B1}+{2'd0,A4B2[35:18],A3B2}+{2'd0,A3B3[35:18],A2B3};
StageA[5]<={2'd0,A5B1,18'd0}+{2'd0,A4B3[17:0],A1B4}+{2'd0,A3B4[17:0],A0B5};
StageA[6]<={2'd0,A5B2[35:18],A2B4}+{2'd0,A4B3[35:18],A1B5}+{2'd0,A2B5,18'd0};
StageA_7<={19'd0,A5B3}+{19'd0,A4B4}+{1'd0,A5B5[17:0],A3B5};
StageA_8<={19'd0,A5B4,A3B4[35:18]}+{1'd0,A5B5[35:18],A4B5,18'd0};


// second stage adder
StageB_Low<={StageA_0[17:0], StageA_Low};
StageB0<={1'd0,StageA[1]}+{20'd0, StageA_0[54:18]};
StageB1<={1'd0, StageA[3], 18'd0}+{19'd0, StageA[2]};
StageB2<={1'b0,StageA[6],18'd0}+{19'd0,StageA[5]}+{19'd0,StageA[4]};
StageB3<={19'd0,StageA_7}+{1'd0,StageA_8};

// thrid stage adder
StageC_Low<={StageB0[17:0], StageB_Low};
StageC1<={37'd0, StageB0[56:18]}+{1'b0, StageB1};
StageC2<={54'd0, StageB2}+{1'd0,StageB3, 54'd0};

// fourth stage adder
StageD_Low<={StageC1[35:0], StageC_Low};
StageD_Medium<={33'd0, StageC1[75:36]}+{1'b0, StageC2[71:0]};
StageD_High<=StageC2[128:72];

// fifth stage
StageE_Low<={StageD_Medium[71:0], StageD_Low};
StageE_High<=StageD_High+{56'd0, StageD_Medium[72]};

// 32-bit operation
StageA32<={1'b0, A5B5, A4B4[35:18]}+{19'd0, A5B4}+{19'd0, A4B5};

// 64-bit operation
StageA64_Low<={2'd0, A5B3, A3B3[35:18]}+{2'd0, A5B4[17:0], A4B3}+{2'd0, A4B5[17:0], A3B4};
StageA64_Medium<={1'b0, A5B4[35:18], A4B4}+{1'b0, A4B5[35:18], A3B5};
StageA64_High<=A5B5;

StageB64<={StageA64_High, 36'd0}+{17'd0, StageA64_Medium}+{34'd0, StageA64_Low[55:18]};

end


endmodule
