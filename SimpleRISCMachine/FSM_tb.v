
module FSM_tb();

reg s, reset,clk;
wire w,loada,loadb,asel,bsel,loadc,loads,write;
reg [1:0] ALUop; 
reg [2:0] opcode;
wire [2:0]nsel;
wire [1:0] vsel;

initial begin
		clk = 1'b0;
	forever begin
		#5; clk = ~clk;
	end
end

FSM DUTFSM(s, reset, opcode, ALUop,nsel, w,vsel,loada,loadb,asel,bsel,loadc,loads,write,clk);

initial begin
#1;
reset=0;
@(posedge clk);
$display(" The state is %b, expected 1000", FSM_tb.DUTFSM.present_state);
opcode=3'b110;
ALUop=2'b10;
s=1;
reset=1;
@(posedge clk);
$display(" The state is %b, expected 1000", FSM_tb.DUTFSM.present_state);
@(posedge clk);
$display("The state is %b, expected 1111", FSM_tb.DUTFSM.present_state);
@(posedge clk);
$display("The state is %b, expected 1110", FSM_tb.DUTFSM.present_state);
//if(FSM_tb.FSMDUT.present_state==

$stop;
end




endmodule