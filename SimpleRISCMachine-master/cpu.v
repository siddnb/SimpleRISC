



`define SW 5 //state width
`define SDEC 5'b00000//0
`define Sa 5'b00001 //1
`define Sb 5'b00010 //2
`define Sc 5'b00011 //3
`define Sd 5'b00100 //4             //each 5 bit constant reprsents a different state in FSM
`define Sm 5'b00101 //5
`define Sx 5'b00110 //6
`define Sw 5'b00111 //7
`define Sr 5'b01000 //reset state //8
`define Sy 5'b01001 //9
`define IF1 5'b01010 //10
`define IF2 5'b01011 //11
`define UpdatePC 5'b01100 //12
`define SLDR1 5'b01101 //13
`define SLDR2 5'b01110 //14
`define SLDR3 5'b01111 //15
`define SLDR4 5'b10000 //16
`define SLDR5 5'b10001 //17
`define SSTR1 5'b10010 //18
`define SSTR2 5'b10011 //19
`define SSTR3 5'b10100 //20
`define SSTR4 5'b10101 //21
`define SSTR5 5'b10110 //22
`define SSTR6 5'b10111 //23
`define HALT 5'b11000 //24


//CPU controls datapath, loadInstruction, instructionDecoder, and the FSM

module cpu(clk, mdata, mem_cmd, mem_addr,C,reset,N,V,Z, LED8);
input clk, reset;
input [15:0] mdata;
output [1:0] mem_cmd;
output [8:0] mem_addr;
output N,V,Z, LED8;
output [15:0] C;
wire [2:0]nsel;
wire[15:0] instruction, sximm8,sximm5, mdata ;
wire [2:0] opcode;
wire[1:0] op, ALUop, shift, vsel;
wire [2:0] Rd, Rm, Rn,Z_out, readnum, writenum;
wire write, loada, loadb, asel, bsel, loadc, loads,load_ir, load_pc, load_addr, reset_pc ;
wire [8:0] dataAddr_out, next_pc, PC;

wire [2:0] cond;
reg [8:0]pick_PC; //in_PC;
wire LED8;

loadInstruction #(16) loadInst(clk, mdata, load_ir, instruction); //loads instruction into instruction register if load is 1
loadInstruction #(9) PrgmCounter(clk, next_pc, load_pc, PC); //changes PC
loadInstruction #(9) dataAddress(clk, C[8:0], load_addr, dataAddr_out); 

/*
always@(posedge load_pc) begin
PC=next_pc;
end
*/
instructionDec InstDec(instruction, nsel, ALUop, opcode, sximm8, sximm5,shift, //takes the instruction and deciphers things about it 
       readnum, writenum, Rm, Rn, Rd);  //like op, opcode, sximm8+5, shift, also gives datapath read and writenum values based on nsel value 
                            //which respresents Rn, Rm, and Rd

assign cond= instruction[10:8]; //wil be used in following logic block to determine instruction

always@(*)begin


casex({opcode, op, cond, LED8}) //TABLE ONE IMPLEMENTATION
{3'b001, 2'bxx, 3'b000, 1'b0}: pick_PC=PC+sximm8+1'b1; //always sets PC to PC+!+sximm8
{3'b001, 2'bxx, 3'b001, 1'b0}: begin if(Z==1) pick_PC=PC+sximm8+1'b1; //depending on status flags, we either +1 or +1+sximm8
                              else pick_PC=PC+1'b1; end                  
{3'b001, 2'bxx, 3'b010, 1'b0}: begin if(Z==0) pick_PC=PC+sximm8+1'b1;//depending on status flags, we either +1 or +1+sximm8
                              else pick_PC=PC+1'b1; end
{3'b001, 2'bxx, 3'b011, 1'b0}: begin if(N!=V) pick_PC=PC+sximm8+1'b1;//depending on status flags, we either +1 or +1+sximm8
                              else pick_PC=PC+1'b1; end
{3'b001, 2'bxx, 3'b100, 1'b0}: begin if((N!=V)||(Z==1)) pick_PC=PC+sximm8+1'b1;
                              else pick_PC=PC+1'b1; end
{3'b111, 2'bxx, 3'bxxx, 1'b1}: begin  pick_PC=PC; end    //if opcode is 111 and the LED is set to 1, the PC stops changing because we
{3'bxxx, 2'bxx, 3'bxxx, 1'b0}: begin   pick_PC=PC+1'b1; end  //are in the HALT state


default: begin  pick_PC=PC+1'b1; end //pick_PC just adds one if nothing as been set

endcase 
 
if(PC==0) begin pick_PC=9'b000000001; end //makes sure when PC is initialized to 0, the next PC will be 1

end


Mux2 beforePC(pick_PC, {9{1'b0}}, reset_pc, next_pc); //makes PC 0 is reset is 1
Mux2 afterPC( dataAddr_out, PC, addr_sel, mem_addr);

FSM fsm_inst(reset, opcode, ALUop,nsel,vsel,loada,loadb,asel,bsel,loadc,loads,write,clk,
load_ir,load_pc,reset_pc,addr_sel,mem_cmd, load_addr, LED8); //state machine that runs on psedge clk


datapath DP(clk,readnum,vsel,loada,loadb,shift,asel,bsel,ALUop,loadc,loads //we have seen this in lab5, does sequence of things based on inputs
		,writenum,write,sximm8, sximm5, mdata, PC,C,Z_out);


assign N=Z_out[1]; //flag is 1 if it is a negative val
assign Z=Z_out[0]; //flag is 1 if it is 0
assign V=Z_out[2]; //flag is 1 if there is an overflow



endmodule

module FSM(reset, opcode, ALUop,nsel,vsel,loada,loadb,asel,bsel,loadc,loads,write,
clk,load_ir,load_pc,reset_pc,addr_sel,mem_cmd, load_addr, LED8);
input reset,clk;
input [1:0]ALUop; //input to ALU if opcode is 101, otherwise specifies instruction to perform
input [2:0] opcode; //either 110 or 101 depending on instruction we want to perform, further specification of this instruction is ALUop
output reg write, loada, loadb, asel, bsel, loadc, loads; //sending these back into datapath (except w, which tell us if we are in wait state or not)
output reg load_ir,load_pc,load_addr,reset_pc,addr_sel;
output reg [2:0] nsel; //based on if we want to set Rd, Rm, or Rn, this is one hot code to input into MUX
output reg[1:0] vsel, mem_cmd;
reg [`SW-1:0] state_next_reset,state_next; //represents next state
reg [`SW-1:0] present_state; //represents present state
reg [`SW+19-1:0] next; //first 4 bits represent next state, last 12 bits are specified below
output reg LED8;

//myvDFF #(`SW) STATE(state_next_reset, clk, present_state); //flip flop controls when state changes based on posedge of clk

//assign state_next_reset = reset ? `Sr :state_next ; //if reset==1, we will go to regular next state in sequence, if reset==0 we go to reset state
always@(posedge clk)begin
{state_next,write, loada, loadb, loadc, loads, asel, bsel, vsel, nsel, load_ir, load_pc,
reset_pc,addr_sel,mem_cmd,load_addr}= next;
present_state=state_next;
end

always @(*) begin
/*
What the 16 bits of next represent:                   [6] - asel
[15:12] - the next state in the sequence              [5] - bsel
[11] - write                                          [4:3] - vsel
[10] - loada                                          [2:0] - nsel -> Rn - nsel=001      Rm - nsel=100         Rd - nsel=010
[9] - loadb
[8] - loadc
[7] - loads
*/
/*
RESET
Sr -> Reset state, if in this state, w=1, wait for s=1 to move out of reset state

TRANSITION STATES
Sa -> Loads value under register Rn into register A
Sb -> Loads value under register Rm into register B
Sc -> ADDs, ANDs, CMPs, and MVNs and loads output value into register C
Sd -> Sets output value of previous state into register Rd
Sm -> Sends status flags of ALU to the status register to be outputed
Sw -> Send value at register B through the ALU with Ain=0 and adds these values together, loads this value to register C
Sx -> Loads value under register Rm into register B
Sy -> Sets value of Rn to im8
*/
casex({present_state,reset, opcode, ALUop}) //calculates next state and all outputs based on what the present state is and what instruction we want to perform 
{`Sr, 6'b0xxxxx}: begin next={`IF1, 16'b0000000000000001,2'b01,1'b0}; LED8=0;end //addr in PC
{`Sr, 6'b1xxxxx}: begin next={`Sr, 19'b0000000000000110000};LED8=0; end //if in reset state stay in reset state
{`IF1,6'b0xxxxx}: begin next = {`IF2, 16'b0000000000001001,2'b01,1'b0}; LED8=0; end // mem_cmd = 10 (write) and addr_sel =0
{`IF2,6'b0xxxxx}: begin next =  {`UpdatePC, 19'b0000000000000100000};  LED8=0;end //updates pc
{`UpdatePC,6'b0xxxxx}: begin next = {`SDEC, 19'b0000000000000000000}; LED8=0;end //decoder state
{`SDEC, 6'b010111}:begin next={`Sb, 12'b0_0_1_0_0_0_0_00_100,7'b0000000}; LED8=0; end// mem_cmd = 10 (write) and addr_sel =0
{`SDEC, 6'b0101x0}:begin next={`Sa, 12'b0_1_0_0_0_0_0_00_001,7'b0000000}; LED8=0;end//Loads value under register Rn into register A
{`SDEC, 6'b010101}:begin next={`Sa, 12'b0_1_0_0_0_0_0_00_001,7'b0000000};LED8=0; end//nsel=001, loada=1
{`SDEC, 6'b011010}:begin next={`Sy, 12'b1_0_0_0_0_0_0_10_001,7'b0000000}; LED8=0;end //sends to y to start MOV instructions
{`SDEC, 6'b011000}:begin next={`Sx, 12'b0_0_1_0_0_0_0_00_100,7'b0000000}; LED8=0;end//loada=0, asel=0,bsel=1,loadc=1
{`SDEC, 6'b001100}:begin next={`SLDR1, 12'b0_1_0_0_0_0_0_00_001,7'b0000000}; LED8=0;end //nsel=001, loada=1
{`SDEC, 6'b010000}:begin next={`SSTR1, 12'b0_1_0_0_0_0_0_00_001,7'b0000000}; LED8=0;end //nsel=001, loada=1
{`SDEC, 6'b0111xx}:begin next={`HALT, 12'b0_0_0_0_0_0_0_00_000,7'b0000000}; LED8=0;end// mem_cmd = 10 (write) and addr_sel =0
{`SLDR1, 6'b0xxxxx}:begin next={`SLDR2, 12'b0_0_0_1_0_0_1_00_001,7'b0000000};LED8=0; end //loada=0, asel=0,bsel=1,loadc=1
{`SLDR2, 6'b0xxxxx}:begin next={`SLDR3, 12'b0_0_0_0_0_0_0_00_001,7'b0000001};LED8=0; end // load_addr=1
{`SLDR3, 6'b0xxxxx}:begin next={`SLDR4, 12'b0_0_0_0_0_0_0_00_001,7'b0000010};LED8=0; end //addr_sel=0,mem_cmd=01
{`SLDR4, 6'b0xxxxx}:begin next={`SLDR5, 12'b1_0_0_0_0_0_0_11_010,7'b0000010}; LED8=0;end //vsel=11, nsel=010, write=1
{`SLDR5, 6'b0xxxxx}:begin next={`IF1, 12'b0_0_0_0_0_0_0_00_010,7'b0001010};LED8=0; end// mem_cmd = 10 (write) and addr_sel =0
{`SSTR1, 6'b0xxxxx}:begin next={`SSTR2, 12'b0_0_0_1_0_0_1_00_001,7'b0000000};LED8=0; end //loada=0, asel=0,bsel=1,loadc=1
{`SSTR2, 6'b0xxxxx}:begin next={`SSTR3, 12'b0_0_0_0_0_0_0_00_001,7'b0000001};LED8=0; end // load_addr=1 //CHANGED NSEL from 001f
{`SSTR3, 6'b0xxxxx}:begin next={`SSTR4, 12'b0_0_1_0_0_0_0_00_010,7'b0000000};LED8=0; end //nsel=010, loadb=1 
{`SSTR4, 6'b0xxxxx}:begin next={`SSTR5, 12'b0_0_0_1_0_1_0_00_010,7'b0000000};LED8=0; end //asel=1 loadc=1
{`SSTR5, 6'b0xxxxx}:begin next={`SSTR6, 12'b0_0_0_0_0_0_0_00_010,7'b0000100}; LED8=0;end//Sets output value of previous state into register Rd
{`SSTR6, 6'b0xxxxx}:begin next={`IF1, 16'b0000000000000001,2'b01,1'b0}; LED8=0;end//Loads value under register Rm into register B
{`Sa, 6'b0xxxxx}:begin next={`Sb, 12'b0_0_1_0_0_0_0_00_100,7'b0000000}; LED8=0; end//loada=0, asel=0,bsel=1,loadc=1
{`Sb, 6'b0xxxxx}:begin next={`Sc, 12'b0_0_0_1_0_0_0_00_100,7'b0000000};LED8=0; end
{`Sc, 6'b0xxxx0}:begin next={`Sd, 12'b1_0_0_0_0_0_0_00_010,7'b0000000}; LED8=0;end//Sets output value of previous state into register Rd
{`Sc, 6'b010101}:begin next={`Sm, 12'b0_0_0_0_1_0_0_00_010,7'b0000000}; LED8=0;end//mem_cmd = 10 (write) and addr_sel =0
{`Sc, 6'b0xxx11}:begin next={`Sd, 12'b1_0_0_0_0_0_0_00_010,7'b0000000}; LED8=0; end//loada=0, asel=0,bsel=1,loadc=1

{`Sm, 6'b0xxxxx}:begin next={`IF1, {9{1'b0}}, 3'b000,7'b0001010}; LED8=0;end//Loads value under register Rn into register A
{`Sd, 6'b0xxxxx}:begin next={`IF1, {9{1'b0}}, 3'b000,7'b0001010}; LED8=0; end//Loads value under register Rm into register B
{`Sx, 6'b0xxx00}:begin next={`Sw, 12'b0_0_0_1_0_1_0_00_100,7'b0000000}; LED8=0;end//Send value at register B through the ALU with Ain=0 and adds these values together, loads this value to register C
{`Sw, 6'b0xxx00}:begin next={`Sd, 12'b1_0_0_0_0_0_0_00_010,7'b0000000}; LED8=0; end//loada=0, asel=0,bsel=1,loadc=1
{`Sy, 6'b0xxxxx}:begin next={`IF1, {9{1'b0}}, 3'b000,7'b0001010}; LED8=0;end//Loads value under register Rm into register B
{`HALT, 6'b0111xx}:begin next={`HALT, {19{1'b0}}}; LED8=1'b1; end
{{5{1'bx}},1'b1,5'bxxxxx}: begin next={`Sr, 19'b0000000000000110000}; LED8=0; end
default: begin next={`IF1, 16'b0000000000000001,2'b01,1'b0}; LED8=0;end
endcase

end

endmodule


//takes the instruction and deciphers things about it 
//like op, opcode, sximm8+5, shift, also gives datapath read and writenum values based on nsel value 
//which respresents Rn, Rm, and Rd
module instructionDec(instruction, nsel, opO, opcodeO, sximm8, sximm5,shift, 
       readnumO, writenumO, Rn, Rm, Rd);
input [15:0] instruction;
input [2:0]nsel;
wire [7:0] imm8;
wire [4:0] imm5;
wire [2:0] Rout;
output [2:0] Rd, Rm, Rn;
output [1:0] opO,shift;
output [2:0]opcodeO, readnumO, writenumO;
output [15:0]sximm8,sximm5;

Mux3 U5(Rn, Rd, Rm, nsel, Rout); //picks which Rn, Rd, or Rm value we want to work with based on the nsel value we are given
                                 //Rn - nsel=001      Rm - nsel=100         Rd - nsel=010
assign sximm8 = { {8{imm8[7]}}, imm8[7:0] }; //sximm8 is sign extended version of imm8
assign sximm5 = { {11{imm5[4]}}, imm5[4:0] }; //sximm5 is sign extended version of imm5


//sets outputs for all things instruction could represent
assign imm8=instruction[7:0];
assign imm5=instruction[4:0];
assign Rd=instruction[7:5];            
assign Rm=instruction[2:0];
assign Rn=instruction[10:8];

assign opO=instruction[12:11];
assign opcodeO=instruction[15:13];
assign readnumO=Rout;
assign writenumO=Rout;
assign shift=instruction[4:3];

endmodule

//loads instruction into register if load is 1, otherwise, nothing happens
module loadInstruction(clk,in,load,out);
parameter n = 16;
input clk, load;
input [n-1:0] in;
output [n-1:0] out;
reg [n-1:0] out;
wire [n-1:0] next_out;
 
//If the load is true then update the value, otherwise assign the original value again.
assign next_out = load ? in : out; 

always @(posedge clk) begin
out = next_out;
end
endmodule 

module myvDFF(D, clk, Q);
parameter n=4; 
input [n-1:0]D; // Data input 
input clk; // clock input 
output [n-1:0]Q; // output Q 
reg [n-1:0]temp;
always @(posedge clk) 
begin
 temp = D; 
end 
assign Q=temp;
endmodule


module Mux3(a0, a1, a2, sel, out);
input [2:0] a0, a1, a2;
input [2:0] sel;
output [2:0] out;
reg [2:0] temp;

always @(*) begin
case(sel)
3'b100:temp=a2;
3'b010:temp=a1;
3'b001:temp=a0;
default: temp=3'bxxx;
endcase
end
assign out=temp;
endmodule

module Mux2(a0, a1, sel, out);
input [8:0] a0, a1;
input sel;
output [8:0] out;
reg [8:0] temp;

always @(*) begin
case(sel)
0: temp=a0;
1: temp=a1;
default: temp={9{1'bx}};
endcase
end
assign out=temp;
endmodule


