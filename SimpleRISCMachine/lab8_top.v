

//this is lab7_top module with a few changes to accomodate the chnages made in lab8
`define MWRITE 2'b10
`define MREAD 2'b01
`define MNONE 2'b00


module lab8_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5, clk);
input clk; //may need to change
input [3:0] KEY;
input [9:0] SW;
output [9:0] LEDR;
output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
//wire N,V,Z;
wire [1:0] mem_cmd;
wire [8:0] mem_addr;
wire [15:0] read_data, dout, din;
wire clk, enableTri_dout, write, enableTri_SW, reset, LED8; 
wire [7:0] next_LEDs;
wire [15:0] write_data;
wire [15:0] out;
assign LEDR[9]=0;
assign HEX4=7'b1111111;
assign HEX5=7'b1111111;

 sseg H0(out[3:0],   HEX0);  //assigns hex's
  sseg H1(out[7:4],   HEX1);
  sseg H2(out[11:8],  HEX2);
  sseg H3(out[15:12], HEX3);


assign out=write_data;

assign reset=~KEY[1];

RAM MEM(clk,mem_addr[7:0],mem_addr[7:0],write,write_data,dout); //instatiates RAM

assign enableTri_dout = (mem_cmd==`MREAD) && (mem_addr[8]==1'b0); //sets the enable for the tristate buffer connected to output of dout

assign read_data = enableTri_dout? dout : 16'bz;//this is tristate buffer

assign write = (mem_cmd==`MWRITE) && (mem_addr[8]==0); //tells whether or not we are writing

//assign clk=~KEY[0]; //this is the clock



assign LEDR[7:0]=next_LEDs;

loadLEDs #(8) loadLED(clk, write_data[7:0], load, next_LEDs); //load LEDs next output to LED register

assign load= ((mem_cmd==`MWRITE) && (mem_addr==9'h100)); //load that controls loading LEDs

assign enableTri_SW = ((mem_cmd==`MREAD)&&(mem_addr==9'h140)); //enable for tristate driver that connects the input switches to read_data

assign read_data[7:0] = enableTri_SW? SW[7:0] : {8{1'bz}}; //tristate buffer connecting switched to read_data

cpu CPU(clk,read_data, mem_cmd, mem_addr, write_data, reset,N,V,Z, LED8);

assign LEDR[8]=LED8;

endmodule

module RAM(clk,read_address,write_address,write,din,dout); //cite: Slide Set 7
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "lab8fig2.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem); //reads file

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end

 
endmodule


module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;
  always @(posedge clk)
    Q <= D;
endmodule

module loadLEDs(clk,in,load,out);
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




module sseg(in,segs);
  input [3:0] in;
  output [6:0] segs;

  reg [6:0] segst;

always@(*) begin 
case(in)
4'b0000 : segst = 7'b1000000;
4'b0001 : segst = 7'b1111001;
4'b0010 : segst = 7'b0100100;
4'b0011 : segst = 7'b0110000;
4'b0100 : segst = 7'b0011001;
4'b0101 : segst = 7'b0010010;
4'b0110 : segst = 7'b0000011;
4'b0111 : segst = 7'b1111000;
4'b1000 : segst = 7'b0000000;
4'b1001 : segst = 7'b0011000;
4'b1010 : segst = 7'b0001000;
4'b1011 : segst = 7'b0000011;
4'b1100 : segst = 7'b1000110;
4'b1101 : segst = 7'b0100001;
4'b1110 : segst = 7'b0000110;
4'b1111 : segst = 7'b0001110;
endcase

end
assign segs = segst;

endmodule

