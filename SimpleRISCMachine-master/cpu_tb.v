
module cpu_tb();

reg clk, reset;

reg [15:0] mdata;

wire [1:0] mem_cmd;

wire [8:0] mem_addr;

wire [15:0] out;
wire N,V,Z;

reg err;


cpu DUT(clk, mdata, mem_cmd, mem_addr, out, reset,N, V, Z);

initial begin
                      clk = 1'b0;
           forever begin
                      #10; clk = ~clk;
           end

end

initial begin
err=0;

@(posedge clk);
reset=0;

//SETS REGISTER R0 TO VALUE 1

mdata=16'b1101000000000001; //sets R0 to the value 1

@(posedge clk);

@(posedge clk);

//reset stage now

@(posedge clk);

 

//SETS REGISTER R1 TO VALUE 2

mdata=16'b1101000100000010; //sets R1 to the value 2

@(posedge clk);

@(posedge clk);

@(posedge clk);

 

//SETS REGISTER R2 TO VALUE -1

mdata=16'b1101001011111111; //sets R2 to the value -1

@(posedge clk);

@(posedge clk);

@(posedge clk);

 

//SETS REGISTER R3 TO VALUE -128

mdata=16'b1101001110000000; //sets R3 to the value -128

@(posedge clk);

@(posedge clk);

@(posedge clk);

 

//SETS REGISTER R4 TO VALUE 127
//Shifts the value using second MOV function
mdata=16'b1100000010001001; //sets R4 to the value 4

@(posedge clk);

@(posedge clk);

@(posedge clk);

 

// ADDS TOGETHER THE NUMBERS 1 AND 2

mdata=16'b1010000111100000; //1+2 (R0+R1) store to R7

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

if(out!=16'b0000000000000011)begin

$display("Expected output is 0000000000000011, output was %b", out);

err=1;

end


 

// ADDS TOGETHER THE NUMBERS 2 AND 2

mdata=16'b1010000111100001; //2+2(R1+R1) store to R7

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

#1;

if(out!=16'b0000000000000100)begin

$display("Expected output is 0000000000000001, output was %b", out);

err=1;

end

 

// ANDS THE NUMBERS 1 and 2

mdata=16'b1011000111100000; //1&2 (R0&R1) store to R7

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

#3;

if(out!={16{1'b0}})begin
$display("Expected output is 0000000000000000, output was %b", out);
err=1;
end

// MVN 2 which is stored in R1

mdata=16'b1011100011100001; //~2 (~R1) store to R7

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

#3;

if(out!={{14{1'b1}},2'b01})begin
$display("Expected output is 1111111111111101, output was %b", out);
err=1;
end

// compare R4 to R1

mdata=16'b1010110000000001; //R4-R1; 4-2

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

@(posedge clk);

#3;
/*
if(Z!=0)begin
$display("Expected output is 0, output was %b", out);
err=1;
end
if(N!=0)begin
$display("Expected output is 0, output was %b", out);
err=1;
end
if(V!=0)begin
$display("Expected output is 0, output was %b", out);
err=1;
end
*/
if(err==1)

$display("failed");

else

$display("passed");

$stop;
end

endmodule 

