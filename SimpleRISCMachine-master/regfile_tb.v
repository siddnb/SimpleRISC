//TestBench for the regfile
module regfile_tb();
reg [15:0] data_in;
reg [2:0] writenum, readnum;
reg write, clk;
wire [15:0] data_out;
reg err;  // A signal to catch an error. 

regfile DUT(data_in,writenum,write,readnum,clk,data_out); 

//Set the clock to alternate truth values back and forth. 
initial begin
		clk = 1'b1;
	forever begin
		#5; clk = ~clk;
	end
end

//A task to aid in expiditing the "self checking" testbench. 
task checker;
	input [15:0] expectedO;
begin
	//Print out error message if the current state is not the expected state or is undefined or z. 
  	if(regfile_tb.DUT.data_out !== expectedO) begin
	$display("Error ** output is %b, expected %b",
		regfile_tb.DUT.data_out, expectedO);
	err = 1'b1; 	//Set the err signal to true (1) to aid in debugging. 
	end
       end
endtask

initial begin
//First load arbitrary values into each register to test reading from each. 
write = 1'b1; writenum = 3'b000; readnum = 3'b000; data_in = 16'b0000000000000001; err = 1'b0;
#10; writenum = 3'b001; data_in = 16'b0000000000000010;
#10; writenum = 3'b010; data_in = 16'b0000000000000100;
#10; writenum = 3'b011; data_in = 16'b0000000000001000;
#10; writenum = 3'b100; data_in = 16'b0000000000010000;
#10; writenum = 3'b101; data_in = 16'b0000000000100000;
#10; writenum = 3'b110; data_in = 16'b0000000001000000;
#10; writenum = 3'b111; data_in = 16'b0000000010000000; 
#10; write = 1'b0; 
			//Check all the registers. 
//Check reading the first register.  
	$display ("Checking R0");
	#10; checker(16'b0000000000000001);

//Check reading the 2nd register.  
	$display ("Checking R1");
	readnum = 3'b001; 
	#10; checker(16'b0000000000000010);

//Check reading the 3rd register.  
	$display ("Checking R2");
	readnum = 3'b010; 
	#10; checker(16'b0000000000000100);

//Check reading the 4th register.  
	$display ("Checking R3");
	readnum = 3'b011; 
	#10; checker(16'b0000000000001000);

//Check reading the 5th register.  
	$display ("Checking R4");
	readnum = 3'b100; 
	#10; checker(16'b0000000000010000);

//Check reading the 6th register.  
	$display ("Checking R5");
	readnum = 3'b101; 
	#10; checker(16'b0000000000100000);

//Check reading the 7th register.  
	$display ("Checking R6");
	readnum = 3'b110; 
	#10; checker(16'b0000000001000000);

//Check reading the 8th register.  
	$display ("Checking R7");
	readnum = 3'b111; 
	#10; checker(16'b0000000010000000);

//Rewrite the binary number 24 into R4 and read from R4.
	writenum = 3'b100; write = 1'b1; data_in = 16'b0000000000011000; readnum = 3'b100;
	$display("Reading from R4 after writing the number 24(binary) into R4");  
	#10; checker(16'b0000000000011000); 

//Output whether all tests passed or one or more failed based on the err signal. 
	if(~err) $display ("Passed!");
	else $display("Failed!"); 
end
endmodule


