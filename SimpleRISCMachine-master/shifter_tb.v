module shifter_tb;
  reg [15:0] in;
  reg [1:0] shift;
  wire [15:0] sout;
  reg err;

  shifter DUT(in,shift,sout);

  task my_checker;
    input [15:0] expected_out;

    begin
      if(shifter_tb.DUT.sout !== expected_out) begin
        $display("ERROR: expected output is %b, actual is %b", expected_out, shifter_tb.DUT.sout);
        err = 1'b1;
      end
    end
  endtask

  initial begin
  in = 16'b1111000011110000; shift = 2'b00; err = 1'b0; 

  $display("checking no shift");
  #10;
  my_checker(16'b1111000011110000);

  $display("Checking shift to the left");
  shift = 2'b01;
  #10;
  my_checker(16'b1110000111100000);

  $display("Checking shift to right");
  shift = 2'b10;
  #10;
  my_checker(16'b0111100001111000);

  $display("Checking shift to right with replacement of 15th bit");
  shift = 2'b11;
  #10;
  my_checker(16'b1111100001111000);
  if(err) $display("FAILED");
  else $display("PASSED");
  end
endmodule


