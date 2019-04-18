module ALU_tb; 
  reg [15:0] Ain, Bin;
  reg [1:0] ALUop;
  wire [15:0] out;
  wire [2:0]Z;
  reg err;

  ALU DUT(Ain,Bin,ALUop,out,Z);

  //creates a checker to be called upon using the expected values
  task my_checker;
    input [15:0] expected_out;
    input [2:0]expected_z;
  begin
    //error messages get displayed when the expected output in not the same as actual output
    if(ALU_tb.DUT.out !== expected_out) begin
      $display("ERROR: expected output is %b, actual is %b", expected_out, ALU_tb.DUT.out);
      err = 1'b1;
    end
    if(ALU_tb.DUT.zZTemp != expected_z[0]) begin
      $display("ERROR: expected z is %b, actual is %b", expected_z[0], ALU_tb.DUT.zZTemp);
      err = 1'b1;
    end
  end
  endtask

  initial begin
    //initialise all values
    Ain = 16'b0000000000001010; Bin = 16'b0000000000000001;
    ALUop = 2'b00; err = 1'b0;
    #10;
    $display("Checking Ain + Bin");
    my_checker(16'b0000000000001011, 1'b0);
    
    ALUop = 2'b01;
    #10;
    $display("Checking Ain - Bin");
    my_checker(16'b0000000000001001, 1'b0);
    
    ALUop = 2'b10;
    #10;
    $display("Checking Ain & Bin");
    my_checker(16'b0000000000000000, 1'b1);
    
    ALUop = 2'b11;
    #10;
    $display("Checking ~Bin");
    my_checker(16'b1111111111111110, 1'b0);
    #10;
    if(err) $display("FAILED");
    else $display("PASSED");
    end
endmodule
