
module ALU(Ain,Bin,ALUop,out,Z_out);
  input [15:0] Ain, Bin;
  input [1:0] ALUop;
  output [15:0] out;
  output [2:0]Z_out;
  reg [15:0] outTemp; 
  reg zZTemp, zNTemp, zOTemp;
//given a value of ALUop, we assign the result of an operation on Ain and Bin to a reg
always@(*) begin
  case (ALUop)
    2'b00: outTemp = Ain + Bin;
    2'b01: outTemp = Ain - Bin;
    2'b10: outTemp = Ain & Bin;
    2'b11: outTemp = ~Bin;
  endcase

  if(outTemp[15] == 1) zNTemp = 1; 
  else zNTemp = 0; 

  if(outTemp==0) zZTemp=1;
  else zZTemp = 0;



if (
((Ain[15]&Bin[15])&&(ALUop==2'd0)&&(~out[15]))||
((~Ain[15]&~Bin[15])&&(ALUop==2'd0)&&(out[15]))||
((Ain[15]&~Bin[15])&&(ALUop==2'd1)&&(~out[15]))||
((~Ain[15]&Bin[15])&&(ALUop==2'd1)&&(out[15]))
) zOTemp=1; //if out overflows v is one
else zOTemp=0;

  
end
assign Z_out={zOTemp, zNTemp, zZTemp};
assign out=outTemp;
endmodule 