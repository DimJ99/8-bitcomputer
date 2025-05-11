module clock (
  input  logic enable,
  output logic clk,
  output logic clk_inv
);

  initial clk = 0;
  always begin
    #10; //10 ns or sm idek
    if (enable)
      clk = ~clk;
  end

  assign clk_inv = ~clk;

endmodule
