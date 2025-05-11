module reg(
  input wire [7:0] in,
  input wire clk,
  input wire reset,
  input wire enable,
  output reg [7:0] out
);
  always_ff @(posedge clk or posedge reset) begin
    if (reset)
      out <= '0;
    else if (enable)
      out <= in;
  end
  endmodule