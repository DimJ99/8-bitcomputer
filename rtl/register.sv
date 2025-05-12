module register(
  input wire [7:0] in,
  input wire clk,
  input wire reset,
  input wire enable,
  output reg [7:0] out
);
always_ff @(posedge clk or posedge reset) begin
  if (reset) begin
    out <= '0;
  end else if (enable) begin
    out <= in;
    $display("[MAR] loading bus = %0h into addr_bus", in);

  end
end


  endmodule