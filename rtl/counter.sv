module counter( 
  input  logic              clk,
  input  logic [7:0] in,
  input  logic              sel_in,
  input  logic              reset,
  input  logic              down,
  output logic [7:0] out
);
  input logic enable;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      out <= '0;
    end else if (sel_in) begin
      out <= in;
    end else if (down) begin
      out <= out - 1;
    end else begin
      out <= out + 1;
    end
  end

endmodule
