// ==========================
// Counter Testbench
// ==========================
module counter_tb;
  logic clk = 0;
  logic reset = 1;
  logic [7:0] out;

  // Clock
  always #5 clk = ~clk;

  // Instantiate
  counter c1 (
    .clk    (clk),
    .in     (8'h00),   // initial value
    .sel_in (reset),   // reset load
    .reset  (reset),
    .down   (1'b0),
    .out    (out)
  );

  // Release reset after one clock
  initial begin
    #12 reset = 0;
  end

  // Stimulus: let it count
  initial begin
    #100 $finish;
  end

  initial begin
    $monitor("At time %0t: COUNT = %0h (%0d)", $time, out, out);
  end
endmodule
