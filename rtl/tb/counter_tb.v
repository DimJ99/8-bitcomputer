module test;

  // Clock and reset signals
  logic clk = 0;
  logic reset = 0;

  // Counter output
  logic [7:0] value;

  // Generate clock: toggles every 5 time units
  always #5 clk = ~clk;

  // Generate reset pulses
  initial begin
    #17 reset = 1;
    #11 reset = 0;
    #29 reset = 1;
    #11 reset = 0;
    #100 $stop;
  end

  // DUT: instantiate the counter
  counter c1 (
    .value(value),
    .clk  (clk),
    .reset(reset)
  );

  // Monitor output
  initial begin
    $monitor("At time %0t: VALUE = 0x%0h (%0d)", $time, value, value);
  end

endmodule
