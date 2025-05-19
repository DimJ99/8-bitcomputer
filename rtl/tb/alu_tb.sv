// ==========================
// ALU Testbench
// ==========================
module register_tb;

  // Inputs
  logic [7:0] in = 8'd15;
  logic       reset = 0;
  logic       enable = 1;
  logic       clk = 0;

  // Output
  logic [7:0] value;

  // Clock generation: toggle every 5 time units
  always #5 clk = ~clk;

  // Instantiate the DUT (register)
  register r1 (
    .in(in),
    .clk(clk),
    .enable(enable),
    .reset(reset),
    .out(value)
  );

  // Stimulus
  initial begin
    #17 reset = 1;
    #10 in = 8'd10;
    #10 enable = 0;
    #10 in = 8'd5;
    #10 enable = 1;
    #100 $stop;
  end

  // Output monitor
  initial begin
    $monitor("At time %0t: VALUE = 0x%0h (%0d), ENABLE = %b",
              $time, value, value, enable);
  end
initial begin
  #5000 $display("Simulation timed out!"); $finish;
end
endmodule
