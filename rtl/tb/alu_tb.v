module alu_tb;

  // Inputs
  logic [7:0] in_a;
  logic [7:0] in_b;
  logic       cin = 0;

  // Outputs
  logic [7:0] sum;
  logic       cout;

  // DUT instantiation
  alu dut (
    .cin (cin),
    .cout(cout),
    .in_a(in_a),
    .in_b(in_b),
    .sum (sum)
  );

  // Stimulus
  initial begin
    $display("Starting ALU Testbench...");

    #10 in_a = 8'd10; in_b = 8'd20;
    #10 in_a = 8'd50; in_b = 8'd10;
    #10 in_a = 8'd100; in_b = 8'd155;
    #10 in_a = 8'd255; in_b = 8'd1;
    #10 $finish;
  end

  // Output monitor
  initial begin
    $monitor("time=%0t | CIN=%0b COUT=%0b | A=%0d B=%0d → SUM=%0d", 
              $time, cin, cout, in_a, in_b, sum);
  end

endmodule
