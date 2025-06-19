module counter (
  input  logic        clk,          // system clock
  input  logic        reset,        // asynchronous reset
  input  logic [7:0]  load_value,   // value to load into PC
  input  logic        load,         // synchronous load enable
  input  logic        inc,          // increment enable
  input  logic        dec,          // decrement enable
  output logic [7:0]  out           // current PC value
);

  // Priority: reset > load > decrement > increment
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      out <= 8'd0;
    end else if (load) begin
      out <= load_value;
    end else if (dec) begin
      out <= out - 8'd1;
    end else if (inc) begin
      out <= out + 8'd1;
    end // otherwise retain current PC
  end

endmodule
