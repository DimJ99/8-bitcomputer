// ============================================================================
// register.sv — 8-bit load-enable register (used for MAR/IR/IMM/etc.)
// ----------------------------------------------------------------------------
// • Captures `in` on a rising clock edge when `enable` is 1.
// • Asynchronous active-high reset clears the output to 0 immediately.
// • Simple, 1-cycle pipeline element frequently used across the CPU datapath.
// • The $display inside the load branch is a sim-only trace (kept as-is).
// ============================================================================

module register(
  input wire [7:0] in,      // data to capture (typically the shared bus)
  input wire clk,           // clock
  input wire reset,         // ASYNC active-high reset (see always_ff sensitivity)
  input wire enable,        // load strobe: when 1 at clk rising edge, latch `in`
  output reg [7:0] out      // registered output
);

  // --------------------------------------------------------------------------
  // Storage element
  // --------------------------------------------------------------------------
  // • Asynchronous reset because sensitivity list is (posedge clk OR posedge reset).
  //   In ASIC flows, ensure reset is properly synchronized/deasserted to avoid
  //   metastability when coming out of reset.
  // • When `enable` is low, the register holds its previous value.
  // • `'0` zeros the vector width-safely (all bits to 0).
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      out <= '0;          // clear to 0x00 on reset
    end else if (enable) begin
      out <= in;          // capture new value on rising edge when enabled
      $display("[MAR] loading bus = %0h into addr_bus", in);
      // ^ sim trace: this instance often serves as MAR in the CPU, hence the tag
    end
  end

endmodule
