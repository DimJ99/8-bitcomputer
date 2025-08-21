// ============================================================================
// clock.sv — tiny behavioral clock generator for simulation
// ----------------------------------------------------------------------------
// • Produces a square-ish wave on `clk` by toggling every #1 time unit when
//   `enable` is high. With `timescale 1ns/1ps`, that’s a ~2 ns period (500 MHz).
// • `clk_inv` is just the logical inverse of `clk` for convenience.
// • This is **testbench/simulation-only** style. Real ASIC/FPGA designs do not
//   synthesize free-running clocks from procedural toggles; they use PLLs/DCMs
//   and dedicated clock routing. Treat this module as a stimulator.
// • The unconditional `always begin ... end` loop is time-driven by `#1` so the
//   simulator advances even when `enable` is low (no toggle occurs then).
// • Duty cycle: when `enable` remains high, the loop toggles `clk` every #1,
//   yielding a ~50% duty cycle. If `enable` changes mid-iteration, the next
//   check applies on the next #1 boundary.
// • Tip: In your testbench, set an explicit `\`timescale` (e.g., `1ns/1ps`) so
//   the `#1` delay is well-defined.
// ============================================================================

module clock (
  input  logic enable, // when 1, allow the oscillator to toggle `clk`
  output logic clk,    // generated clock (behavioral)
  output logic clk_inv // inverted replica of `clk`
);

  // Start from a known state in simulation
  initial clk = 0;

  // Free-running time loop:
  // - Wait #1 time unit, then if enabled, invert clk.
  // - Keeps looping forever. When enable==0, clk holds its last value.
  // - This style is not synthesizable and is intended for simulation clocks.
  always begin
    #1; // time step (set by `timescale` in your sim)
    if (enable)
      clk = ~clk; // blocking assign is fine in a pure TB-style oscillator
  end

  // Convenience inverse. In sim, this updates combinationally with `clk`.
  assign clk_inv = ~clk;

endmodule
