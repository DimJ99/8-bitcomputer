// ============================================================================
// counter.sv — 8-bit up/down counter with synchronous load (used for PC/SP)
// ----------------------------------------------------------------------------
// • Asynchronous active-high reset clears the counter to 0 immediately.
// • One-cycle synchronous LOAD has priority over INC/DEC.
// • DEC has priority over INC if both are asserted (by design).
// • 8-bit arithmetic naturally wraps around (mod-256).
// • Fully synthesizable; uses non-blocking assignments in a single always_ff.
// ============================================================================

module counter (
  input  logic        clk,         // clock
  input  logic        reset,       // ASYNC active-high reset (clears immediately)
  input  logic [7:0]  load_value,  // value to load when 'load' is asserted
  input  logic        load,        // synchronous load enable (highest priority after reset)
  input  logic        inc,         // increment request (+1) — lower priority than 'dec'
  input  logic        dec,         // decrement request (-1) — wins if inc&dec both 1
  output logic [7:0]  out          // current count value
);

//---------------------------------------------------------------
// PC/SP counter core
//---------------------------------------------------------------
// Priority order (highest → lowest):
//   1) reset   : out ← 0           (asynchronous)
//   2) load    : out ← load_value  (synchronous)
//   3) dec     : out ← out - 1     (synchronous)
//   4) inc     : out ← out + 1     (synchronous)
//
// Notes:
// • Because 'out' is 8 bits, add/sub wraps around (e.g., 0-1 = 0xFF).
// • If both 'inc' and 'dec' are asserted in the same cycle, 'dec' takes effect.
// • Use clean, single-cycle pulses on load/inc/dec to avoid multiple updates.
// • For ASICs, ensure reset deassertion is timed/synchronized per methodology.
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      out <= 8'd0;            // async clear to 0x00
    end else if (load) begin
      out <= load_value;      // synchronous load
    end else if (dec) begin
      out <= out - 8'd1;      // count down
    end else if (inc) begin
      out <= out + 8'd1;      // count up
    end 
  end

endmodule
