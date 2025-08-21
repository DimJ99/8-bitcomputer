module machine (
  input  logic       clk,   // master clock (drives the CPU's 3-phase generator)
  input  logic       reset  // synchronous reset forwarded into the CPU
);

// -----------------------------------------------------------------------------
// Quick bring-up print
// -----------------------------------------------------------------------------
// Small one-shot probe that fires 1000 time units after sim start.
// It peeks into the CPU instance (hierarchical references) and dumps PC/state/IR.
// This is purely for visibility and has no functional impact.
  initial begin
    #1000;
    $display("PC: %h, State: %h, Instruction: %h",
             m_cpu.pc_out, m_cpu.state, m_cpu.regi_out);
  end

  // ─────────────────────────────────────────────────────────────────────
  // CPU
  // ─────────────────────────────────────────────────────────────────────
  // Shared address/data/control between CPU and memory/IO.
  // • addr_bus : 8-bit address driven by the CPU’s MAR.
  // • bus      : 8-bit bidirectional data bus (tri-stated when nobody drives).
  // • c_ri     : RAM write enable from CPU (CPU→RAM).
  // • c_ro     : RAM output enable from CPU (RAM→CPU).
  // • mem_clk  : memory-phase clock from CPU’s 3-phase generator.
  // • mem_io   : 1 = I/O cycle, 0 = memory cycle.
  logic [7:0] addr_bus;
  tri   [7:0] bus;
  logic       c_ri, c_ro, mem_clk, mem_io;

  // Instantiate the CPU core. All bus/interface signals are exposed so the
  // test harness (this module) can connect RAM and a simple I/O device.
  cpu m_cpu (
    .clk      (clk),
    .reset    (reset),
    .addr_bus (addr_bus),
    .bus      (bus),
    .mem_clk  (mem_clk),
    .c_ri     (c_ri),
    .c_ro     (c_ro),
    .mem_io   (mem_io)
  );

  // ─────────────────────────────────────────────────────────────────────
  // RAM
  // ─────────────────────────────────────────────────────────────────────
  // Simple RAM model/macro interface:
  // • clk  : driven by CPU’s memory phase (mem_clk).
  // • addr : address from CPU’s MAR.
  // • o    : write enable (active when CPU asserts c_ri during *memory* cycles).
  // • oe   : output enable (active when CPU asserts c_ro during *memory* cycles).
  // • data : shared tri-state data bus.
  //
  // Note: Internal tri-states in ASICs synthesize to muxes; at the top level of
  // a testbench like this, the tri bus is okay for simulation.
ram m_ram (
  .clk  (mem_clk),
  .addr (addr_bus),
  .o    (c_ri && !mem_io),   // write only on memory cycles
  .oe   (c_ro && !mem_io),   // read  only on memory cycles
  .data (bus)
);

  // ─────────────────────────────────────────────────────────────────────
  // I/O peripheral
  // ─────────────────────────────────────────────────────────────────────
  // Very tiny “peripheral” demo:
  //   • addr 0x00: treated as an OUTPUT port (we just $display the value on bus)
  //   • addr 0x01: treated as an INPUT  port (we drive 0xFF onto the bus)
  //
  // mem_io distinguishes I/O vs memory cycles. This block only logs; the actual
  // *driving* of the input value happens in the Bus arbitration section below.
  logic [7:0] io_out;
  assign io_out   = 8'hFF;  // constant input value presented by the “device”

  always_ff @(posedge mem_clk) begin
    if (mem_io) begin
      if (addr_bus == 8'h00)
        $display("Output: %0d ($%0h)", bus, bus); // OUT → print what CPU wrote
      else if (addr_bus == 8'h01)
        $display("Input: set $FF on data bus");   // IN  → we will drive 0xFF
      else
        $display("Unknown I/O @ $%0h: %0d ($%0h)", addr_bus, bus, bus);
    end
  end

  // ─────────────────────────────────────────────────────────────────────
  // Bus arbitration
  // ─────────────────────────────────────────────────────────────────────
  // The shared `bus` may be driven by RAM, the CPU’s internal sources, or an
  // I/O device. To avoid contention, each driver only asserts during its slot.
  //
  // Here we model the I/O *input* device (port 0x01):
  // • When the cycle is I/O (`mem_io==1`) AND the CPU requests a read (`c_ro==1`)
  //   AND the addressed port is 0x01, this peripheral drives 0xFF.
  // • Otherwise, it releases the bus (ZZ), allowing RAM/CPU to drive instead.
  //
  // TIP (debugging): If you ever see X on the bus during an IO read, check that
  // exactly one of RAM/io/CPU is enabled for that phase/state.
assign bus = (mem_io && c_ro && (addr_bus == 8'h01)) ? 8'hFF : 8'hZZ;

endmodule
