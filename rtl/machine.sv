module machine (
  input  logic       clk,
  input  logic       reset
);

//
  initial begin
    #1000;
    $display("PC: %h, State: %h, Instruction: %h",
             m_cpu.pc_out, m_cpu.state, m_cpu.regi_out);
  end

  // ─────────────────────────────────────────────────────────────────────
  // CPU
  // ─────────────────────────────────────────────────────────────────────
  logic [7:0] addr_bus;
  tri   [7:0] bus;
  logic       c_ri, c_ro, mem_clk, mem_io;

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
  logic [7:0] io_out;
  assign io_out   = 8'hFF;

  always_ff @(posedge mem_clk) begin
    if (mem_io) begin
      if (addr_bus == 8'h00)
        $display("Output: %0d ($%0h)", bus, bus);
      else if (addr_bus == 8'h01)
        $display("Input: set $FF on data bus");
      else
        $display("Unknown I/O @ $%0h: %0d ($%0h)", addr_bus, bus, bus);
    end
  end

  // ─────────────────────────────────────────────────────────────────────
  // Bus arbitration
  // ─────────────────────────────────────────────────────────────────────
  // If mem_io (I/O cycle) and io_drive, drive bus from io_out.
assign bus = (mem_io && c_ro && (addr_bus == 8'h01)) ? 8'hFF : 8'hZZ;

endmodule
