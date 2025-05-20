module machine (
  input  logic       clk,
  input  logic       reset
);

  // Debug at 1000 ns
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
  tri   [7:0] ram_bus;

  ram m_ram (
    .clk  (mem_clk),
    .addr (addr_bus),
    .o    (c_ri),     // write enable
    .oe   (c_ro),     // output enable
    .data (ram_bus)   // local tri-state net
  );

  // ─────────────────────────────────────────────────────────────────────
  // I/O peripheral
  // ─────────────────────────────────────────────────────────────────────
  logic [7:0] io_out;
  logic       io_drive;

  // drive from I/O only when mem_io is true and it's the right address
  assign io_drive = mem_io && addr_bus == 8'h01 && mem_clk;
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
  // Else if memory cycle (!mem_io) and RAM’s oe (c_ro), drive bus from ram_bus.
  // Otherwise tri-state.
  assign bus = (mem_io   && io_drive) ? io_out :
               (!mem_io  && c_ro)     ? ram_bus :
                                         8'hZZ;

endmodule
