module machine (
  input  logic clk,
  input  logic reset
);
initial begin
  #1000;
  $display("PC: %h, State: %h, Instruction: %h", m_machine.m_cpu.pc_out, m_machine.m_cpu.state, m_machine.m_cpu.regi_out);
end

  // ==========================
  // CPU
  // ==========================
  logic [7:0] addr_bus;
  tri   [7:0] bus;
  logic       c_ri;
  logic       c_ro;
  logic       mem_clk;
  logic       mem_io;

  cpu m_cpu (
    .clk(clk),
    .reset(reset),
    .addr_bus(addr_bus),
    .bus(bus),
    .mem_clk(mem_clk),
    .c_ri(c_ri),
    .c_ro(c_ro),
    .mem_io(mem_io)
  );

  // ==========================
  // RAM
  // ==========================
  tri   [7:0] ram_bus;
  logic       ram_oe;
  
assign bus = (mem_io && io_drive) ? io_out :
             (!mem_io && ram_oe)  ? ram_bus :
             8'hZZ;

assign ram_bus = ram_oe ? bus : 8'hZZ;
assign ram_oe = c_ro;

  ram m_ram (
    .clk(mem_clk),
    .addr(addr_bus),
    .data(ram_bus),
    .o(c_ri),
    .oe(ram_oe)
  );

  // ==========================
  // DEBUG I/O PERIPHERAL
  // ==========================
  logic [7:0] io_out;
  logic       io_drive;
  
  always_ff @(posedge mem_clk) begin
    if (mem_io) begin
      if (addr_bus == 8'h00)
        $display("Output: %0d ($%0h)", bus, bus);
      else if (addr_bus == 8'h01)
        $display("Input: set $FF on data bus");
      else
        $display("Unknown I/O on address $%0h: %0d ($%0h)", addr_bus, bus, bus);
    end
  end

  // Only drive the bus from I/O when mem_io is HIGH (I/O access)
  assign io_drive = mem_io && mem_clk && addr_bus == 8'h01;
  assign io_out = 8'hFF;


endmodule