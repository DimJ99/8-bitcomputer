module machine_tb;

  logic clk;
  logic enable_clk = 0;

  clock m_sys_clk (
    .enable(enable_clk),
    .clk(clk)
  );

  logic reset = 0;

  machine m_machine (
    .reset(reset),
    .clk(clk)
  );
  initial begin
    $readmemh("memory.list", m_machine.m_ram.mem);
    $dumpfile("machine.vcd");
    $dumpvars(0, m_machine);

    #10 reset = 1;
    #10 reset = 0;
    #10 enable_clk = 1;
  end

  always_ff @(posedge m_machine.m_cpu.halted) begin
    $display("============================================");
    $display("CPU halted normally.");
    $display(
      "REGISTERS: A: %h, B: %h, C: %h, D: %h, E: %h, F: %h, G: %h, Temp: %h",
      m_machine.m_cpu.m_registers.rega,
      m_machine.m_cpu.m_registers.regb,
      m_machine.m_cpu.m_registers.regc,
      m_machine.m_cpu.m_registers.regd,
      m_machine.m_cpu.m_registers.rege,
      m_machine.m_cpu.m_registers.regf,
      m_machine.m_cpu.m_registers.regg,
      m_machine.m_cpu.m_registers.regt
    );
  $display("MEM[0x00] = %h", m_machine.m_ram.mem[8'd00]);

    $stop;
  end
initial begin
  #500 $display("Simulation timed out!"); $finish;
end
endmodule
