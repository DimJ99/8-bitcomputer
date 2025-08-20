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
    "REGISTERS(dec): A:%0d, B:%0d, C:%0d, D:%0d, E:%0d, F:%0d, G:%0d, Temp:%0d",
    $unsigned(m_machine.m_cpu.m_registers.rega),
    $unsigned(m_machine.m_cpu.m_registers.regb),
    $unsigned(m_machine.m_cpu.m_registers.regc),
    $unsigned(m_machine.m_cpu.m_registers.regd),
    $unsigned(m_machine.m_cpu.m_registers.rege),
    $unsigned(m_machine.m_cpu.m_registers.regf),
    $unsigned(m_machine.m_cpu.m_registers.regg),
    $unsigned(m_machine.m_cpu.m_registers.regt)
  );
  $display("MEM[0x00] = %0d", $unsigned(m_machine.m_ram.mem[8'd00]));
  $stop;
end
initial begin
  #2000 $display("Simulation timed out!"); $finish;
end
endmodule
