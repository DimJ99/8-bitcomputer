module ram (
  input  logic       clk,           // memory clock (driven by CPU's mem_clk phase)
  input  logic [7:0] addr,          // byte address from MAR
  input  logic       o,             // write enable (CPU→RAM). When 1, we WRITE.
  input  logic       oe,            // output enable (RAM→CPU). When 1, we may READ/drive.
  inout  tri  [7:0]  data           // shared 8-bit data bus (tri-stated when not driving)
);
  // ---------------------------------------------------------------------------
  // Byte-wide memory array: 256 entries of 8 bits (address range 0x00..0xFF)
  // ---------------------------------------------------------------------------
  logic [7:0] mem [0:255];

  // Small read buffer used to model a synchronous 1-cycle read latency.
  // On a READ cycle we capture mem[addr] into 'buffer' on clk, and then the
  // continuous assign (below) drives the bus with 'buffer' when oe=1 and o=0.
  logic [7:0] buffer;

  // ---------------------------------------------------------------------------
  // Initialization for simulation
  // ---------------------------------------------------------------------------
  // Loads hex bytes into mem[] at time 0. This is a *simulation-only* aid.
  // In FPGA/ASIC, you'd use an initialized BRAM or a compiled SRAM macro.
  initial begin
    $readmemh("memory.list", mem);
    $display("Memory initialized from memory.list");
  end

  // ---------------------------------------------------------------------------
  // Tri-state bus drive
  // ---------------------------------------------------------------------------
  // We only drive the bus during READ cycles (oe=1) AND when we are NOT writing
  // (o=0). This prevents contention with the CPU or other bus masters.
  // In synthesis, internal tri-states get turned into muxes; top-level tri on
  // an I/O pad is fine in simulation.
  assign data = (oe && !o) ? buffer : 8'hZZ;

  // ---------------------------------------------------------------------------
  // Synchronous write / synchronous read (1-cycle)
  // ---------------------------------------------------------------------------
  // • WRITE (o=1): sample 'data' from the external bus and store at mem[addr].
  //   We do not drive the bus in this case (see assign above).
  // • READ  (o=0): register mem[addr] into 'buffer' on this clock edge.
  //   The driven value appears on the bus *next* cycle (1-cycle latency)
  //   provided oe remains asserted.
  always_ff @(posedge clk) begin
    if (o) begin
      // WRITE: sample data from the external bus
      mem[addr] <= data;
      $display("Memory: set [0x%0h] => 0x%0h", addr, data);
    end else begin
      // READ: prefetch into buffer (1-cycle sync read)
      buffer <= mem[addr];
      $display("[RAM] READ   @ 0x%0h => 0x%0h", addr, mem[addr]);
    end
  end
endmodule
