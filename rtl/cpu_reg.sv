module cpu_reg (
  input  logic        clk,
  input  logic        reset,     // Synchronous reset (clears the whole file on the next clk)
  input  logic [7:0]  data_in,   // Write data (typically from the shared bus)
  input  logic [2:0]  sel_in,    // Which register to WRITE (0..7)
  input  logic [2:0]  sel_out,   // Which register to READ  (0..7)
  input  logic        enable_write,   // High → capture data_in into registers[sel_in]
  input  logic        output_enable,  // High → drive data_out with registers[sel_out]
  output logic [7:0]  data_out,  // Read port (sources bus when OE=1, else 'Z' in sim)
  output logic [7:0]  rega,      // Named views: A = R0
  output logic [7:0]  regb       //                B = R1
);

  // --------------------------------------------------------------------------
  // 8×8b register file storage
  // --------------------------------------------------------------------------
  // Indexing is 3 bits (0..7). If sel_in/sel_out go X/Z at sim time,
  // you may see X-propagation. The controller should keep selects stable.
  logic [7:0] registers [0:7];

  // --------------------------------------------------------------------------
  // Synchronous reset + single write port
  // --------------------------------------------------------------------------
  // • On reset: clear all 8 registers to 0x00 (synchronous style).
  // • Otherwise, when enable_write is high: write data_in to registers[sel_in].
  // Notes for ASIC newbies:
  //   - This is a classic 1W/1R register file built from flops.
  //   - The for-loop under reset is synthesizable; tools unroll it into resets.
  always_ff @(posedge clk) begin
    if (reset) begin
      // Explicitly initialize all registers to zero on reset
      for (int i = 0; i < 8; i++) begin
        registers[i] <= 8'h00;
      end
    end else if (enable_write) begin
      registers[sel_in] <= data_in;
    end
  end

  // Optional write trace (simulation aid)
  always_ff @(posedge clk) begin
    if (enable_write) begin
      $display("[REG WRITE] Register[%0d] <= %0h (from bus %0h)", sel_in, data_in, data_in);
    end
  end

  // --------------------------------------------------------------------------
  // Read port with simple output-enable
  // --------------------------------------------------------------------------
  // When output_enable=1, present registers[sel_out] on data_out.
  // When output_enable=0, drive 'Z' (helps catch unintended multi-drives
  // in simulation on a shared inout bus). In ASIC/FPGA synthesis, internal
  // tri-states are usually replaced by mux/AND gating.
  assign data_out = output_enable ? registers[sel_out] : 8'hZZ;

  // --------------------------------------------------------------------------
  // Named register views (A..G,T) for convenience / debug prints
  // --------------------------------------------------------------------------
  // R0→A, R1→B, R2→C, R3→D, R4→E, R5→F, R6→G, R7→T (temporary)
  logic [7:0] regc, regd, rege, regf, regg, regt;

  assign rega = registers[0];
  assign regb = registers[1];
  assign regc = registers[2];
  assign regd = registers[3];
  assign rege = registers[4];
  assign regf = registers[5];
  assign regg = registers[6];
  assign regt = registers[7];

  // Continuous snapshot print each cycle (can be noisy; great for bring-up)
  always_ff @(posedge clk) begin
    $display("[REG_DEBUG] A:%h, B:%h, C:%h, D:%h, E:%h, F:%h, G:%h, T:%h", 
             rega, regb, regc, regd, rege, regf, regg, regt);
  end

endmodule
