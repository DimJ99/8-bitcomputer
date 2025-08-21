// ============================================================================
// cpu.sv — 8-bit microcoded CPU
// ----------------------------------------------------------------------------
// Orientation for ASIC/RTL newcomers
// • This core uses ONE shared 8-bit internal data bus (`bus`). Only one block
//   may drive it at a time. We explicitly qualify every potential driver with
//   a "drive enable" and then gate that enable with `=== 1'b1` to avoid
//   accidental multi-drives due to X/Z in simulation. In synthesis this
//   tri-stated bus is typically inferred as a mux.
// • The address bus (`addr_bus`) is separate and comes from the Memory Address
//   Register (MAR).
// • A simple 3-phase micro-cycle ring counter makes three non-overlapping
//   phases: cycle_clk → mem_clk → internal_clk.
// • Program Counter (PC) and Stack Pointer (SP) are 8-bit up/down counters.
// • The register file has one read port (can drive the shared bus) and one
//   write port (can capture from the shared bus).
// • The ALU updates flags (Zero/Carry). Some control flow uses those flags.
// • `cpu_ctrl` (FSM) decodes the current instruction/phase/handshake and raises
//   control strobes (e.g., register file enable, PC inc/load, etc.).
//
// Synthesis notes
// • Internal tri-states aren’t synthesized in most ASIC flows; expect a mux.
// • We treat `bus_ready` as “external memory is actually driving valid data.”
// • `mem_io` differentiates memory vs I/O cycles for your top-level memory map.
// ----------------------------------------------------------------------------
// ============================================================================

module cpu (
    input  logic       clk,
    input  logic       reset,
    output logic [7:0] addr_bus,
    output logic       c_ri,
    output logic       c_ro,
    output logic       mem_clk,
    output logic       mem_io,   
    inout  tri   [7:0] bus
  );

logic        pc_enable;   // (declared; note: enable gating is handled via derived signals)
logic        pc_load;     // PC load (bus → PC)
logic        pc_dec;      // PC decrement (used by stack/flow)
logic        c_rfi, c_rfo; // register file write/read enables (sink/source for shared bus)

    // ---------------------------------------------------------------
    //  Opcode, state, ALU-mode, jump-condition, and register codes
    // ---------------------------------------------------------------
    localparam [7:0]
      OP_NOP  = 8'b00_000_000, //no operation
      OP_CALL = 8'b00_000_001, //pushes current PC onto stack
      OP_RET  = 8'b00_000_010, // pops return address and resumes execution 
      OP_OUT  = 8'b00_000_011, // sends data out, useless rn
      OP_IN   = 8'b00_000_100, // similar sends input data, 
      //useless until i decide to wire to FPGA
      OP_HLT  = 8'b00_000_101, // stop execution
      OP_CMP  = 8'b00_000_110, // Subtracts two values (typically registers) and sets CPU flags (Zero, Negative, Carry, etc.), 
      // but does not store the result. Used for conditional jumps.
      OP_LDI  = 8'b00_010_000, // Load value into register
      OP_JMP  = 8'b00_011_000, // sets PC to new address
      OP_PUSH = 8'b00_100_000,  //saves a register onto stack
      OP_POP  = 8'b00_101_000, // restores top from stack onto reg
      OP_ALU  = 8'b01_000_000, //ALU add subtract inc dec and or
      OP_MOV  = 8'b10_000_000; // moves data between registers
// ^ Above encodes instruction classes in the top 2 bits and sub-ops in the fields.

localparam [7:0]
  // Micro-states for the control FSM. These sequence each instruction.
  STATE_NEXT         = 8'h00,
  STATE_FETCH_PC     = 8'h01, // put PC on the bus → MAR
  STATE_FETCH_INST   = 8'h02, // read instruction byte into IR
  STATE_HALT         = 8'h03,
  STATE_JUMP         = 8'h04, // conditional jump path (uses flags)
  STATE_OUT          = 8'h05, // I/O write cycle
  STATE_ALU_OUT      = 8'h06, // ALU result drives bus
  STATE_ALU_EXEC     = 8'h07, // ALU computes; flags update here
  STATE_MOV_STORE    = 8'h08, // MOV reg→mem store phase
  STATE_MOV_FETCH    = 8'h09, // MOV mem→reg address phase
  STATE_MOV_LOAD     = 8'h0A, // MOV mem→reg data phase
  STATE_FETCH_SP     = 8'h0C, // SP → bus (stack ops)
  STATE_PC_STORE     = 8'h0D, // write return PC (during CALL)
  STATE_TMP_JUMP     = 8'h0E, // helper
  STATE_RET          = 8'h0F, // return PC from stack (RET)
  STATE_INC_SP       = 8'h10, // POP post-increment
  STATE_SET_ADDR     = 8'h11, // helper: MAR load
  STATE_IN           = 8'h12, // I/O read cycle
  STATE_REG_STORE    = 8'h13, // PUSH: write to memory
  STATE_SET_REG      = 8'h14, // writeback into register file
  STATE_LOAD_IMM     = 8'h15, // LDI: drive immediate onto bus
  STATE_WAIT_FOR_RAM = 8'h16, // wait for external RAM to drive bus
  STATE_ALU_WRITEBACK= 8'h17, // ALU → REG_A architectural writeback
  STATE_FETCH_IMM    = 8'h19, // fetch immediate operand
  STATE_WAIT_IMM     = 8'h1A;  // wait for immediate to be valid (bus_ready)

    localparam [2:0]
      // ALU op encodings (used when instruction class is OP_ALU)
      ALU_ADD = 3'b000,
      ALU_SUB = 3'b001,
      ALU_INC = 3'b010,
      ALU_DEC = 3'b011,
      ALU_AND = 3'b100,
      ALU_OR  = 3'b101,
      ALU_XOR = 3'b110,
      ALU_ADC = 3'b111;

    localparam [2:0]
      // Jump condition codes (consumed by JMP sub-ops)
      JMP_JMP = 3'b000,
      JMP_JZ  = 3'b001,
      JMP_JNZ = 3'b010,
      JMP_JC  = 3'b011,
      JMP_JNC = 3'b100;

    localparam [2:0] REG_A = 3'b000, // canonical accumulator
                    REG_T = 3'b111; // scratch/pseudo-mem register used by MOV[mem] paths

    // ---------------------------------------------------------------
    //  Internal shared-bus framework (one driver per source)
    // ---------------------------------------------------------------
    // Each datapath block that *might* drive the bus presents:
    // • its data (`bus_from_*`) and
    // • a drive-enable (`bus_drive_*`).
    // Later, we gate those enables through "safe_*" to mask X/Z in sim.
    logic [7:0] bus_from_pc,  bus_from_sp,  bus_from_alu, bus_from_reg;
    logic       bus_drive_pc, bus_drive_sp, bus_drive_alu, bus_drive_reg;

    // ---------------------------------------------------------------
    //  Status flags, clocks and cycle generator
    // ---------------------------------------------------------------
    logic flag_zero, flag_carry; // Z and C flags are produced by the ALU

// 8 regs → 3-bit index for the register file ports
typedef logic [2:0] regid_t;

// Operand fields captured from the IR (instruction register).
// NOTE: SystemVerilog allows strong typing here; some tools prefer just `regid_t`.
// Keep as-is per your request (comments only).
wire regid_t operand_src = regi_out[5:3]; // bits [5:3]
wire regid_t operand_dst = regi_out[2:0]; // bits [2:0]
regid_t sel_in, sel_out; // register file write-port select / read-port select

    // 3-phase ring counter generates cycle_clk → mem_clk → internal_clk
    logic cycle_clk    = 0;
    logic internal_clk = 0;
    logic [3:0] cycle;         // (declared later in ctrl hookup, but used for visibility)
    logic [2:0] cnt    = 3'b100;
    logic halted       = 0;    // sticky latch for halt state (sim convenience)
    logic c_halt;              // goes high when FSM enters STATE_HALT

    always_ff @(posedge clk) begin
      if (!halted) begin
        {cycle_clk, mem_clk, internal_clk} <= cnt;
        case (cnt)
          3'b100: cnt <= 3'b010;
          3'b010: cnt <= 3'b001;
          3'b001: cnt <= 3'b100;
        endcase
      end
    end
    // ^ In ASIC synthesis, treat these as derived enables; ensure clock-gating
    //   methodology if converting to multi-phase clocks in silicon.

    // ---------------------------------------------------------------
    //  General-purpose registers
    // ---------------------------------------------------------------
    // One read port (sources the bus when c_rfo=1) + one write port
    // (captures the bus when c_rfi=1). A/B are hardwired views for ALU.
    logic [7:0] rega_out, regb_out, regs_out;

    cpu_reg m_registers (
  .clk           (internal_clk),
  .reset         (reset),    
  .data_in       (bus),       
  .sel_in        (sel_in),
  .sel_out       (sel_out),
  .enable_write  (c_rfi),
  .output_enable (c_rfo),
  .data_out      (regs_out),  
  .rega          (rega_out),
  .regb          (regb_out)
  );

    // ---------------------------------------------------------------
    //  Instruction register
    // ---------------------------------------------------------------
    // IR latches the fetched instruction byte. We also keep a stable copy
    // of the low 3 bits (`rd_q`) in the same instant we capture the IR so
    // that later states (which might change the bus) still have the original
    // destination field.
    logic [7:0] regi_out;
      logic       c_ii;   // IR load strobe when bus has a valid instruction byte

register m_regi (
  .in    (bus),
  .clk   (internal_clk),   
  .enable(c_ii),
  .reset (reset),
  .out   (regi_out)
);
logic [2:0] rd_q; // latched destination bits for non-ALU writebacks

always_ff @(posedge internal_clk or posedge reset) begin
  if (reset) rd_q <= '0;
  // Latch the low 3 bits (dst) at the moment IR is captured
  else if (state == STATE_FETCH_INST && c_ii) rd_q <= instruction[2:0];
  
end

// For ALU ops you always write REG_A by design; otherwise use latched rd
wire [2:0] wb_dst = (is_ALU) ? REG_A : rd_q;

    // ---------------------------------------------------------------
    //  Memory-address register (MAR)
    // ---------------------------------------------------------------
    // MAR holds the address that goes to external memory. For OUT cycles,
    // you force address 0 (simple I/O map example).
    logic c_mi;
 register m_mar (
 .in    ((state == STATE_OUT && is_OUT) ? 8'h00 : bus),
  .clk   (internal_clk),
  .enable(c_mi),
  .reset (reset),
  .out   (addr_bus)
);


    // ---------------------------------------------------------------
    //  Program counter
    // ---------------------------------------------------------------
    // PC can be loaded from the bus (jumps/calls), incremented (fetch/linear
    // execution), or decremented (stack/addressing patterns).
    logic [7:0] pc_out;
    logic       c_co, c_ci, c_j; // c_co: drive PC → bus; c_ci: increment; c_j: load/branch

 counter m_pc (
  .clk        (internal_clk),
  .reset      (reset),
  .load_value (bus),        
  .load       (pc_load),    
  .inc        (pc_inc),  
  .dec        (pc_dec),      
  .out        (pc_out)
);

    // ---------------------------------------------------------------
    //  Stack pointer
    // ---------------------------------------------------------------
    // SP resets to 0xFF. PUSH uses pre-decrement; POP uses post-increment.
    logic [7:0] sp_out;
    logic       c_si, c_sd, c_so;

counter m_sp (
  .clk        (internal_clk),
  .reset      (reset),   
  .load_value (8'hFF),     
  .load       (reset),     
  .inc        (c_si),
  .dec        (c_sd),
  .out        (sp_out)
);

// Group decodes (top 5 bits)
// ----- Instruction classes (top 2 bits) -----
// NOTE: `instruction` is a shorthand for `regi_out` declared later below.
// Many tools accept this forward reference; if yours does not, move the alias
// earlier. (Not changing here per your “comments-only” requirement.)
wire class00 = (instruction[7:6] == 2'b00);
wire class01 = (instruction[7:6] == 2'b01);
wire class10 = (instruction[7:6] == 2'b10);

// ----- Group decodes -----
// Class 01 → ALU, class 10 → MOV, class 00 → control/LDI/JMP/PUSH/POP/...
wire is_ALU  = class01;                               // 01_op_dst
wire is_MOV  = class10;                               // 10_src_dst
wire is_LDI  = class00 && (instruction[5:3] == 3'b010);
wire is_JMP  = class00 && (instruction[5:3] == 3'b011);
wire is_PUSH = class00 && (instruction[5:3] == 3'b100);
wire is_POP  = class00 && (instruction[5:3] == 3'b101);

// ----- Single, exact 00_000_xxx opcodes -----
wire is_NOP  = (instruction == OP_NOP );
wire is_CALL = (instruction == OP_CALL);
wire is_RET  = (instruction == OP_RET );
wire is_OUT  = (instruction == OP_OUT );
wire is_IN   = (instruction == OP_IN  );
wire is_HLT  = (instruction == OP_HLT );
wire is_CMP  = (instruction == OP_CMP );

// ----- ALU op field and selection -----
// For ALU class, take op from instruction[5:3]. For CMP, reuse SUB to set flags,
// and otherwise default is don't-care when ALU disabled.
wire [2:0] op_field = instruction[5:3];
assign alu_op =
  is_ALU ? op_field :
  is_CMP ? ALU_SUB  : ALU_ADD;  // default doesn't matter; ALU disabled outside ALU/CMP

    wire       c_eo_alu; // ALU bus-drive enable (derived from state)
    logic c_ee;          // ALU execute/enable (computes/updates flags)
    logic [7:0] alu_out;
    logic [2:0] alu_op;

    alu m_alu (
      .clk        (internal_clk),
      .enable     (c_ee),
      .reset      (reset),
      .in_a       (rega_out),
      .in_b       (regb_out),
      .out        (alu_out),
      .op         (alu_op),
      .flag_zero  (flag_zero),
      .flag_carry (flag_carry)
    );

    assign c_eo_alu = (state == STATE_ALU_OUT || state == STATE_ALU_WRITEBACK);
    // ALU can legally drive the bus only when putting result out or during writeback.

  // ---------------------------------------------------------------
  //  Immediate value register for LDI
  // ---------------------------------------------------------------
  // Two-cycle LDI: fetch immediate from memory, then drive it onto the bus
  // for register writeback.
  logic [7:0] imm_out;
  logic       c_ie; // load enable for immediate register (only when RAM is driving valid data)

  register m_imm (
    .in     (bus),
    .clk    (internal_clk),
    .enable (c_ie),
    .reset  (reset),
    .out    (imm_out)
  );

  // Mask X/Z on enable lines so only definite 1 drives bus in sim.
  // (Helps catch unintended multi-drives in waveforms/logs.)
  logic safe_bus_drive_pc;
  logic safe_bus_drive_sp;
  logic safe_bus_drive_alu;
  logic safe_bus_drive_reg;

  // ------------------------------------------------------------------
  // Datapath -> shared-bus connections
  // ------------------------------------------------------------------
  assign bus_from_pc  = pc_out;
  assign bus_drive_pc = c_co;          // program counter out

  assign bus_from_sp  = sp_out;
  assign bus_drive_sp = c_so;          // stack pointer out

  assign bus_from_alu = alu_out;
  assign bus_drive_alu = c_eo_alu;         // ALU out
  logic c_eo_imm;
assign c_eo_imm = is_LDI && (state == STATE_LOAD_IMM); // LDI drives immediate onto the bus

  // Composite “any CPU datapath driving?” — used later in debug mirrors
  wire c_eo;

  assign bus_from_reg = regs_out;
  assign bus_drive_reg = c_rfo;        // register file out

// --------------------------------------------
// Safe Bus Driver So shit doesnt get overwritten
// --------------------------------------------
  assign safe_bus_drive_pc  = (bus_drive_pc  === 1'b1);
  assign safe_bus_drive_sp  = (bus_drive_sp  === 1'b1);
  assign safe_bus_drive_alu = (bus_drive_alu === 1'b1);
  assign safe_bus_drive_reg = (bus_drive_reg === 1'b1);
  logic [7:0] bus_from_imm;
  logic       bus_drive_imm;
  logic       safe_bus_drive_imm;

  assign bus_from_imm  = imm_out;
assign bus_drive_imm = (is_LDI && (state == STATE_LOAD_IMM));
  assign safe_bus_drive_imm = (bus_drive_imm === 1'b1);

// MUX BUS
// Final shared bus select. In ASIC synthesis this tri bus becomes a mux tree.
// Priority order here: PC > SP > ALU > REG > IMM (only one should be true).
assign bus =
       (safe_bus_drive_pc  ? bus_from_pc  :
        safe_bus_drive_sp  ? bus_from_sp  :
        safe_bus_drive_alu ? bus_from_alu :
        safe_bus_drive_reg ? bus_from_reg :
        safe_bus_drive_imm ? bus_from_imm :
                             8'hZZ);

// Immediate capture happens when RAM is driving the bus and we’re in WAIT_IMM.
assign c_ie = (state == STATE_WAIT_IMM) && bus_ready;


    //  ---------------------------------------------------------------
    //  Control logic & FSM (unchanged except for regs_out path)
    //  ---------------------------------------------------------------
// IR shorthand (used by many decodes above). See note about forward reference.
wire [7:0] instruction = regi_out;

// handy shorthand
wire push_write = (state == STATE_REG_STORE) || (state == STATE_MEM_WRITE);

// External RAM convenience wires (abstracted here; typically hooked to a RAM macro)
logic       ram_we, ram_oe;
logic [7:0] ram_din;

// Write-enable the RAM for BOTH cycles of the push
assign ram_we = push_write;

// RAM must NOT drive the bus while we're writing
assign ram_oe = !( // only enable during reads/fetches
    state == STATE_FETCH_INST   ||
    state == STATE_WAIT_FOR_RAM ||
    state == STATE_WAIT_IMM
);

// Data to RAM comes from the system bus (driven by the source register)
assign ram_din = bus;

// Bus is ready when external RAM is enabled and (for memory cycles) is actually
// driving a non-Z value on the bus.
logic bus_ready;
assign bus_ready = (c_ro && !mem_io && (bus !== 8'hZZ));

    logic [7:0] opcode; // latched/decoded opcode for visibility
    logic [7:0] state;  // FSM state
    // `cycle` declared above with clocks; ctrl fills it for debug
assign operand1 = regi_out[5:3]; // operand fields (secondary shorthand)
assign operand2 = regi_out[2:0];
    logic       next_state = (state == STATE_NEXT) | reset; // helper visibility

    // mem_io high → I/O cycle; low → memory cycle
    assign mem_io = (state == STATE_OUT || state == STATE_IN);

    // MOV uses REG_T (alias 111) to denote memory ref in either src or dst
    logic mov_memory   = (operand1 == 3'b111 || operand2 == 3'b111);

    // Jump condition resolution (consumes ALU flags)
    logic jump_allowed = (operand2 == JMP_JMP) ||
                        ((operand2 == JMP_JZ)  && flag_zero) ||
                        ((operand2 == JMP_JNZ) && !flag_zero) ||
                        ((operand2 == JMP_JC)  && flag_carry) ||
                        ((operand2 == JMP_JNC) && !flag_carry);

// Register-file write-port select (who captures the bus when c_rfi=1)
assign sel_in =  //egister-file write-port select.
       (is_MOV)            ? operand_dst :
       (is_ALU || is_IN)   ? REG_A       :
       (is_POP || is_LDI)  ? operand_dst :
       (is_CALL)           ? REG_T       : '0;

// Register-file read-port select (who drives the bus when c_rfo=1)
assign sel_out =  //register-file read-port select (drives bus).
       (is_MOV)  ? operand_src :
       (is_OUT)  ? REG_A       :
       (is_CALL) ? REG_T       :
       (is_PUSH) ? operand_src :
                   '0;

localparam [7:0] STATE_DEC_SP  = 8'h18; // pre-decrement for PUSH

   // IR load (Instruction-In) when fetching an instruction and the bus is ready.
   assign c_ii = (state == STATE_FETCH_INST) && bus_ready; 

// PC increment (Counter-Inc) after a byte is consumed: fetch, imm fetch, RET,
// allowed JUMP/TMP_JUMP, and MOV[mem] address phase.
assign c_ci 
//PC increment (Counter-Inc) after a byte is consumed: 
// fetch, imm fetch, RET, allowed JUMP/TMP_JUMP, MOV from memory.
= ((state == STATE_FETCH_INST && bus_ready) ||
               (state == STATE_WAIT_IMM   && bus_ready) ||
               (state == STATE_RET        && bus_ready) ||
               ((state == STATE_JUMP)     && jump_allowed && bus_ready) ||
               (state == STATE_TMP_JUMP   && bus_ready) ||
               ((state == STATE_MOV_FETCH) && mov_memory && bus_ready));

// PC → bus during address-formers and immediate fetches
assign c_co = 
//PC out to bus
//during FETCH_PC, PC_STORE, memory MOV fetch, and immediate fetch.
(state == STATE_FETCH_PC)
           || (state == STATE_PC_STORE)
           || (state == STATE_MOV_FETCH && mov_memory)
           || (state == STATE_FETCH_IMM);   

// Simple visibility print of MAR captures (no functional effect)
always_ff @(posedge internal_clk) if (c_mi)
  $display("[MAR] <= %02h", bus);

// One-shot checker: after capturing IR, compare to controller’s opcode
logic arm_compare;
always_ff @(posedge internal_clk or posedge reset) begin
  if (reset) arm_compare <= 1'b0;
  else if (state == STATE_FETCH_INST && c_ii) arm_compare <= 1'b1;
  else if (arm_compare) begin
    arm_compare <= 1'b0;
    if (instruction != opcode)
      $display("[WARN] IR/opcode mismatch: IR=%02h opcode=%02h", instruction, opcode);
  end
end


    assign c_halt = (state == STATE_HALT); //halted when in STATE_HALT
    assign c_j    = ((state == STATE_JUMP && jump_allowed) ||
                    state == STATE_RET ||
                    state == STATE_TMP_JUMP); //Jump / PC load strobe on allowed JUMP, RET, or TMP_JUMP.

// MAR load from bus during address-forming and immediate fetch
assign c_mi //MAR in (capture address from bus) during FETCH_PC, FETCH_SP, memory MOV fetch, FETCH_IMM.
= (state == STATE_FETCH_PC)
           || (state == STATE_FETCH_SP)
           || (state == STATE_MOV_FETCH && mov_memory)
           || (state == STATE_FETCH_IMM);   

// RAM read enable during instruction/data fetches and POP/RET/JUMP paths
assign c_ro = //RAM out / memory read enable during instruction fetches (FETCH_INST, WAIT_FOR_RAM, WAIT_IMM), MOV load from memory, the special MOV store case 
// when operand2==3'b111, allowed JUMP, RET, and POP path in STATE_SET_REG.
       (state == STATE_FETCH_INST)     ||
       (state == STATE_WAIT_FOR_RAM)   ||
       (state == STATE_WAIT_IMM)       ||
       ((state == STATE_MOV_LOAD)  && mov_memory) ||
       ((state == STATE_MOV_STORE) && (operand2 == 3'b111)) ||
       (state == STATE_JUMP && jump_allowed) ||
       (state == STATE_RET)            ||
       ((state == STATE_SET_REG) && is_POP); 

localparam [7:0] STATE_MEM_WRITE = 8'h1B; // write-hold to satisfy memory timing

// RAM write enable for MOV→[mem], PUSH, write-hold, and CALL PC store
assign c_ri = //RAM in / memory write enable when writing memory: MOV to [mem] 
//(operand1==3'b111), PUSH (STATE_REG_STORE), the synchronous write hold
// (STATE_MEM_WRITE), and PC_STORE (e.g., CALL saves return addr).
       ((state == STATE_MOV_STORE) && (operand1 == 3'b111)) 
    ||  (state == STATE_REG_STORE)                          // PUSH write
    ||  (state == STATE_MEM_WRITE)                          // hold for RAM edge
    ||  (state == STATE_PC_STORE);                          // CALL                      

// Stack discipline: expose SP on bus / pre-dec / post-inc at the right times
//c_so — SP out to bus during FETCH_SP. 
//c_sd — SP decrement (pre-decrement for PUSH) in STATE_DEC_SP.
//c_si — SP increment (post-increment for POP) in STATE_INC_SP.
assign c_so = (state == STATE_FETCH_SP);  
assign c_sd = (is_PUSH && state == STATE_DEC_SP);  
assign c_si = (is_POP  &&  state == STATE_INC_SP);                      

    // ALU computes/updates flags only in EXEC
    assign c_ee = (state == STATE_ALU_EXEC); //ALU execute/enable during STATE_ALU_EXEC.

// ------------------------------------------------------------------
// Hook up the microcoded controller (sequencer)
// ------------------------------------------------------------------
// NOTE: `c_eo` is unused for ALU (we derive ALU bus-drive from state),
// but we still wire it for completeness/visibility.
always_ff @(posedge internal_clk) begin
  if (reset) $display("[SP RESET] sp_out <= %02h", sp_out);
  if (state == STATE_FETCH_SP)
    $display("[SP] out=%02h inc=%b dec=%b (next dec/inc applied after this edge)",
             sp_out, c_si, c_sd);
end
wire unused_pc_inc;
cpu_ctrl m_ctrl (
  .clk(internal_clk),
  .reset_cycle(reset),
  .instruction(regi_out),
  .state(state),
  .cycle(cycle),
  .bus_ready(bus_ready),
  .opcode(opcode),
  .c_rfi(c_rfi),
  .c_eo(c_eo),
  .c_rfo(c_rfo),
  .pc_inc(pc_inc),
  .pc_load(pc_load),
  .jump_allowed(jump_allowed),
  .pc_dec(pc_dec)
);

// -------------------------------------------------
// DECODE BLOCK (visibility/printf style debug)
// -------------------------------------------------
always_ff @(posedge clk) begin
  if (state == STATE_LOAD_IMM && opcode == OP_LDI) begin
    $display("[LDI WRITE] R%0d <- 0x%02h | c_rfi=%b bus=%h imm_out=%h",
             operand2, imm_out, c_rfi, bus, imm_out);
  end
end


always_ff @(posedge internal_clk) begin
  if (c_ri && !c_ro && !mem_io)
    $display("[RAM WRITE] @ %02h <= %02h", addr_bus, bus);
end
    always_ff @(posedge c_halt) halted <= 1;
    always_ff @(posedge clk) begin
    if (state == STATE_FETCH_INST) begin
      $display("[DEBUG] FETCH_INST: bus = 0x%0h, c_ii = %0b", bus, c_ii);
    end
  end

  always_ff @(posedge clk) begin
    if (c_ie) begin
    $display("[IMM LOAD] Immediate value 0x%0h loaded into imm register", bus);
  end

    $display("[CPU] PC: %0h, Instruction: %0h, State: %0d", pc_out, instruction, state);
 
  if (state == STATE_FETCH_INST) begin
    $display("[FETCH]  PC: 0x%0h | Bus: 0x%0h | c_ii: %b | c_ro: %b", pc_out, bus, c_ii, c_ro);
  end
$display("[CTRL EN] c_rfi=%b c_rfo=%b c_co=%b c_so=%b c_eo=%b", 
  c_rfi, c_rfo, c_co, c_so, c_eo);


  if (c_ii) begin
    $display("[IR LOAD] Instruction register set to 0x%0h from bus", bus);
  end
    $display("BUS DRIVER CHECK: pc=%b sp=%b alu=%b reg=%b imm=%b",
              safe_bus_drive_pc, safe_bus_drive_sp, safe_bus_drive_alu, safe_bus_drive_reg, safe_bus_drive_imm);
 
  $display("[STATE] PC: 0x%0h | State: %0h | Opcode: %0h | Inst: 0x%0h", pc_out, state, opcode, instruction);

 
  $display("[DECODE] operand1 = %0d (%0b), operand2 = %0d (%0b)", operand1, operand1, operand2, operand2);
  $display("[BUS CHECK] time=%0t | state=%0h | bus=%0h", $time, state, bus);
  $display("[BUS EN] PC:%b SP:%b ALU:%b REG:%b", 
    safe_bus_drive_pc, safe_bus_drive_sp, safe_bus_drive_alu, safe_bus_drive_reg);
  $display("[CTRL EN] c_co=%b c_so=%b c_eo=%b c_rfo=%b", 
    c_co, c_so, c_eo, c_rfo);
  $display("[ALU WRITEBACK] Writing ALU result to R%0d", operand1);

    if (state == STATE_SET_REG) begin
    $display("[SET_REG] Writing 0x%0h to reg[%0d] from bus=0x%0h",
            bus, sel_in, bus);

    $display("[IMM DEBUG] imm_out = 0x%0h, bus_drive_imm = %b", imm_out, bus_drive_imm);
    
  end
  
    $display("[RESET] time=%0t reset=%b state(before)=%0h", $time, reset, state);
    $display("[BUS DRV] c_co=%b c_so=%b c_eo=%b c_rfo=%b => bus=%0h", 
              c_co, c_so, c_eo, c_rfo, bus);  $display("[CTRL FSM] OUT state = %0h", state);
                  $display("[STATE_CHANGE] %0t, %0h", $time, state);
  $display("[CPU DATA] IR = %h, bus = %h, c_ii = %b", regi_out, bus, c_ii);
    if (c_ie) $display("[IMM REG DEBUG] bus=%h -> imm_out=%h", bus, imm_out);
    if (state == STATE_ALU_WRITEBACK) begin
    $display("[ALU EXEC] rega=%h, regb=%h, op=%b", rega_out, regb_out, alu_op);
      if (state == STATE_WAIT_FOR_RAM) begin
    $display("[DEBUG] WAIT_FOR_RAM | time = %0t | bus = %0h | bus_ready = %b", $time, bus, bus_ready);
        if (c_ii && state != STATE_FETCH_INST) begin
  $display("[BUG] c_ii is high outside of STATE_FETCH_INST! Current state: %h", state);
end
  end
  end
  end
always_ff @(posedge clk) begin
  if (state == STATE_ALU_WRITEBACK) begin
    $display("[ALU WRITEBACK] R%0d <= %02h  (dest=REG_A)", REG_A, alu_out);
  end
end
always_ff @(posedge internal_clk) begin
  if (is_LDI && state == STATE_LOAD_IMM && c_rfi === 1'b1) begin
    $display("[LDI WRITE] R%0d <= %02h", wb_dst, imm_out);
  end
end
always_ff @(posedge clk) begin
    if (c_ii) begin
        $display("[IR DEBUG] Before: regi_out=%h, bus=%h, c_ii=%b", regi_out, bus, c_ii);
        $display("[IR DEBUG] After next clock, regi_out should be %h", bus);
    end
end

logic [7:0] regs_mirror [0:7];
integer i;
wire cpu_dr = (c_co | c_so | c_eo | c_rfo | bus_drive_imm ); // do NOT include c_ii/IR
wire ram_dr = (!mem_io) && c_ro;
wire io_dr  = ( mem_io) && c_ro && (addr_bus == 8'h01); // only if that IO port sources the bus

always_ff @(posedge internal_clk or posedge reset) begin
  if (reset) begin
    for (i = 0; i < 8; i++) regs_mirror[i] <= 8'h00; 
  end else if (c_rfi === 1'b1) begin
    regs_mirror[sel_in] <= bus; 
  end
end

task automatic dump_regs;
  $display("REGS: A:%02h B:%02h C:%02h D:%02h E:%02h F:%02h G:%02h T:%02h",
           regs_mirror[0], regs_mirror[1], regs_mirror[2], regs_mirror[3],
           regs_mirror[4], regs_mirror[5], regs_mirror[6], regs_mirror[7]);
endtask

always_ff @(posedge internal_clk) begin
  if (c_rfi === 1'b1) begin
    $display("[REG WRITE] R%0d <= %02h (mirror)", sel_in, bus);
    dump_regs();
  end
end


always_ff @(posedge c_halt) begin
  $display("============================================");
  $display("CPU halted normally.");
  dump_regs();
end
always_ff @(posedge clk) if (instruction != opcode)
  $display("[WARN] IR/opcode mismatch: IR=%02h opcode=%02h", instruction, opcode);

always_ff @(posedge clk) if (state == STATE_FETCH_IMM)
  $display("[FETCH_IMM DEBUG] bus=%h bus_ready=%b c_ie=%b imm_out=%h",
           bus, bus_ready, c_ie, imm_out);

  endmodule
