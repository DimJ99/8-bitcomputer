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

// near the top of cpu.sv, alongside c_ri/c_ro etc.
logic        pc_enable;   // maps to counter.inc
logic        pc_load;     // maps to counter.load
logic        pc_dec;      // maps to counter.dec
// also make sure you’ve declared:
logic        c_rfi, c_rfo;

    // ─────────────────────────────────────────────────────────────
    //  Opcode, state, ALU‑mode, jump‑condition, and register codes
    // ─────────────────────────────────────────────────────────────
    localparam [7:0]
      OP_NOP  = 8'b00_000_000,
      OP_CALL = 8'b00_000_001,
      OP_RET  = 8'b00_000_010,
      OP_OUT  = 8'b00_000_011,
      OP_IN   = 8'b00_000_100,
      OP_HLT  = 8'b00_000_101,
      OP_CMP  = 8'b00_000_110,
      OP_LDI  = 8'b00_010_000,
      OP_JMP  = 8'b00_011_000,
      OP_PUSH = 8'b00_100_000,
      OP_POP  = 8'b00_101_000,
      OP_ALU  = 8'b01_000_000,
      OP_MOV  = 8'b10_000_000;

localparam [7:0]
  STATE_NEXT         = 8'h00,
  STATE_FETCH_PC     = 8'h01,
  STATE_FETCH_INST   = 8'h02,
  STATE_HALT         = 8'h03,
  STATE_JUMP         = 8'h04,
  STATE_OUT          = 8'h05,
  STATE_ALU_OUT      = 8'h06,
  STATE_ALU_EXEC     = 8'h07,
  STATE_MOV_STORE    = 8'h08,
  STATE_MOV_FETCH    = 8'h09,
  STATE_MOV_LOAD     = 8'h0A,
  STATE_FETCH_SP     = 8'h0C,
  STATE_PC_STORE     = 8'h0D,
  STATE_TMP_JUMP     = 8'h0E,
  STATE_RET          = 8'h0F,
  STATE_INC_SP       = 8'h10,
  STATE_SET_ADDR     = 8'h11,
  STATE_IN           = 8'h12,
  STATE_REG_STORE    = 8'h13,
  STATE_SET_REG      = 8'h14,
  STATE_LOAD_IMM     = 8'h15,
  STATE_WAIT_FOR_RAM = 8'h16,
  STATE_ALU_WRITEBACK= 8'h17,
  STATE_FETCH_IMM    = 8'h19,
  STATE_WAIT_IMM     = 8'h1A;   // <<< add this

    localparam [2:0]
      ALU_ADD = 3'b000,
      ALU_SUB = 3'b001,
      ALU_INC = 3'b010,
      ALU_DEC = 3'b011,
      ALU_AND = 3'b100,
      ALU_OR  = 3'b101,
      ALU_XOR = 3'b110,
      ALU_ADC = 3'b111;

    localparam [2:0]
      JMP_JMP = 3'b000,
      JMP_JZ  = 3'b001,
      JMP_JNZ = 3'b010,
      JMP_JC  = 3'b011,
      JMP_JNC = 3'b100;

    localparam [2:0] REG_A = 3'b000,
                    REG_T = 3'b111;

    // ─────────────────────────────────────────────────────────────
    //  Internal shared‑bus framework (one driver per source)
    // ─────────────────────────────────────────────────────────────
    logic [7:0] bus_from_pc,  bus_from_sp,  bus_from_alu, bus_from_reg;
    logic       bus_drive_pc, bus_drive_sp, bus_drive_alu, bus_drive_reg;

    // ─────────────────────────────────────────────────────────────
    //  Status flags, clocks and cycle generator
    // ─────────────────────────────────────────────────────────────
    logic flag_zero, flag_carry;

// 8 regs → 3-bit index
typedef logic [2:0] regid_t;

// From the instruction register (IR)
wire regid_t operand_src = regi_out[5:3]; // bits [5:3]
wire regid_t operand_dst = regi_out[2:0]; // bits [2:0]
regid_t sel_in, sel_out;

    logic cycle_clk    = 0;
    logic internal_clk = 0;
    logic [2:0] cnt    = 3'b100;
    logic halted       = 0;
    logic c_halt;

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

    // ─────────────────────────────────────────────────────────────
    //  General‑purpose registers
    // ─────────────────────────────────────────────────────────────
    logic [7:0] rega_out, regb_out, regs_out;

    cpu_reg m_registers (
  .clk           (internal_clk),
  .reset         (reset),       // Make sure reset is connected
  .data_in       (bus),         // read from bus
  .sel_in        (sel_in),
  .sel_out       (sel_out),
  .enable_write  (c_rfi),
  .output_enable (c_rfo),
  .data_out      (regs_out),    // This is used for bus connection below
  .rega          (rega_out),
  .regb          (regb_out)
  );

    // ─────────────────────────────────────────────────────────────
    //  Instruction register
    // ─────────────────────────────────────────────────────────────
    logic [7:0] regi_out;
      logic       c_ii;

register m_regi (
  .in    (bus),
  .clk   (internal_clk),   
  .enable(c_ii),
  .reset (reset),
  .out   (regi_out)
);
logic [2:0] rd_q;

always_ff @(posedge internal_clk or posedge reset) begin
  if (reset) rd_q <= '0;
  // Latch the low 3 bits (dst) at the moment IR is captured
  else if (state == STATE_FETCH_INST && c_ii) rd_q <= instruction[2:0];
  
end

// For ALU ops you always write REG_A by design; otherwise use latched rd
wire [2:0] wb_dst = (is_ALU) ? REG_A : rd_q;
    // ─────────────────────────────────────────────────────────────
    //  Memory‑address  
    // ─────────────────────────────────────────────────────────────
    logic c_mi;
 register m_mar (
 .in    ((state == STATE_OUT && is_OUT) ? 8'h00 : bus),
  .clk   (internal_clk),
  .enable(c_mi),
  .reset (reset),
  .out   (addr_bus)
);


    // ─────────────────────────────────────────────────────────────
    //  Program counter
    // ─────────────────────────────────────────────────────────────
    logic [7:0] pc_out;
    logic       c_co, c_ci, c_j;

 counter m_pc (
  .clk        (internal_clk),
  .reset      (reset),
  .load_value (bus),         // when load==1, PC ← bus
  .load       (pc_load),     // assert in JMP/CALL/RET states
  .inc        (pc_inc),   // assert in FETCH_PC & LOAD_IMM
  .dec        (pc_dec),      // assert on RET/POP
  .out        (pc_out)
);

    // ─────────────────────────────────────────────────────────────
    //  Stack pointer
    // ─────────────────────────────────────────────────────────────
    logic [7:0] sp_out;
    logic       c_si, c_sd, c_so;

counter m_sp (
  .clk        (internal_clk),
  .reset      (1'b0),       // SP never asynchronously resets here
  .load_value (8'hFF),      // initial SP on reset
  .load       (reset),      // sync load on reset
  .inc        (c_si),
  .dec        (c_sd),
  .out        (sp_out)
);

// Group decodes (top 5 bits)
// Decode off the IR (regi_out), not the controller's opcode latch
wire is_LDI  = (instruction[7:3] == OP_LDI [7:3]);
wire is_MOV  = (instruction[7:3] == OP_MOV [7:3]);
wire is_ALU  = (instruction[7:3] == OP_ALU [7:3]);
wire is_POP  = (instruction[7:3] == OP_POP [7:3]);
wire is_IN   = (instruction[7:3] == OP_IN  [7:3]);
wire is_CALL = (instruction[7:3] == OP_CALL[7:3]);
// Single-opcode exact (bit pattern is unique)
wire is_OUT  = (instruction == OP_OUT);
    wire       c_eo_alu;
    logic c_ee;
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

  // ─────────────────────────────────────────────────────────────
  //  Immediate value register for LDI
  // ─────────────────────────────────────────────────────────────
  logic [7:0] imm_out;
  logic       c_ie;

  register m_imm (
    .in     (bus),
    .clk    (internal_clk),
    .enable (c_ie),
    .reset  (reset),
    .out    (imm_out)
  );
  // Mask X/Z on enable lines so only definite 1 drives bus
  logic safe_bus_drive_pc;
  logic safe_bus_drive_sp;
  logic safe_bus_drive_alu;
  logic safe_bus_drive_reg;
  // ------------------------------------------------------------------
  // Datapath ➜ shared-bus connections
  // ------------------------------------------------------------------
  assign bus_from_pc  = pc_out;
  assign bus_drive_pc = c_co;          // program counter out

  assign bus_from_sp  = sp_out;
  assign bus_drive_sp = c_so;          // stack pointer out

  assign bus_from_alu = alu_out;
  assign bus_drive_alu = c_eo_alu;         // ALU out
  logic c_eo_imm;
assign c_eo_imm = is_LDI && (state == STATE_LOAD_IMM);





  wire c_eo;

  assign bus_from_reg = regs_out;
  assign bus_drive_reg = c_rfo;        // register file out

  assign safe_bus_drive_pc  = (bus_drive_pc  === 1'b1);
  assign safe_bus_drive_sp  = (bus_drive_sp  === 1'b1);
  assign safe_bus_drive_alu = (bus_drive_alu === 1'b1);
  assign safe_bus_drive_reg = (bus_drive_reg === 1'b1);
  logic [7:0] bus_from_imm;
  logic       bus_drive_imm;
  logic       safe_bus_drive_imm;

  assign bus_from_imm  = imm_out;
  assign bus_drive_imm = is_LDI && (state == STATE_LOAD_IMM);
  assign safe_bus_drive_imm = (bus_drive_imm === 1'b1);

assign bus =
       (bus_drive_pc  ? bus_from_pc  :
        bus_drive_sp  ? bus_from_sp  :
        bus_drive_alu ? bus_from_alu :
        bus_drive_reg ? bus_from_reg :
        bus_drive_imm ? bus_from_imm :
                        8'hZZ);
assign c_ie = (state == STATE_WAIT_IMM) && bus_ready;




    // ─────────────────────────────────────────────────────────────
    //  Control logic & FSM (unchanged except for regs_out path)
    // ─────────────────────────────────────────────────────────────
wire [7:0] instruction = regi_out;

// Bus is ready when someone (RAM) has driven a valid value
logic bus_ready;
assign bus_ready = (bus !== 8'hZZ);

    logic [7:0] opcode;
    logic [7:0] state;
    logic [3:0] cycle;
assign operand1 = regi_out[5:3];
assign operand2 = regi_out[2:0];

    logic       next_state = (state == STATE_NEXT) | reset;

    assign mem_io = (state == STATE_OUT || state == STATE_IN);

    logic mov_memory   = (operand1 == 3'b111 || operand2 == 3'b111);
    logic jump_allowed = (operand2 == JMP_JMP) ||
                        ((operand2 == JMP_JZ)  && flag_zero) ||
                        ((operand2 == JMP_JNZ) && !flag_zero) ||
                        ((operand2 == JMP_JC)  && flag_carry) ||
                        ((operand2 == JMP_JNC) && !flag_carry);
assign alu_op =
  is_ALU ? instruction[5:3] :
  (instruction[7:3] == OP_CMP[7:3]) ? ALU_SUB : 3'b000;

assign sel_in =
       (is_MOV)            ? operand_dst :
       (is_ALU || is_IN)   ? REG_A       :
       (is_POP || is_LDI)  ? operand_dst :
       (is_CALL)           ? REG_T       : '0;

assign sel_out =
       (is_MOV)            ? operand_src :
       (is_OUT)            ? REG_A       :
       (is_CALL)           ? REG_T       : '0;


   assign c_ii = (state == STATE_FETCH_INST) && bus_ready;
assign c_ci = ((state == STATE_FETCH_INST && bus_ready) ||
               (state == STATE_WAIT_IMM   && bus_ready) ||
               (state == STATE_RET        && bus_ready) ||
               ((state == STATE_JUMP)     && jump_allowed && bus_ready) ||
               (state == STATE_TMP_JUMP   && bus_ready) ||
               ((state == STATE_MOV_FETCH) && mov_memory && bus_ready));

assign c_co = (state == STATE_FETCH_PC)
           || (state == STATE_PC_STORE)
           || (state == STATE_MOV_FETCH && mov_memory)
           || (state == STATE_FETCH_IMM);   
always_ff @(posedge internal_clk) if (c_mi)
  $display("[MAR] <= %02h", bus);

// Optional: keep the warning but time it correctly
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


    assign c_halt = (state == STATE_HALT);
    assign c_j    = ((state == STATE_JUMP && jump_allowed) ||
                    state == STATE_RET ||
                    state == STATE_TMP_JUMP);

assign c_mi = (state == STATE_FETCH_PC)
           || (state == STATE_FETCH_SP)
           || (state == STATE_MOV_FETCH && mov_memory)
           || (state == STATE_FETCH_IMM);   
assign c_ro =
       (state == STATE_FETCH_INST)     ||
       (state == STATE_WAIT_FOR_RAM)   ||  // <— keep asserted while waiting for opcode
       (state == STATE_WAIT_IMM)       ||  // <— keep asserted while waiting for immediate
       ((state == STATE_MOV_LOAD)  && mov_memory) ||
       ((state == STATE_MOV_STORE) && (operand2 == 3'b111)) ||
       (state == STATE_JUMP && jump_allowed) ||
       (state == STATE_RET);            


// …after state/opcode/operand wiring…

assign c_ri =
       ((state == STATE_MOV_STORE) && (operand1 == 3'b111)) // MOV r -> [mem]
    ||  (state == STATE_REG_STORE)                          // PUSH
    ||  (state == STATE_PC_STORE);                          // CALL

    assign c_so = (state == STATE_FETCH_SP);
    assign c_sd = (state == STATE_TMP_JUMP || state == STATE_REG_STORE);
    assign c_si = (state == STATE_TMP_JUMP || state == STATE_REG_STORE || state == STATE_INC_SP);
    assign c_ee = (state == STATE_ALU_EXEC);


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
always_ff @(posedge clk) begin
  if (state == STATE_LOAD_IMM && opcode == OP_LDI) begin
    $display("[LDI WRITE] R%0d <- 0x%02h | c_rfi=%b bus=%h imm_out=%h",
             operand2, imm_out, c_rfi, bus, imm_out);
  end
end


    // Halt flip‑flop

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
    // Shows what's on the bus during instruction fetch
  if (state == STATE_FETCH_INST) begin
    $display("[FETCH]  PC: 0x%0h | Bus: 0x%0h | c_ii: %b | c_ro: %b", pc_out, bus, c_ii, c_ro);
  end
$display("[CTRL EN] c_rfi=%b c_rfo=%b c_co=%b c_so=%b c_eo=%b", 
  c_rfi, c_rfo, c_co, c_so, c_eo);

  // Print when instruction register updates
  if (c_ii) begin
    $display("[IR LOAD] Instruction register set to 0x%0h from bus", bus);
  end
    $display("BUS DRIVER CHECK: pc=%b sp=%b alu=%b reg=%b imm=%b",
              safe_bus_drive_pc, safe_bus_drive_sp, safe_bus_drive_alu, safe_bus_drive_reg, safe_bus_drive_imm);
  // Decode state – show full FSM progression
  $display("[STATE] PC: 0x%0h | State: %0h | Opcode: %0h | Inst: 0x%0h", pc_out, state, opcode, instruction);

  // Show operand parsing
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
// ── Debug mirror of register file (A..T) ─────────────────────────
logic [7:0] regs_mirror [0:7];
integer i;

always_ff @(posedge internal_clk or posedge reset) begin
  if (reset) begin
    for (i = 0; i < 8; i++) regs_mirror[i] <= 8'h00; // avoid 'xx'
  end else if (c_rfi === 1'b1) begin
    regs_mirror[sel_in] <= bus; // mirror any writeback
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

// Also nice to see them at halt:
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