module cpu_ctrl (
    input  logic       clk,
    input  logic       reset_cycle,
    input  logic [7:0] instruction,
    output logic [7:0] state,
    output logic [3:0] cycle,
    input  logic       bus_ready,
    output logic [7:0] opcode,
    output logic       c_rfi,
    output logic       c_rfo,
    output logic       pc_inc,    // counter.inc  
    output logic       pc_load,   // counter.load 
    output logic       pc_dec,     // counter.dec  
    input  logic       jump_allowed
);

  // ─── Opcodes ─────────────────────────────────────────────
  localparam [7:0]
    OP_NOP   = 8'b00_000_000,
    OP_CALL  = 8'b00_000_001,
    OP_RET   = 8'b00_000_010,
    OP_OUT   = 8'b00_000_011,
    OP_IN    = 8'b00_000_100,
    OP_HLT   = 8'b00_000_101,
    OP_CMP   = 8'b00_000_110,
    OP_LDI   = 8'b00_010_000,
    OP_JMP   = 8'b00_011_000,
    OP_PUSH  = 8'b00_100_000,
    OP_POP   = 8'b00_101_000,
    OP_ALU   = 8'b01_000_000,
    OP_MOV   = 8'b10_000_000;
    

  // ─── States ──────────────────────────────────────────────
  localparam [7:0]
    STATE_NEXT          = 8'h00,
    STATE_FETCH_PC      = 8'h01,
    STATE_WAIT_FOR_RAM  = 8'h16,
    STATE_FETCH_INST    = 8'h02,
    STATE_HALT          = 8'h03,
    STATE_JUMP          = 8'h04,
    STATE_OUT           = 8'h05,
    STATE_ALU_OUT       = 8'h06,
    STATE_ALU_EXEC      = 8'h07,
    STATE_MOV_STORE     = 8'h08,
    STATE_MOV_FETCH     = 8'h09,
    STATE_MOV_LOAD      = 8'h0A,
    STATE_FETCH_SP      = 8'h0C,
    STATE_PC_STORE      = 8'h0D,
    STATE_TMP_JUMP      = 8'h0E,
    STATE_RET           = 8'h0F,
    STATE_INC_SP        = 8'h10,
    STATE_SET_ADDR      = 8'h11,
    STATE_IN            = 8'h12,
    STATE_REG_STORE     = 8'h13,
    STATE_SET_REG       = 8'h14,
    STATE_LOAD_IMM      = 8'h15,
    STATE_ALU_WRITEBACK = 8'h17,
    STATE_REG_READ      = 8'h18,
    STATE_FETCH_IMM     = 8'h19,
    STATE_MOV_MEM_ADDR = 8'h1C;

  logic [7:0] instruction_reg;
  logic [7:0] latched_opcode;

  // ─── Cycle Counter ───────────────────────────────────────
  initial cycle = 0;
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)
      cycle <= 0;
    else
      cycle <= (cycle == 7) ? 0 : cycle + 1;
  end

  // ─── Latch Instruction ───────────────────────────────────
always_ff @(posedge clk or posedge reset_cycle) begin
  if (reset_cycle) begin
    instruction_reg <= 8'h00;
  end else if (state == STATE_FETCH_INST && bus_ready) begin
    instruction_reg <= instruction;
  end
end

  // ─── Decode Opcode ───────────────────────────────────────
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)
      latched_opcode <= 8'h00;
    else if (cycle == 5) begin
      casez (instruction_reg)
        8'b00_000_000: latched_opcode <= OP_NOP;
        8'b00_000_001: latched_opcode <= OP_CALL;
        8'b00_000_010: latched_opcode <= OP_RET;
        8'b00_000_011: latched_opcode <= OP_OUT;
        8'b00_000_100: latched_opcode <= OP_IN;
        8'b00_000_101: latched_opcode <= OP_HLT;
        8'b00_000_110: latched_opcode <= OP_CMP;
        8'b00_010_???: latched_opcode <= OP_LDI;
        8'b00_011_???: latched_opcode <= OP_JMP;
        8'b00_100_???: latched_opcode <= OP_PUSH;
        8'b00_101_???: latched_opcode <= OP_POP;
        8'b01_???_???: latched_opcode <= OP_ALU;
        8'b10_???_???: latched_opcode <= OP_MOV;
        default:       latched_opcode <= OP_NOP;
      endcase
    end
  end

  // ─── Control FSM ─────────────────────────────────────────
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle) begin
      state <= STATE_FETCH_PC;
    end else if (state == STATE_HALT) begin
      state <= STATE_HALT;

    // Add MOV instruction progression
    end else if (state == STATE_MOV_FETCH) begin
      state <= STATE_MOV_LOAD;
    end else if (state == STATE_MOV_LOAD) begin
      state <= STATE_MOV_STORE;
    end else if (state == STATE_MOV_STORE) begin
      state <= STATE_FETCH_PC;

    // Existing ALU & memory operations
    end else if (state == STATE_ALU_WRITEBACK || state == STATE_PC_STORE || state == STATE_SET_REG ||
                 state == STATE_LOAD_IMM || state == STATE_REG_STORE) begin
      state <= STATE_FETCH_PC;

    end else if (state == STATE_WAIT_FOR_RAM && !bus_ready) begin
      state <= STATE_WAIT_FOR_RAM;
    end else if (state == STATE_REG_READ) begin
      state <= STATE_ALU_EXEC;

    // Instruction cycle-based state transitions
    end else begin
      case (cycle)
        4'd0: state <= STATE_FETCH_PC;
        4'd1: state <= STATE_WAIT_FOR_RAM;
        4'd2: state <= STATE_WAIT_FOR_RAM;
        4'd3: state <= STATE_FETCH_INST;
        4'd4: state <= STATE_NEXT;
        4'd5: state <= STATE_NEXT;
        4'd6: state <= (latched_opcode == OP_HLT)                             ? STATE_HALT        :
                       (latched_opcode == OP_MOV)                             ? STATE_MOV_FETCH   :
                       (latched_opcode == OP_ALU || latched_opcode == OP_CMP) ? STATE_REG_READ    :
                       (latched_opcode == OP_RET || latched_opcode == OP_POP) ? STATE_INC_SP      :
                       (latched_opcode == OP_PUSH)                            ? STATE_FETCH_SP    :
                       (latched_opcode == OP_CALL)                            ? STATE_SET_REG     :
                       (latched_opcode == OP_LDI)                             ? STATE_FETCH_IMM   :
                       (latched_opcode == OP_JMP)                             ? STATE_JUMP        :
                       (latched_opcode == OP_IN  || latched_opcode == OP_OUT) ? STATE_SET_ADDR    :
                                                                                STATE_NEXT;

        4'd7: state <= (latched_opcode == OP_LDI)        ? STATE_LOAD_IMM      :
                      (latched_opcode == OP_ALU)        ? STATE_ALU_WRITEBACK :
                      (latched_opcode == OP_PUSH)       ? STATE_REG_STORE     :
                      (latched_opcode == OP_CALL)       ? STATE_PC_STORE      :
                      (latched_opcode == OP_RET ||
                       latched_opcode == OP_POP)        ? STATE_FETCH_SP      :
                      (latched_opcode == OP_JMP)        ? STATE_FETCH_PC      :
                      (latched_opcode == OP_IN  ||
                       latched_opcode == OP_OUT)        ? STATE_NEXT          :
                                                           STATE_NEXT;

        default: state <= STATE_NEXT;
      endcase
    end

    // Debugging output
    $display("[CTRL FSM] Cycle=%0d | instruction = %h", cycle, instruction);
    $display("[CTRL FSM] instruction_reg = %h | latched_opcode = %h", instruction_reg, latched_opcode);
    $display("[FETCH]  cycle=%0d | IR=%h | latched_opcode=%h", cycle, instruction_reg, latched_opcode);
    $display("[FSM] transition | cycle=%0d | opcode=%h | state=%h", cycle, latched_opcode, state);
    if (cycle == 4 && !bus_ready)
      $display("[WARN] Bus not ready at cycle 4; instruction fetch may be invalid.");
    $display("[CTRL FSM] state = %h", state);
    if (cycle == 4) begin
      $display("[DEBUG] cycle=4, instruction=%h, bus_ready=%b", instruction, bus_ready);
    end
  end



  assign opcode = latched_opcode;

  // ─── Control Signal Generation ──────────────────────────
  always_comb begin
    c_rfi = 0;
    c_rfo = 0;
// PC increments on FETCH_PC or LOAD_IMM
pc_inc  = (state == STATE_FETCH_PC) || (state == STATE_LOAD_IMM);

// PC loads (sync) on JUMP/CALL/RET
pc_load = (state == STATE_JUMP    && jump_allowed)
        || (state == STATE_PC_STORE)
        || (state == STATE_RET)
        || (state == STATE_TMP_JUMP);

// PC decrements on RET/POP semantics
pc_dec  = (state == STATE_RET)
        || (state == STATE_FETCH_SP);
    case (state)
      STATE_ALU_WRITEBACK: c_rfi = 1;
      STATE_LOAD_IMM:      c_rfi = 1;
      STATE_IN:            c_rfi = 1;
      STATE_SET_ADDR:      c_rfi = 1;
      STATE_SET_REG:       c_rfi = 1;
      STATE_MOV_STORE:     if (instruction[5:3] != 3'b111) c_rfi = 1;
      STATE_REG_READ:      c_rfo = 1;
      default: begin
        c_rfi = 0;
        c_rfo = 0;
      end
    endcase
  end

endmodule
