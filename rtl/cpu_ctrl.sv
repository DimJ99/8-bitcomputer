module cpu_ctrl (
  input  logic       clk,
  input  logic       reset_cycle,
  input  logic [7:0] instruction,
  output logic [7:0] state,
  output logic [3:0] cycle,
  output logic [7:0] opcode
);
// FUCK THIS SHIT MAN
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
  localparam [7:0]
    STATE_NEXT        = 8'h00,
    STATE_FETCH_PC    = 8'h01,
    STATE_FETCH_INST  = 8'h02,
    STATE_HALT        = 8'h03,
    STATE_JUMP        = 8'h04,
    STATE_OUT         = 8'h05,
    STATE_ALU_OUT     = 8'h06,
    STATE_ALU_EXEC    = 8'h07,
    STATE_MOV_STORE   = 8'h08,
    STATE_MOV_FETCH   = 8'h09,
    STATE_MOV_LOAD    = 8'h0A,
    STATE_FETCH_SP    = 8'h0C,
    STATE_PC_STORE    = 8'h0D,
    STATE_TMP_JUMP    = 8'h0E,
    STATE_RET         = 8'h0F,
    STATE_INC_SP      = 8'h10,
    STATE_SET_ADDR    = 8'h11,
    STATE_IN          = 8'h12,
    STATE_REG_STORE   = 8'h13,
    STATE_SET_REG     = 8'h14;

  // Initialize cycle to 0
  initial begin
    cycle = 0;
  end

  // Reset cycle on rising edge of reset_cycle
  always_ff @(posedge reset_cycle) begin
    cycle <= 0;
  end
always_ff @(posedge clk) begin
  casez (instruction)
    8'b00_010_???: opcode <= OP_LDI;
    8'b10_???_???: opcode <= OP_MOV;
    8'b01_???_000: opcode <= OP_ALU;
    8'b00_011_???: opcode <= OP_JMP;
    8'b00_100_???: opcode <= OP_PUSH;
    8'b00_101_???: opcode <= OP_POP;
    default:        opcode <= instruction;
  endcase
end

  // Main state control logic
  always_ff @(posedge clk) begin
    // FSM state transitions based on cycle
    case (cycle)
      4'b0000: state <= STATE_FETCH_PC; // STATE_FETCH_PC
      4'b0001: state <= STATE_FETCH_INST; // STATE_FETCH_INST

      4'b0010: state <= (opcode == OP_HLT)                                ? STATE_HALT : // OP_HLT → STATE_HALT
                        (opcode == OP_MOV)                                ? STATE_MOV_FETCH : // OP_MOV → STATE_MOV_FETCH
                        (opcode == OP_ALU || opcode == OP_CMP)            ? STATE_ALU_EXEC : // OP_ALU or OP_CMP → STATE_ALU_EXEC
                        (opcode == OP_RET || opcode == OP_POP)            ? STATE_INC_SP : // OP_RET or OP_POP → STATE_INC_SP
                        (opcode == OP_PUSH)                               ? STATE_FETCH_SP : // OP_PUSH → STATE_FETCH_SP
                        (opcode == OP_IN || opcode == OP_OUT || 
                         opcode == OP_CALL || opcode == OP_LDI || 
                         opcode == OP_JMP)                                ? STATE_FETCH_PC : // IN, OUT, CALL, LDI, JMP → STATE_FETCH_PC
                                                                          STATE_NEXT; // STATE_NEXT

      4'b0011: state <= (opcode == OP_JMP)                                ? STATE_JUMP : // OP_JMP → STATE_JUMP
                        (opcode == OP_LDI)                                ? STATE_SET_REG : // OP_LDI → STATE_SET_REG
                        (opcode == OP_MOV)                                ? STATE_MOV_LOAD : // OP_MOV → STATE_MOV_LOAD
                        (opcode == OP_ALU)                                ? STATE_ALU_OUT : // OP_ALU → STATE_ALU_OUT
                        (opcode == OP_OUT || opcode == OP_IN)             ? STATE_SET_ADDR : // OUT or IN → STATE_SET_ADDR
                        (opcode == OP_PUSH)                               ? STATE_REG_STORE : // OP_PUSH → STATE_REG_STORE
                        (opcode == OP_CALL)                               ? STATE_SET_REG : // OP_CALL → STATE_SET_REG
                        (opcode == OP_RET || opcode == OP_POP)            ? STATE_FETCH_SP : // RET or POP → STATE_FETCH_SP
                                                                          STATE_NEXT;

      4'b0100: state <= (opcode == OP_MOV)                                ? STATE_MOV_STORE : // OP_MOV → STATE_MOV_STORE
                        (opcode == OP_CALL)                               ? STATE_FETCH_SP : // OP_CALL → STATE_FETCH_SP
                        (opcode == OP_RET)                                ? STATE_RET : // OP_RET → STATE_RET
                        (opcode == OP_OUT)                                ? STATE_OUT : // OP_OUT → STATE_OUT
                        (opcode == OP_POP)                                ? STATE_SET_REG : // OP_POP → STATE_SET_REG
                        (opcode == OP_IN)                                 ? STATE_IN : // OP_IN → STATE_IN
                                                                          STATE_NEXT;

      4'b0101: state <= (opcode == OP_CALL) ? STATE_PC_STORE : STATE_NEXT; // OP_CALL → STATE_PC_STORE

      4'b0110: state <= (opcode == OP_CALL) ? STATE_TMP_JUMP : STATE_NEXT; // OP_CALL → STATE_TMP_JUMP

      4'b0111: state <= STATE_NEXT; 

      default: $display("Cannot decode: cycle = %0d, instruction = %h", cycle, instruction);
    endcase

    // Cycle increment with wrap-around
    cycle <= (cycle > 6) ? 0 : cycle + 1;
  end

endmodule
