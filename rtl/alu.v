module alu (
  input  logic        enable,
  input  logic        clk,
  input  logic        reset,
  input  logic [2:0]  op,
  input  logic [7:0]  in_a,
  input  logic [7:0]  in_b,
  output logic [7:0]  out,
  output logic        flag_zero,
  output logic        flag_carry
);

  // ALU operation codes
  localparam ALU_ADD = 3'b000;
  localparam ALU_SUB = 3'b001;
  localparam ALU_INC = 3'b010;
  localparam ALU_DEC = 3'b011;
  localparam ALU_AND = 3'b100;
  localparam ALU_OR  = 3'b101;
  localparam ALU_XOR = 3'b110;
  localparam ALU_ADC = 3'b111;

  logic [7:0] buff_out;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      buff_out   <= 8'd0;
      flag_zero  <= 1'b0;
      flag_carry <= 1'b0;
    end else if (enable) begin
      case (op)
        ALU_ADD: {flag_carry, buff_out} <= in_a + in_b;
        ALU_ADC: {flag_carry, buff_out} <= in_a + in_b + flag_carry;
        ALU_SUB: {flag_carry, buff_out} <= in_a - in_b;
        ALU_INC: {flag_carry, buff_out} <= in_a + 1;
        ALU_DEC: {flag_carry, buff_out} <= in_a - 1;
        ALU_AND: begin
          buff_out   <= in_a & in_b;
          flag_carry <= 1'b0;
        end
        ALU_OR: begin
          buff_out   <= in_a | in_b;
          flag_carry <= 1'b0;
        end
        ALU_XOR: begin
          buff_out   <= in_a ^ in_b;
          flag_carry <= 1'b0;
        end
        default: begin
          buff_out   <= 8'hXX;
          flag_carry <= 1'b0;
        end
      endcase
      flag_zero <= (buff_out == 8'd0);
    end
  end

  assign out = buff_out;

endmodule
