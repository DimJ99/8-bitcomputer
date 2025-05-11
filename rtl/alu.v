module alu (
  input wire enable,
  input wire clk,
  input wire reset,
  input wire [2:0] op,
  input wire [7:0] in_a,
  input wire [7:0] in_b,
  output wire [7:0] out,
  output reg flag_zero,
  output reg flag_carry
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

  reg [7:0] buff_out;

  always @(posedge clk or posedge reset) begin
    if (reset) begin
      buff_out   <= 0;
      flag_zero  <= 0;
      flag_carry <= 0;
    end else if (enable) begin
      case (op)
        ALU_ADD: {flag_carry, buff_out} = in_a + in_b;
        ALU_ADC: {flag_carry, buff_out} = in_a + in_b + flag_carry;
        ALU_SUB: {flag_carry, buff_out} = in_a - in_b;
        ALU_INC: {flag_carry, buff_out} = in_a + 1;
        ALU_DEC: {flag_carry, buff_out} = in_a - 1;
        ALU_AND: buff_out = in_a & in_b;
        ALU_OR:  buff_out = in_a | in_b;
        ALU_XOR: buff_out = in_a ^ in_b;
        default: buff_out = 8'hxx;
      endcase

      flag_zero <= (buff_out == 0);
    end
  end

  assign out = buff_out;

endmodule
