module counter (
  input  logic        clk,        
  input  logic        reset,       
  input  logic [7:0]  load_value, 
  input  logic        load,         
  input  logic        inc,       
  input  logic        dec,          
  output logic [7:0]  out           
);

//---------------------------------------------------------------
// PC
//---------------------------------------------------------------
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      out <= 8'd0;
    end else if (load) begin
      out <= load_value;
    end else if (dec) begin
      out <= out - 8'd1;
    end else if (inc) begin
      out <= out + 8'd1;
    end 
  end

endmodule
