module fourBitRCA (
    input        clk,    // clock
    input        rst_n,  // active-low reset
    input  [3:0] A,
    input  [3:0] B,
    input        Cin,
    output reg [3:0] Sum,
    output reg       Cout
);

    reg [3:0] A_reg, B_reg;
    reg       Cin_reg;
    reg [4:0] result;

    // Capture inputs on clock
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            A_reg   <= 4'b0000;
            B_reg   <= 4'b0000;
            Cin_reg <= 1'b0;
        end else begin
            A_reg   <= A;
            B_reg   <= B;
            Cin_reg <= Cin;
        end
    end

    // Compute sum and carry synchronously
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            Sum  <= 4'b0000;
            Cout <= 1'b0;
        end else begin
            result <= A_reg + B_reg + Cin_reg;
            Sum    <= result[3:0];
            Cout   <= result[4];
        end
    end

endmodule

