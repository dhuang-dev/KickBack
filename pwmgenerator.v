// ============================================================================
// pwm_generator.v
// Generates a variable duty-cycle PWM signal to drive the motor driver
// (gate driver for the flywheel launcher motor's power MOSFET/H-bridge).
// ============================================================================
module pwm_generator #(
    parameter integer COUNTER_WIDTH = 10   // 10-bit counter -> 1024 duty steps
) (
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire [COUNTER_WIDTH-1:0]    duty_cycle,   // 0 = off, (2^WIDTH - 1) = 100%
    input  wire                        enable,       // gated off on fault
    output reg                         pwm_out
);

    reg [COUNTER_WIDTH-1:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= {COUNTER_WIDTH{1'b0}};
            pwm_out <= 1'b0;
        end else begin
            counter <= counter + 1'b1;

            if (!enable) begin
                pwm_out <= 1'b0;
            end else begin
                pwm_out <= (counter < duty_cycle);
            end
        end
    end

endmodule
