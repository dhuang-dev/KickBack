// ============================================================================
// speed_controller.v
// Simple proportional (P) controller that adjusts PWM duty cycle to drive
// measured RPM toward the target RPM (e.g. 3000 RPM) commanded by the
// microcontroller.
// ============================================================================
module speed_controller #(
    parameter integer DUTY_WIDTH = 10,
    parameter integer KP_SHIFT   = 4      // proportional gain = 1/(2^KP_SHIFT)
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire [15:0]              target_rpm,   // commanded speed, e.g. 3000
    input  wire [15:0]              measured_rpm, // from tach_rpm_counter
    input  wire                     rpm_valid,    // pulse: new measurement ready
    input  wire                     fault,        // battery/overcurrent fault -> force duty 0
    output reg  [DUTY_WIDTH-1:0]    duty_cycle
);

    localparam integer MAX_DUTY = (1 << DUTY_WIDTH) - 1;

    reg signed [16:0] error;
    reg signed [16:0] correction;
    reg signed [17:0] duty_next;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_cycle <= {DUTY_WIDTH{1'b0}};
        end else if (fault) begin
            duty_cycle <= {DUTY_WIDTH{1'b0}};   // fail-safe: motor off on fault
        end else if (rpm_valid) begin
            error      = $signed({1'b0, target_rpm}) - $signed({1'b0, measured_rpm});
            correction = error >>> KP_SHIFT;
            duty_next  = $signed({2'b00, duty_cycle}) + correction;

            if (duty_next < 0)
                duty_cycle <= {DUTY_WIDTH{1'b0}};
            else if (duty_next > MAX_DUTY)
                duty_cycle <= MAX_DUTY[DUTY_WIDTH-1:0];
            else
                duty_cycle <= duty_next[DUTY_WIDTH-1:0];
        end
    end

endmodule
