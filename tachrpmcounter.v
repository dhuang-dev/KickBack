// ============================================================================
// tach_rpm_counter.v
// Converts hall-sensor / tachometer pulses from the flywheel motor into an
// RPM measurement, updated once per fixed sample window.
//
// RPM = (pulse_count / PULSES_PER_REV) * (60 / SAMPLE_WINDOW_SECONDS)
// This module outputs pulse_count directly each window; the microcontroller
// (or the speed controller below) applies the scaling, since it is a fixed
// multiply that is easier to keep as a constant on the MCU side, or can be
// pre-scaled here via the RPM_SCALE parameter if you'd rather do it in RTL.
// ============================================================================
module tach_rpm_counter #(
    parameter integer CLK_FREQ_HZ       = 50_000_000,  // system clock
    parameter integer SAMPLE_WINDOW_MS  = 100,          // 100 ms sample window
    parameter integer PULSES_PER_REV    = 2             // hall pulses per shaft revolution
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tach_pulse,      // raw tachometer/hall sensor input
    output reg  [15:0] rpm,             // measured RPM, updated every sample window
    output reg          rpm_valid       // 1-cycle pulse when rpm updates
);

    localparam integer WINDOW_CYCLES =
        (CLK_FREQ_HZ / 1000) * SAMPLE_WINDOW_MS;

    // ---- synchronize the async tach input ----
    reg tach_sync0, tach_sync1, tach_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tach_sync0 <= 1'b0;
            tach_sync1 <= 1'b0;
            tach_prev  <= 1'b0;
        end else begin
            tach_sync0 <= tach_pulse;
            tach_sync1 <= tach_sync0;
            tach_prev  <= tach_sync1;
        end
    end

    wire tach_rising = tach_sync1 & ~tach_prev;

    // ---- window timer + pulse accumulator ----
    reg [31:0] window_counter;
    reg [15:0] pulse_count;

    // RPM = pulses_in_window * (60000 / SAMPLE_WINDOW_MS) / PULSES_PER_REV
    localparam integer RPM_MULT = 60000 / SAMPLE_WINDOW_MS;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_counter <= 32'd0;
            pulse_count    <= 16'd0;
            rpm            <= 16'd0;
            rpm_valid      <= 1'b0;
        end else begin
            rpm_valid <= 1'b0;

            if (tach_rising)
                pulse_count <= pulse_count + 1'b1;

            if (window_counter >= WINDOW_CYCLES - 1) begin
                window_counter <= 32'd0;
                rpm            <= (pulse_count * RPM_MULT) / PULSES_PER_REV;
                pulse_count    <= 16'd0;
                rpm_valid      <= 1'b1;
            end else begin
                window_counter <= window_counter + 1'b1;
            end
        end
    end

endmodule
