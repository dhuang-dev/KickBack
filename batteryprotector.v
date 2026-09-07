// ============================================================================
// battery_protect.v
// Monitors digitized (ADC) battery voltage and motor current for the 24V
// pack, and asserts a latched fault when either goes out of safe range.
// Assumes an external ADC feeds 12-bit readings scaled so that:
//   voltage_adc: 0-4095 maps to 0-30.0V   (24V nominal pack, headroom to 30V)
//   current_adc: 0-4095 maps to 0-20.0A   (headroom above motor stall current)
// Thresholds are expressed in the same ADC codes for direct comparison.
// ============================================================================
module battery_protect #(
    // 24V pack: cut off below ~19.2V (80% -- typical Li-ion/LiFePO4 low-voltage
    // cutoff) and above ~28.8V (120%, indicates charger/wiring fault)
    parameter integer UNDERVOLTAGE_CODE = 12'd2621,  // ~19.2V
    parameter integer OVERVOLTAGE_CODE  = 12'd3932,  // ~28.8V
    parameter integer OVERCURRENT_CODE  = 12'd3072   // ~15.0A
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [11:0] voltage_adc,
    input  wire [11:0] current_adc,
    input  wire        fault_clear,     // MCU-issued clear, after resolving the condition
    output reg          undervoltage,
    output reg          overvoltage,
    output reg          overcurrent,
    output wire         fault           // OR of all fault conditions, latched
);

    reg fault_latched;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            undervoltage  <= 1'b0;
            overvoltage   <= 1'b0;
            overcurrent   <= 1'b0;
            fault_latched <= 1'b0;
        end else begin
            undervoltage <= (voltage_adc < UNDERVOLTAGE_CODE);
            overvoltage  <= (voltage_adc > OVERVOLTAGE_CODE);
            overcurrent  <= (current_adc > OVERCURRENT_CODE);

            if ((voltage_adc < UNDERVOLTAGE_CODE) ||
                (voltage_adc > OVERVOLTAGE_CODE)  ||
                (current_adc > OVERCURRENT_CODE)) begin
                fault_latched <= 1'b1;             // latch until explicitly cleared
            end else if (fault_clear) begin
                fault_latched <= 1'b0;
            end
        end
    end

    assign fault = fault_latched;

endmodule
