// ============================================================================
// battery_motor_manager.v  (TOP MODULE)
//
// KICKBACK - 24V battery & flywheel motor management logic.
// Sits between the onboard microcontroller and the power stage on the
// custom PCB: monitors the 24V pack (voltage + current), runs closed-loop
// speed control on the flywheel motor toward an MCU-commanded target RPM
// (e.g. 3000 RPM), and exposes status/control to the MCU over SPI.
//
//   ADC (voltage, current) --> battery_protect ---> fault -----------+
//                                                                    v
//   tach pulses --> tach_rpm_counter --> speed_controller --> pwm_generator --> motor driver gate
//                          ^                    ^
//                          |                    |
//                    measured_rpm         target_rpm
//                          |                    |
//                          +---- spi_slave_regs <--- MCU (SPI master)
// ============================================================================
module battery_motor_manager #(
    parameter integer CLK_FREQ_HZ = 50_000_000
) (
    input  wire        clk,
    input  wire        rst_n,

    // Battery / current sense (from external ADCs on the PCB)
    input  wire [11:0] battery_voltage_adc,
    input  wire [11:0] motor_current_adc,

    // Motor feedback
    input  wire        tach_pulse,

    // Motor driver output
    output wire         motor_pwm,
    output wire         motor_driver_enable,   // gate driver / H-bridge enable

    // Fault indicator (e.g. to an LED on the PCB)
    output wire         fault_led,

    // SPI to onboard microcontroller
    input  wire         spi_sclk,
    input  wire         spi_cs_n,
    input  wire         spi_mosi,
    output wire          spi_miso
);

    // ---- battery protection ----
    wire undervoltage, overvoltage, overcurrent, fault;
    wire fault_clear_pulse;

    battery_protect #(
        .UNDERVOLTAGE_CODE (12'd2621),
        .OVERVOLTAGE_CODE  (12'd3932),
        .OVERCURRENT_CODE  (12'd3072)
    ) u_battery_protect (
        .clk          (clk),
        .rst_n        (rst_n),
        .voltage_adc  (battery_voltage_adc),
        .current_adc  (motor_current_adc),
        .fault_clear  (fault_clear_pulse),
        .undervoltage (undervoltage),
        .overvoltage  (overvoltage),
        .overcurrent  (overcurrent),
        .fault        (fault)
    );

    // ---- tachometer / RPM measurement ----
    wire [15:0] measured_rpm;
    wire        rpm_valid;

    tach_rpm_counter #(
        .CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .SAMPLE_WINDOW_MS (100),
        .PULSES_PER_REV   (2)
    ) u_tach (
        .clk        (clk),
        .rst_n      (rst_n),
        .tach_pulse (tach_pulse),
        .rpm        (measured_rpm),
        .rpm_valid  (rpm_valid)
    );

    // ---- SPI register interface to MCU ----
    wire [15:0] target_rpm;

    spi_slave_regs u_spi (
        .clk               (clk),
        .rst_n             (rst_n),
        .sclk              (spi_sclk),
        .cs_n              (spi_cs_n),
        .mosi              (spi_mosi),
        .miso              (spi_miso),
        .target_rpm        (target_rpm),
        .measured_rpm      (measured_rpm),
        .undervoltage      (undervoltage),
        .overvoltage       (overvoltage),
        .overcurrent       (overcurrent),
        .fault             (fault),
        .fault_clear_pulse (fault_clear_pulse)
    );

    // ---- closed-loop speed control ----
    wire [9:0] duty_cycle;

    speed_controller #(
        .DUTY_WIDTH (10),
        .KP_SHIFT   (4)
    ) u_speed_ctrl (
        .clk          (clk),
        .rst_n        (rst_n),
        .target_rpm   (target_rpm),
        .measured_rpm (measured_rpm),
        .rpm_valid    (rpm_valid),
        .fault        (fault),
        .duty_cycle   (duty_cycle)
    );

    // ---- PWM output to motor driver ----
    pwm_generator #(
        .COUNTER_WIDTH (10)
    ) u_pwm (
        .clk        (clk),
        .rst_n      (rst_n),
        .duty_cycle (duty_cycle),
        .enable     (~fault),
        .pwm_out    (motor_pwm)
    );

    assign motor_driver_enable = ~fault;   // gate driver disabled immediately on fault
    assign fault_led           = fault;

endmodule
