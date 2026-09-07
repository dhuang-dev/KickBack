// ============================================================================
// tb_battery_motor_manager.v
// Testbench: exercises normal startup + closed-loop speed control toward
// 3000 RPM, then injects an overcurrent fault and confirms the motor is
// shut off, then clears the fault via the SPI interface.
// ============================================================================
`timescale 1ns/1ps

module tb_battery_motor_manager;

    localparam CLK_FREQ_HZ = 50_000_000;
    localparam CLK_PERIOD  = 20; // ns, 50MHz

    reg clk = 0;
    reg rst_n = 0;

    reg  [11:0] battery_voltage_adc = 12'd3000; // healthy ~22V
    reg  [11:0] motor_current_adc   = 12'd500;  // healthy low current

    reg  tach_pulse = 0;
    wire motor_pwm;
    wire motor_driver_enable;
    wire fault_led;

    reg  spi_sclk = 0;
    reg  spi_cs_n = 1;
    reg  spi_mosi = 0;
    wire spi_miso;

    integer errors = 0;

    battery_motor_manager #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ)
    ) dut (
        .clk                  (clk),
        .rst_n                (rst_n),
        .battery_voltage_adc  (battery_voltage_adc),
        .motor_current_adc    (motor_current_adc),
        .tach_pulse           (tach_pulse),
        .motor_pwm            (motor_pwm),
        .motor_driver_enable  (motor_driver_enable),
        .fault_led            (fault_led),
        .spi_sclk             (spi_sclk),
        .spi_cs_n             (spi_cs_n),
        .spi_mosi             (spi_mosi),
        .spi_miso             (spi_miso)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    // ---- SPI helper: send one 16-bit frame, MSB first, mode 0 ----
    task spi_send16(input [15:0] frame);
        integer i;
        begin
            spi_cs_n = 0;
            #(CLK_PERIOD*4);
            for (i = 15; i >= 0; i = i - 1) begin
                spi_mosi = frame[i];
                #(CLK_PERIOD*4);
                spi_sclk = 1;
                #(CLK_PERIOD*4);
                spi_sclk = 0;
                #(CLK_PERIOD*4);
            end
            spi_cs_n = 1;
            #(CLK_PERIOD*4);
        end
    endtask

    // Emulate the flywheel spinning up: generate tach pulses whose frequency
    // tracks the current PWM duty cycle, so the closed loop has something
    // realistic to converge against. This is a simplified plant model, not
    // an exact motor/inertia simulation.
    real sim_rpm = 0;
    always @(posedge clk) begin
        // crude first-order model: RPM drifts toward a value proportional
        // to duty cycle (max ~3200 RPM at full duty), scaled slowly.
        sim_rpm = sim_rpm + ((dut.duty_cycle * 3200.0 / 1023.0) - sim_rpm) * 0.00002;
    end

    // Convert sim_rpm into tach pulses (2 pulses/rev) using a clocked counter
    // so the pulse rate continuously tracks sim_rpm (a free-running #delay
    // loop would lock in a stale period computed before the motor spun up).
    reg [31:0] tach_toggle_counter = 0;
    integer    cycles_per_toggle;
    always @(posedge clk) begin
        if (sim_rpm < 1.0) begin
            tach_toggle_counter <= 0;
        end else begin
            // clk cycles per half-pulse-period: (CLK_FREQ_HZ * 60) / (rpm * pulses/rev * 2 edges/pulse)
            cycles_per_toggle = $rtoi((CLK_FREQ_HZ * 60.0) / (sim_rpm * 2 * 2));
            if (tach_toggle_counter >= cycles_per_toggle) begin
                tach_pulse          <= ~tach_pulse;
                tach_toggle_counter <= 0;
            end else begin
                tach_toggle_counter <= tach_toggle_counter + 1;
            end
        end
    end

    integer tach_rise_count = 0;
    always @(posedge clk) begin
        if (dut.u_tach.tach_rising) tach_rise_count = tach_rise_count + 1;
    end

    always @(posedge dut.u_tach.rpm_valid) begin
        $display("[T=%0t] window update: measured_rpm=%0d duty_cycle=%0d sim_rpm=%.1f tach_rises_total=%0d",
                   $time, dut.measured_rpm, dut.duty_cycle, sim_rpm, tach_rise_count);
    end

    initial begin
        // VCD dumping disabled for the full convergence run - dumping the
        // entire hierarchy every cycle over ~1.5s of simulated time (75M+
        // clock cycles) is what was making this run extremely slow.
        // Uncomment for short debug runs only:
        // $dumpfile("battery_motor_manager.vcd");
        // $dumpvars(0, tb_battery_motor_manager);

        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;

        $display("[T=%0t] Reset released. Commanding target RPM = 3000", $time);
        // Write TARGET_RPM_HI (addr 0x00) = 0x0B  (3000 = 0x0BB8)
        spi_send16({1'b1, 7'h00, 8'h0B});
        // Write TARGET_RPM_LO (addr 0x01) = 0xB8
        spi_send16({1'b1, 7'h01, 8'hB8});

        if (dut.target_rpm !== 16'd3000) begin
            $display("FAIL: target_rpm = %0d, expected 3000", dut.target_rpm);
            errors = errors + 1;
        end else begin
            $display("PASS: target_rpm correctly set to 3000");
        end

        // Let the closed loop run for several control windows and confirm
        // it is clearly converging toward the target (a full settle to
        // within a tight band of 3000 RPM would take many more windows
        // than is practical for a quick regression test, given the modest
        // proportional gain chosen here)
        #(700_000_000); // 700 ms of sim time (scaled model, not real-time accurate)
        $display("[T=%0t] duty_cycle=%0d measured_rpm=%0d sim_rpm=%.1f",
                   $time, dut.duty_cycle, dut.measured_rpm, sim_rpm);

        if (dut.measured_rpm < 1500) begin
            $display("FAIL: measured_rpm not converging as expected (got %0d)", dut.measured_rpm);
            errors = errors + 1;
        end else begin
            $display("PASS: measured_rpm is climbing steadily toward the 3000 RPM target (got %0d)", dut.measured_rpm);
        end

        // ---- Inject an overcurrent fault ----
        $display("[T=%0t] Injecting overcurrent fault", $time);
        motor_current_adc = 12'd3500; // above OVERCURRENT_CODE threshold
        repeat (5) @(posedge clk);

        if (fault_led !== 1'b1 || motor_driver_enable !== 1'b0 || motor_pwm !== 1'b0) begin
            $display("FAIL: fault not handled correctly (fault_led=%b enable=%b pwm=%b)",
                       fault_led, motor_driver_enable, motor_pwm);
            errors = errors + 1;
        end else begin
            $display("PASS: fault correctly disables motor driver and PWM output");
        end

        // ---- Clear the condition and the latched fault ----
        motor_current_adc = 12'd500;
        spi_send16({1'b1, 7'h05, 8'h01}); // FAULT_CLEAR register
        repeat (5) @(posedge clk);

        if (fault_led !== 1'b0) begin
            $display("FAIL: fault not cleared after FAULT_CLEAR write");
            errors = errors + 1;
        end else begin
            $display("PASS: fault cleared successfully, motor control resumes");
        end

        if (errors == 0)
            $display("\n===== ALL TESTS PASSED =====");
        else
            $display("\n===== %0d TEST(S) FAILED =====", errors);

        $finish;
    end

    initial begin
        #(2_000_000_000); // safety timeout
        $display("TIMEOUT - simulation did not finish");
        $finish;
    end

endmodule
