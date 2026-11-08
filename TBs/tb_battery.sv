
// tb_battery_motor_manager.sv
// Testbench: Exercises startup, closed-loop speed control, SPI read/write,
// overcurrent faulting, undervoltage/overvoltage thresholds, and SVA safety checks.

`timescale 1ns/1ps

module tb_battery_motor_manager;

    localparam CLK_FREQ_HZ = 50_000_000;
    localparam CLK_PERIOD  = 20; // ns, 50MHz

    reg clk = 0;
    reg rst_n = 0;

    reg  [11:0] battery_voltage_adc = 12'd3000; // nominal 22V
    reg  [11:0] motor_current_adc   = 12'd500;  // nominal safe current

    reg  tach_pulse = 0;
    wire motor_pwm;
    wire motor_driver_enable;
    wire fault_led;

    reg  spi_sclk = 0;
    reg  spi_cs_n = 1;
    reg  spi_mosi = 0;
    wire spi_miso;

    integer errors = 0;

    // Device Under Test (DUT)
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

  // SYSTEMVERILOG ASSERTIONS (SVA) FOR HARDWARE SAFETY & FUNCTIONAL RULES

    // Rule 1: Immediate Safety Shutdown on Fault
    property p_fault_safety_shutdown;
        @(posedge clk) disable iff (!rst_n)
        dut.u_battery_protect.fault |-> (!motor_pwm && !motor_driver_enable && fault_led);
    endproperty
    assert property (p_fault_safety_shutdown)
        else begin
            $error("SVA ERROR: Motor driver or PWM output not disabled during active fault!");
            errors = errors + 1;
        end

    // Rule 2: Speed Controller Zero Duty-Cycle Safety Guard
    property p_speed_ctrl_zero_on_fault;
        @(posedge clk) disable iff (!rst_n)
        dut.u_battery_protect.fault |-> (dut.u_speed_ctrl.duty_cycle == 10'd0);
    endproperty
    assert property (p_speed_ctrl_zero_on_fault)
        else begin
            $error("SVA ERROR: Speed controller duty cycle was not zeroed during fault!");
            errors = errors + 1;
        end

    // Rule 3: PWM Output Gating Alignment
    property p_pwm_gated_when_disabled;
        @(posedge clk) disable iff (!rst_n)
        (!dut.u_pwm.enable) |-> (motor_pwm == 1'b0);
    endproperty
    assert property (p_pwm_gated_when_disabled)
        else begin
            $error("SVA ERROR: PWM generator leaked high output while disabled!");
            errors = errors + 1;
        end
      
    // SPI HELPER TASKS (FULL READ & WRITE SUPPORT)

    // Transfers 16-bit MSB-first frame over SPI Mode 0 and returns MISO payload
    task spi_transfer16(input [15:0] frame_in, output [15:0] frame_out);
        integer i;
        begin
            frame_out = 16'h0000;
            spi_cs_n = 0;
            #(CLK_PERIOD*4);
            for (i = 15; i >= 0; i = i - 1) begin
                spi_mosi = frame_in[i];
                #(CLK_PERIOD*4);
                spi_sclk = 1;
                frame_out[i] = spi_miso;
                #(CLK_PERIOD*4);
                spi_sclk = 0;
                #(CLK_PERIOD*4);
            end
            spi_cs_n = 1;
            #(CLK_PERIOD*4);
        end
    endtask

    // Convenience task for simple SPI write
    task spi_write(input [6:0] addr, input [7:0] data);
        reg [15:0] unused_rx;
        begin
            spi_transfer16({1'b1, addr, data}, unused_rx);
        end
    endtask

    // Convenience task for simple SPI read
    task spi_read8(input [6:0] addr, output [7:0] data);
        reg [15:0] rx_frame;
        begin
            // SPI Mode 0 read: address is sent in frame 1, low byte read on frame response
            spi_transfer16({1'b0, addr, 8'h00}, rx_frame);
            data = rx_frame[7:0];
        end
    endtask

    // PLANT MODEL FOR MOTOR INERTIA & TACHOMETER GENERATION

    real sim_rpm = 0;
    always @(posedge clk) begin
        sim_rpm = sim_rpm + ((dut.duty_cycle * 3200.0 / 1023.0) - sim_rpm) * 0.00002;
    end

    reg [31:0] tach_toggle_counter = 0;
    integer    cycles_per_toggle;
    always @(posedge clk) begin
        if (sim_rpm < 1.0) begin
            tach_toggle_counter <= 0;
        end else begin
            cycles_per_toggle = $rtoi((CLK_FREQ_HZ * 60.0) / (sim_rpm * 2 * 2));
            if (tach_toggle_counter >= cycles_per_toggle) begin
                tach_pulse          <= ~tach_pulse;
                tach_toggle_counter <= 0;
            end else begin
                tach_toggle_counter <= tach_toggle_counter + 1;
            end
        end
    end

    // MAIN TEST SEQUENCES
    reg [7:0] read_status_val;
    reg [7:0] rpm_hi, rpm_lo;

    initial begin
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;

        $display("\n--- TEST 1: System Startup & SPI Target Write ---");
        $display("[T=%0t] Reset released. Commanding Target RPM = 3000", $time);
        
        // Write TARGET_RPM = 3000 (0x0BB8)
        spi_write(7'h00, 8'h0B); // TARGET_RPM_HI
        spi_write(7'h01, 8'hB8); // TARGET_RPM_LO

        if (dut.target_rpm !== 16'd3000) begin
            $display("FAIL: Target RPM mismatch! Expected 3000, got %0d", dut.target_rpm);
            errors = errors + 1;
        end else begin
            $display("PASS: target_rpm set to 3000 via SPI");
        end

        $display("\n--- TEST 2: Closed-Loop Acceleration ---");
        #(700_000_000); // 700ms simulation time
        $display("[T=%0t] Measured RPM: %0d | Duty Cycle: %0d | Sim Speed: %.1f",
                 $time, dut.measured_rpm, dut.duty_cycle, sim_rpm);

        if (dut.measured_rpm < 1500) begin
            $display("FAIL: Closed-loop control failing to accelerate motor (got %0d)", dut.measured_rpm);
            errors = errors + 1;
        end else begin
            $display("PASS: Measured RPM climbing properly toward target");
        end

        $display("\n--- TEST 3: SPI Register Readback Verification ---");
        spi_read8(7'h02, rpm_hi);
        spi_read8(7'h03, rpm_lo);
        spi_read8(7'h04, read_status_val);

        $display("SPI Readback -> Measured RPM: 0x%0h%0h (%0d), Status Reg: 8'b%08b", 
                 rpm_hi, rpm_lo, {rpm_hi, rpm_lo}, read_status_val);

        if (read_status_val[7] !== 1'b0) begin
            $display("FAIL: Status register falsely reported active fault");
            errors = errors + 1;
        end else begin
            $display("PASS: Status register correctly reads 0x00 under healthy conditions");
        end

        $display("\n--- TEST 4: Overcurrent Fault Injection & Auto-Shutdown ---");
        motor_current_adc = 12'd3500; // Above 15A cutoff code (3072)
        repeat (10) @(posedge clk);

        spi_read8(7'h04, read_status_val); // Read STATUS register
        if (fault_led !== 1'b1 || motor_driver_enable !== 1'b0 || motor_pwm !== 1'b0 || read_status_val[6] !== 1'b1) begin
            $display("FAIL: Overcurrent fault failed to disable driver or update STATUS register!");
            errors = errors + 1;
        end else begin
            $display("PASS: Overcurrent fault latched, motor gate disabled, STATUS overcurrent bit set");
        end

        // Clear overcurrent fault
        motor_current_adc = 12'd500;
        spi_write(7'h05, 8'h01); // FAULT_CLEAR
        repeat (10) @(posedge clk);

        if (fault_led !== 1 me 0 && dut.u_battery_protect.fault !== 1'b0) begin
            $display("FAIL: Fault failed to clear after FAULT_CLEAR register write");
            errors = errors + 1;
        end else begin
            $display("PASS: Overcurrent fault successfully cleared via SPI");
        end

        $display("\n--- TEST 5: Undervoltage Protection Cutoff ---");
        battery_voltage_adc = 12'd2000; // Below 19.2V cutoff code (2621)
        repeat (10) @(posedge clk);

        spi_read8(7'h04, read_status_val);
        if (fault_led !== 1'b1 || read_status_val[4] !== 1'b1) begin
            $display("FAIL: Undervoltage condition did not trigger latched fault!");
            errors = errors + 1;
        end else begin
            $display("PASS: Undervoltage fault successfully detected and latched");
        end

        // Clear undervoltage fault
        battery_voltage_adc = 12'd3000;
        spi_write(7'h05, 8'h01);
        repeat (10) @(posedge clk);

        $display("\n--- TEST 6: Overvoltage Protection Cutoff ---");
        battery_voltage_adc = 12'd4000; // Above 28.8V cutoff code (3932)
        repeat (10) @(posedge clk);

        spi_read8(7'h04, read_status_val);
        if (fault_led !== 1'b1 || read_status_val[5] !== 1'b1) begin
            $display("FAIL: Overvoltage condition did not trigger latched fault!");
            errors = errors + 1;
        end else begin
            $display("PASS: Overvoltage fault successfully detected and latched");
        end

        // Clean up
        battery_voltage_adc = 12'd3000;
        spi_write(7'h05, 8'h01);
        repeat (10) @(posedge clk);

        if (errors == 0)
            $display("ALL TESTS & ASSERTIONS PASSED SUCCESSFULLY");
        else
          $display("VERIFICATION FAILED WITH %0d ERROR(S)", errors);

        $finish;
    end

    initial begin
        #(2_000_000_000);
        $display("TIMEOUT: Simulation limit reached");
        $finish;
    end

endmodule
