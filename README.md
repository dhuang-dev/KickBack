# KICKBACK — 24V Battery & Motor Management (Verilog)

Digital control logic for the KICKBACK robotic soccer ball launcher/receiver (2nd Place Overall, Engineering Academy Entrepreneurship Competition). Sits between the onboard microcontroller and the power stage on the custom PCB: monitors the 24V battery pack, runs closed-loop speed control on the flywheel motor toward a commanded target RPM (e.g. 3000 RPM), and exposes status/control to the MCU over SPI.

```
ADC (voltage, current) --> battery_protect ---> fault -----------+
                                                                  v
tach pulses --> tach_rpm_counter --> speed_controller --> pwm_generator --> motor driver gate
                       ^                    ^
                       |                    |
                 measured_rpm         target_rpm
                       |                    |
                       +---- spi_slave_regs <--- MCU (SPI master)
```

## Modules

| File | What it does |
|---|---|
| `battery_protect.v` | Monitors battery voltage and motor current (via ADC). Raises a **latched fault** on undervoltage, overvoltage, or overcurrent — stays latched until the MCU explicitly clears it. |
| `tach_rpm_counter.v` | Converts hall-sensor/tachometer pulses from the motor into an RPM measurement, updated every 100ms. |
| `speed_controller.v` | Proportional (P) controller — adjusts PWM duty cycle each update to drive measured RPM toward the MCU-commanded target RPM. Forces duty cycle to zero immediately on fault. |
| `pwm_generator.v` | Generates the variable duty-cycle PWM signal that drives the motor driver / gate driver. |
| `spi_slave_regs.v` | SPI slave register interface. Lets the MCU write a target RPM and read back measured RPM and fault status. |
| `battery_motor_manager.v` | **Top module** — wires all of the above together. |
| `tb_battery_motor_manager.v` | Testbench — simulates an MCU issuing SPI commands and a spinning motor generating tach pulses, to verify the whole system before it touches real hardware. |

## SPI Register Map

| Address | Register | Access |
|---|---|---|
| `0x00` | `TARGET_RPM_HI` | Write |
| `0x01` | `TARGET_RPM_LO` | Write |
| `0x02` | `MEASURED_RPM_HI` | Read |
| `0x03` | `MEASURED_RPM_LO` | Read |
| `0x04` | `STATUS` (`{fault, overcurrent, overvoltage, undervoltage}`) | Read |
| `0x05` | `FAULT_CLEAR` (write `0x01` to clear) | Write |

## Running the simulation

Requires [Icarus Verilog](http://iverilog.icarus.com/):

```bash
iverilog -o sim.out battery_motor_manager.v battery_protect.v tach_rpm_counter.v \
    speed_controller.v pwm_generator.v spi_slave_regs.v tb_battery_motor_manager.v
vvp sim.out
```

To inspect signal waveforms, uncomment the `$dumpfile`/`$dumpvars` lines in the testbench, re-run, and open the resulting `.vcd` file in [GTKWave](http://gtkwave.sourceforge.net/).

### Sample output

```
[T=90000] Reset released. Commanding target RPM = 3000
PASS: target_rpm correctly set to 3000
[T=100000070000] window update: measured_rpm=0    duty_cycle=0   sim_rpm=0.0
[T=200000070000] window update: measured_rpm=600  duty_cycle=187 sim_rpm=584.9
[T=300000070000] window update: measured_rpm=1200 duty_cycle=337 sim_rpm=1054.2
[T=400000070000] window update: measured_rpm=1200 duty_cycle=449 sim_rpm=1404.5
[T=500000070000] window update: measured_rpm=1800 duty_cycle=561 sim_rpm=1754.8
[T=600000070000] window update: measured_rpm=2100 duty_cycle=636 sim_rpm=1989.4
[T=700000070000] window update: measured_rpm=2100 duty_cycle=692 sim_rpm=2164.6
PASS: measured_rpm is climbing steadily toward the 3000 RPM target (got 2100)
Injecting overcurrent fault
PASS: fault correctly disables motor driver and PWM output
PASS: fault cleared successfully, motor control resumes

===== ALL TESTS PASSED =====
```

## Core concepts

- **Combinational vs. sequential logic** — combinational output depends only on current inputs (`assign fault = ...`); sequential output depends on history and needs a clock (`always @(posedge clk)` blocks).
- **Blocking (`=`) vs. non-blocking (`<=`) assignment** — non-blocking is used inside clocked blocks so multiple registers update simultaneously at the next clock edge; blocking executes immediately and is used in combinational logic.
- **Synchronizers** — external signals (SPI clock, tach pulses) are passed through 2-stage flip-flop synchronizers before use, to avoid metastability from signals that aren't aligned to the internal clock.
- **Closed-loop control** — `error = target - measured; correction = error * gain;` is a basic proportional controller, the same principle behind cruise control and thermostats.
- **SPI** — a simple protocol for shifting bits between chips over a clock line, a data line, and a chip-select line.

## Tools

- **Icarus Verilog** — simulation
- **GTKWave** — waveform viewing
- **EDA Playground** — browser-based alternative, no install needed
- **Vivado** / **Quartus** — synthesis onto real FPGA hardware (Xilinx/AMD and Intel/Altera respectively)
