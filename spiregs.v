// ============================================================================
// spi_slave_regs.v
// Simple SPI slave exposing a small register file so the onboard
// microcontroller can command a target RPM and read back status/faults.
//
// Protocol (mode 0, MSB first), 16-bit transfers:
//   MCU sends:  [R/W][7-bit addr][8 don't-care bits]  -- write also shifts in
//               a second 16-bit word containing the write data.
//   For simplicity here: a WRITE is a single 16-bit frame:
//       bit15   = 1 (write)
//       bit14:8 = register address
//       bit7:0  = write data (8-bit, sufficient for these registers)
//   A READ is a single 16-bit frame:
//       bit15   = 0 (read)
//       bit14:8 = register address
//       bit7:0  = don't care (MOSI); MISO returns the register value on the
//                 following 8 clocks
//
// Registers:
//   0x00  TARGET_RPM_HI   (target_rpm[15:8])
//   0x01  TARGET_RPM_LO   (target_rpm[7:0])
//   0x02  MEASURED_RPM_HI (read-only)
//   0x03  MEASURED_RPM_LO (read-only)
//   0x04  STATUS          (read-only: {fault, overcurrent, overvoltage, undervoltage, 4'b0})
//   0x05  FAULT_CLEAR     (write 8'h01 to clear latched fault)
// ============================================================================
module spi_slave_regs (
    input  wire        clk,
    input  wire        rst_n,

    // SPI physical interface
    input  wire        sclk,
    input  wire        cs_n,
    input  wire        mosi,
    output reg         miso,

    // Register file interface to the rest of the design
    output reg  [15:0] target_rpm,
    input  wire [15:0] measured_rpm,
    input  wire        undervoltage,
    input  wire        overvoltage,
    input  wire        overcurrent,
    input  wire        fault,
    output reg         fault_clear_pulse
);

    // ---- synchronize SPI pins into system clock domain ----
    reg [2:0] sclk_sync, cs_sync;
    reg       mosi_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_sync <= 3'b000;
            cs_sync   <= 3'b111;
            mosi_sync <= 1'b0;
        end else begin
            sclk_sync <= {sclk_sync[1:0], sclk};
            cs_sync   <= {cs_sync[1:0], cs_n};
            mosi_sync <= mosi;
        end
    end

    wire sclk_rising = (sclk_sync[2:1] == 2'b01);
    wire cs_active   = ~cs_sync[1];

    reg [15:0] shift_in;
    reg [15:0] shift_in_next;
    reg [4:0]  bit_count;
    reg [7:0]  read_data;

    always @(*) begin
        shift_in_next = {shift_in[14:0], mosi_sync};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_in          <= 16'd0;
            bit_count         <= 5'd0;
            target_rpm        <= 16'd0;
            fault_clear_pulse <= 1'b0;
            miso              <= 1'b0;
            read_data         <= 8'd0;
        end else begin
            fault_clear_pulse <= 1'b0;

            if (!cs_active) begin
                bit_count <= 5'd0;
            end else if (sclk_rising) begin
                shift_in  <= shift_in_next;
                bit_count <= bit_count + 1'b1;

                // After the address+R/W byte is fully shifted in (8 bits),
                // latch up the read data so MISO can start shifting it out
                // on the following 8 clocks. Use shift_in_next here since
                // shift_in itself won't reflect this edge's bit until after
                // this always block completes.
                if (bit_count == 5'd7) begin
                    case (shift_in_next[6:0])
                        7'h02: read_data <= measured_rpm[15:8];
                        7'h03: read_data <= measured_rpm[7:0];
                        7'h04: read_data <= {fault, overcurrent, overvoltage, undervoltage, 4'b0000};
                        default: read_data <= 8'h00;
                    endcase
                end

                // Full 16-bit frame received (use shift_in_next: the value
                // shift_in is about to take on this same edge)
                if (bit_count == 5'd15) begin
                    if (shift_in_next[15]) begin  // write
                        case (shift_in_next[14:8])
                            7'h00: target_rpm[15:8] <= shift_in_next[7:0];
                            7'h01: target_rpm[7:0]  <= shift_in_next[7:0];
                            7'h05: if (shift_in_next[7:0] == 8'h01)
                                       fault_clear_pulse <= 1'b1;
                            default: ; // no-op
                        endcase
                    end
                    bit_count <= 5'd0;
                end
            end

            // Shift MISO out once the address/R-W byte has been received
            if (cs_active && bit_count >= 5'd8)
                miso <= read_data[15 - bit_count];
        end
    end

endmodule
