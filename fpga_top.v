// =============================================================================
//  fpga_top.v  -  FPGA Auto-Run 240-Sample Demo  (v5 - SW0 control)
//  PMDC Motor Fault Detection - SVM Inference Core (OvR, LinearSVC)
//
//  Board : Digilent Nexys 4 (XC7A100T-1CSG324C)
//  Clock : 100 MHz onboard crystal
//
//  Operation:
//    sw0 = 1 (UP)   → Reset (FPGA held in reset)
//    sw0 = 0 (DOWN) → Auto-run all 240 samples, print accuracy summary to UART
//
//  UART Output (115200 8N1):
//    SVM FPGA Results
//    Correct : XXX / 240
//    Accuracy: XXX.XX%
//
//  ROM Requirements:
//    - File: samples.mem (in Vivado project directory)
//    - Format: 240 lines of 128-char hex (512-bit wide entries)
//    - See: gen_samples_mem.py to convert tb_vectors.txt → samples.mem
//
//  LED Display:
//    led[7:0]   = ar_index (sample 0-239)
//    led[15:8]  = correct_cnt (how many correct)
//    dp         = 0 when done, 1 during run
//    seg        = units digit of sample index (0-9) or "d" when done
// =============================================================================

`timescale 1ns/1ps

module fpga_top (
    input  wire        clk,
    input  wire        btnC,
    input  wire        btnR,
    input  wire        btnL,
    input  wire        sw0,       // Active-high reset / run control
    input  wire        sw1,

    output wire [15:0] led,
    output wire [6:0]  seg,
    output wire [3:0]  an,
    output wire        dp,
    output wire        uart_tx
);

    // =========================================================================
    // Reset (active-high from sw0)
    // =========================================================================
    wire rst_n = ~sw0;

    // =========================================================================
    // Sample ROM: 240 × 512-bit
    //
    // Bit layout per entry:
    //   [31:0]    = feat_0   (signed 32-bit, two's complement)
    //   [63:32]   = feat_1
    //   ...
    //   [479:448] = feat_14
    //   [481:480] = label    (0=Healthy, 1=F1, 2=F2, 3=F3)
    //   [511:482] = 0 (padding)
    // =========================================================================
    reg [511:0] sample_mem [0:239];
    initial $readmemh("samples.mem", sample_mem);

    // =========================================================================
    // Auto-Run FSM Registers
    // =========================================================================
    localparam AR_FETCH = 3'd0;  // Wait for ROM registered output
    localparam AR_LOAD  = 3'd1;  // Assert feat_valid with ROM data
    localparam AR_WAIT  = 3'd2;  // Wait for result_valid from SVM core
    localparam AR_NEXT  = 3'd3;  // Update counters, advance sample index
    localparam AR_DONE  = 3'd4;  // All 240 done, build UART summary message
    localparam AR_UART  = 3'd5;  // Wait for UART TX to complete

    reg [2:0]  ar_state;
    reg [7:0]  ar_index;       // 0..239
    reg [7:0]  correct_cnt;    // count of correct predictions
    reg [1:0]  expected_class; // label of current sample
    reg        feat_valid;

    // Feature registers to SVM core
    reg signed [31:0] feat_data_0,  feat_data_1,  feat_data_2;
    reg signed [31:0] feat_data_3,  feat_data_4,  feat_data_5;
    reg signed [31:0] feat_data_6,  feat_data_7,  feat_data_8;
    reg signed [31:0] feat_data_9,  feat_data_10, feat_data_11;
    reg signed [31:0] feat_data_12, feat_data_13, feat_data_14;

    // Registered ROM output (1-cycle latency from address)
    reg [511:0] cur_sample;
    always @(posedge clk)
        cur_sample <= sample_mem[ar_index];

    // Feature extraction from ROM
    wire signed [31:0] r_f0  = cur_sample[31:0];
    wire signed [31:0] r_f1  = cur_sample[63:32];
    wire signed [31:0] r_f2  = cur_sample[95:64];
    wire signed [31:0] r_f3  = cur_sample[127:96];
    wire signed [31:0] r_f4  = cur_sample[159:128];
    wire signed [31:0] r_f5  = cur_sample[191:160];
    wire signed [31:0] r_f6  = cur_sample[223:192];
    wire signed [31:0] r_f7  = cur_sample[255:224];
    wire signed [31:0] r_f8  = cur_sample[287:256];
    wire signed [31:0] r_f9  = cur_sample[319:288];
    wire signed [31:0] r_f10 = cur_sample[351:320];
    wire signed [31:0] r_f11 = cur_sample[383:352];
    wire signed [31:0] r_f12 = cur_sample[415:384];
    wire signed [31:0] r_f13 = cur_sample[447:416];
    wire signed [31:0] r_f14 = cur_sample[479:448];
    wire        [1:0]  r_lbl = cur_sample[481:480];

    // =========================================================================
    // Auto-Run FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_state       <= AR_FETCH;
            ar_index       <= 8'd0;
            correct_cnt    <= 8'd0;
            expected_class <= 2'd0;
            feat_valid     <= 1'b0;
        end else begin
            feat_valid <= 1'b0;  // default: deasserted

            case (ar_state)

                AR_FETCH: begin
                    // ROM output (registered) will be valid on next cycle.
                    // ar_index[7:0] is already set.
                    ar_state <= AR_LOAD;
                end

                AR_LOAD: begin
                    // ROM output is now valid. Load feature registers and
                    // assert feat_valid for one cycle.
                    feat_data_0  <= r_f0;
                    feat_data_1  <= r_f1;
                    feat_data_2  <= r_f2;
                    feat_data_3  <= r_f3;
                    feat_data_4  <= r_f4;
                    feat_data_5  <= r_f5;
                    feat_data_6  <= r_f6;
                    feat_data_7  <= r_f7;
                    feat_data_8  <= r_f8;
                    feat_data_9  <= r_f9;
                    feat_data_10 <= r_f10;
                    feat_data_11 <= r_f11;
                    feat_data_12 <= r_f12;
                    feat_data_13 <= r_f13;
                    feat_data_14 <= r_f14;
                    expected_class <= r_lbl;
                    feat_valid   <= 1'b1;
                    ar_state     <= AR_WAIT;
                end

                AR_WAIT: begin
                    // Stall until SVM result comes back
                    if (result_valid)
                        ar_state <= AR_NEXT;
                end

                AR_NEXT: begin
                    if (result_class == expected_class)
                        correct_cnt <= correct_cnt + 8'd1;

                    if (ar_index == 8'd239) begin
                        ar_state <= AR_DONE;
                    end else begin
                        ar_index <= ar_index + 8'd1;
                        ar_state <= AR_FETCH;
                    end
                end

                AR_DONE: begin
                    // One-cycle pulse: build UART buffer
                    // (triggered by this state, see UART section)
                    ar_state <= AR_UART;
                end

                AR_UART: begin
                    // Stay here while UART sends.
                    // Will stay in this state indefinitely after UART finishes.
                end

                default: ar_state <= AR_FETCH;
            endcase
        end
    end

    wire ar_done    = (ar_state == AR_UART);
    wire ar_running = rst_n && (ar_state != AR_UART);

    // =========================================================================
    // SVM Inference Core Instantiation
    // =========================================================================
    wire        result_valid;
    wire [1:0]  result_class;
    wire [11:0] result_votes_dbg;

    svm_fault_top u_svm (
        .clk              (clk),
        .rst_n            (rst_n),
        .feat_valid       (feat_valid),
        .feat_data_0      (feat_data_0),
        .feat_data_1      (feat_data_1),
        .feat_data_2      (feat_data_2),
        .feat_data_3      (feat_data_3),
        .feat_data_4      (feat_data_4),
        .feat_data_5      (feat_data_5),
        .feat_data_6      (feat_data_6),
        .feat_data_7      (feat_data_7),
        .feat_data_8      (feat_data_8),
        .feat_data_9      (feat_data_9),
        .feat_data_10     (feat_data_10),
        .feat_data_11     (feat_data_11),
        .feat_data_12     (feat_data_12),
        .feat_data_13     (feat_data_13),
        .feat_data_14     (feat_data_14),
        .result_valid     (result_valid),
        .result_class     (result_class),
        .result_votes_dbg (result_votes_dbg)
    );

    // =========================================================================
    // Accuracy Computation (combinational)
    // =========================================================================
    // acc_val = correct_cnt * 10000 / 240  (range 0..10000)
    wire [31:0] acc_scaled = correct_cnt * 32'd10000;
    wire [31:0] acc_val    = acc_scaled / 32'd240;  // 0..10000
    wire [31:0] acc_int    = acc_val / 32'd100;      // 0..100
    wire [31:0] acc_frac   = acc_val % 32'd100;      // 0..99

    // Integer part digits (3 digits for 000..100)
    wire [3:0] d_i2 = acc_int / 32'd100;               // hundreds (0 or 1)
    wire [3:0] d_i1 = (acc_int / 32'd10) % 32'd10;     // tens
    wire [3:0] d_i0 = acc_int % 32'd10;                 // units

    // Fractional part digits
    wire [3:0] d_f1 = acc_frac / 32'd10;   // tenths
    wire [3:0] d_f0 = acc_frac % 32'd10;   // hundredths

    // correct_cnt digits (3 digits for 000..240)
    wire [3:0] cc_h = correct_cnt / 32'd100;
    wire [3:0] cc_m = (correct_cnt / 32'd10) % 32'd10;
    wire [3:0] cc_l = correct_cnt % 32'd10;

    // =========================================================================
    // UART TX  -  115200 baud, 8N1, 100 MHz clock
    //
    // Sends summary message (58 bytes total):
    //   "SVM FPGA Results\r\n"          [18 bytes]
    //   "Correct : XXX / 240\r\n"       [21 bytes]
    //   "Accuracy: XXX.XX%\r\n"         [19 bytes]
    // =========================================================================
    localparam BAUD_DIV = 868;        // 100 MHz / 115200 ≈ 868

    localparam TX_IDLE  = 2'd0;
    localparam TX_START = 2'd1;
    localparam TX_DATA  = 2'd2;
    localparam TX_STOP  = 2'd3;

    localparam MSG_LEN = 58;

    reg [7:0]  uart_buf [0:MSG_LEN-1];
    reg [5:0]  uart_byte_idx;
    reg [5:0]  uart_total_bytes;
    reg [9:0]  baud_cnt;
    reg [2:0]  bit_idx;
    reg [1:0]  tx_state;
    reg        tx_reg;

    assign uart_tx = tx_reg;

    // ASCII helper function
    function [7:0] to_digit;
        input [3:0] v;
        begin to_digit = 8'd48 + {4'd0, v}; end
    endfunction

    // Trigger: build buffer when entering AR_DONE (one-cycle pulse)
    wire uart_trigger = (ar_state == AR_DONE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_total_bytes <= 6'd0;
        end else if (uart_trigger) begin
            // ── Line 1: "SVM FPGA Results\r\n" (18 bytes, [0..17]) ──
            uart_buf[ 0] <= "S";
            uart_buf[ 1] <= "V";
            uart_buf[ 2] <= "M";
            uart_buf[ 3] <= " ";
            uart_buf[ 4] <= "F";
            uart_buf[ 5] <= "P";
            uart_buf[ 6] <= "G";
            uart_buf[ 7] <= "A";
            uart_buf[ 8] <= " ";
            uart_buf[ 9] <= "R";
            uart_buf[10] <= "e";
            uart_buf[11] <= "s";
            uart_buf[12] <= "u";
            uart_buf[13] <= "l";
            uart_buf[14] <= "t";
            uart_buf[15] <= "s";
            uart_buf[16] <= 8'h0D;  // \r
            uart_buf[17] <= 8'h0A;  // \n

            // ── Line 2: "Correct : XXX / 240\r\n" (21 bytes, [18..38]) ──
            uart_buf[18] <= "C";
            uart_buf[19] <= "o";
            uart_buf[20] <= "r";
            uart_buf[21] <= "r";
            uart_buf[22] <= "e";
            uart_buf[23] <= "c";
            uart_buf[24] <= "t";
            uart_buf[25] <= " ";
            uart_buf[26] <= ":";
            uart_buf[27] <= " ";
            uart_buf[28] <= to_digit(cc_h[3:0]);   // hundreds
            uart_buf[29] <= to_digit(cc_m[3:0]);   // tens
            uart_buf[30] <= to_digit(cc_l[3:0]);   // units
            uart_buf[31] <= " ";
            uart_buf[32] <= "/";
            uart_buf[33] <= " ";
            uart_buf[34] <= "2";
            uart_buf[35] <= "4";
            uart_buf[36] <= "0";
            uart_buf[37] <= 8'h0D;  // \r
            uart_buf[38] <= 8'h0A;  // \n

            // ── Line 3: "Accuracy: XXX.XX%\r\n" (19 bytes, [39..57]) ──
            uart_buf[39] <= "A";
            uart_buf[40] <= "c";
            uart_buf[41] <= "c";
            uart_buf[42] <= "u";
            uart_buf[43] <= "r";
            uart_buf[44] <= "a";
            uart_buf[45] <= "c";
            uart_buf[46] <= "y";
            uart_buf[47] <= ":";
            uart_buf[48] <= " ";
            uart_buf[49] <= to_digit(d_i2[3:0]);   // hundreds
            uart_buf[50] <= to_digit(d_i1[3:0]);   // tens
            uart_buf[51] <= to_digit(d_i0[3:0]);   // units
            uart_buf[52] <= ".";
            uart_buf[53] <= to_digit(d_f1[3:0]);   // tenths
            uart_buf[54] <= to_digit(d_f0[3:0]);   // hundredths
            uart_buf[55] <= "%";
            uart_buf[56] <= 8'h0D;  // \r
            uart_buf[57] <= 8'h0A;  // \n

            uart_total_bytes <= MSG_LEN;
        end
    end

    // UART TX trigger
    reg uart_send_req;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) uart_send_req <= 1'b0;
        else        uart_send_req <= uart_trigger;
    end

    // UART TX state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state      <= TX_IDLE;
            tx_reg        <= 1'b1;
            baud_cnt      <= 10'd0;
            bit_idx       <= 3'd0;
            uart_byte_idx <= 6'd0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx_reg <= 1'b1;
                    if (uart_send_req) begin
                        uart_byte_idx <= 6'd0;
                        baud_cnt      <= 10'd0;
                        tx_reg        <= 1'b0;  // start bit
                        tx_state      <= TX_START;
                    end
                end

                TX_START: begin
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 10'd0;
                        bit_idx  <= 3'd0;
                        tx_reg   <= uart_buf[uart_byte_idx][0];
                        tx_state <= TX_DATA;
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end

                TX_DATA: begin
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 10'd0;
                        if (bit_idx == 3'd7) begin
                            tx_reg   <= 1'b1;  // stop bit
                            tx_state <= TX_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                            tx_reg  <= uart_buf[uart_byte_idx][bit_idx + 1];
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end

                TX_STOP: begin
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 10'd0;
                        if (uart_byte_idx == uart_total_bytes - 1) begin
                            tx_state <= TX_IDLE;
                        end else begin
                            uart_byte_idx <= uart_byte_idx + 6'd1;
                            tx_reg        <= 1'b0;  // start bit of next byte
                            tx_state      <= TX_START;
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 10'd1;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // LED Display
    // =========================================================================
    assign led[7:0]  = ar_index;       // current sample index (0-239)
    assign led[15:8] = correct_cnt;    // correct predictions so far

    // =========================================================================
    // 7-Segment Display
    // =========================================================================
    assign an = 4'b1110;  // AN0 active only (rightmost digit)
    assign dp = ar_done ? 1'b0 : 1'b1;  // DP: 0 when done, 1 during run

    function [6:0] digit_to_seg;
        input [3:0] d;
        begin
            case (d)
                4'd0: digit_to_seg = 7'b1000000;  // 0
                4'd1: digit_to_seg = 7'b1111001;  // 1
                4'd2: digit_to_seg = 7'b0100100;  // 2
                4'd3: digit_to_seg = 7'b0110000;  // 3
                4'd4: digit_to_seg = 7'b0011001;  // 4
                4'd5: digit_to_seg = 7'b0010010;  // 5
                4'd6: digit_to_seg = 7'b0000010;  // 6
                4'd7: digit_to_seg = 7'b1111000;  // 7
                4'd8: digit_to_seg = 7'b0000000;  // 8
                4'd9: digit_to_seg = 7'b0010000;  // 9
                default: digit_to_seg = 7'b0001000;  // "H" for help/done
            endcase
        end
    endfunction

    // Show "d" (done) on 7-seg when finished, else sample units digit (0-9)
    assign seg = ar_done ? 7'b0101111 : digit_to_seg(ar_index % 4'd10);

endmodule
