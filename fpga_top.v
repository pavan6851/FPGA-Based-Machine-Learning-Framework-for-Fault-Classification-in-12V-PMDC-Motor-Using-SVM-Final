// =============================================================================
//  fpga_top.v  -  FPGA Demo Wrapper  (v4 - UART output added)
//  PMDC Motor Fault Detection - SVM Inference Core (OvR, LinearSVC)
//
//  Board : Digilent Nexys 4 (XC7A100T-1CSG324C)
//  Clock : 100 MHz onboard crystal
//
//  Changes from v3:
//    - Added UART TX (115200 8N1) on pin uart_tx
//    - On result_valid, sends ASCII string over UART to PuTTY:
//        "SVM: Smp=N Pred=X [LABEL] Exp=Y PASS\r\n"  or  "...FAIL\r\n"
//    - uart_tx added to port list (connect to JA[0] or dedicated UART pin)
//    - All existing LED/7-seg/button logic unchanged
//
//  UART format example:
//    SVM: Smp=3 Pred=1 [F1 ] Exp=1 PASS
//    SVM: Smp=5 Pred=2 [F2 ] Exp=3 FAIL
//
//  XDC addition needed:
//    set_property PACKAGE_PIN D4  [get_ports uart_tx]   ;# Nexys4 UART_TXD_IN
//    set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
// =============================================================================

`timescale 1ns/1ps

module fpga_top (
    input  wire        clk,
    input  wire        btnC,
    input  wire        btnR,
    input  wire        btnL,
    input  wire        sw0,

    output wire [15:0] led,
    output wire [6:0]  seg,
    output wire [3:0]  an,
    output wire        dp,
    output wire        uart_tx       // NEW: connect to Nexys4 UART TX pin
);

    // -------------------------------------------------------------------------
    // Reset
    // -------------------------------------------------------------------------
    wire rst_n = ~sw0;

    // -------------------------------------------------------------------------
    // 8 representative samples (2 per class)
    // -------------------------------------------------------------------------

    // ── Class 0: Healthy - Line 1 ─────────────────────────────────────────────
    localparam signed [31:0] S0_F0  =  32'sd542110;
    localparam signed [31:0] S0_F1  = -32'sd541765;
    localparam signed [31:0] S0_F2  =  32'sd541766;
    localparam signed [31:0] S0_F3  =  32'sd574076;
    localparam signed [31:0] S0_F4  = -32'sd541765;
    localparam signed [31:0] S0_F5  = -32'sd1827978;
    localparam signed [31:0] S0_F6  = -32'sd3949404;
    localparam signed [31:0] S0_F7  = -32'sd6569152;
    localparam signed [31:0] S0_F8  =  32'sd5419851;
    localparam signed [31:0] S0_F9  = -32'sd9517836;
    localparam signed [31:0] S0_F10 = -32'sd8209230;
    localparam signed [31:0] S0_F11 = -32'sd1871887;
    localparam signed [31:0] S0_F12 = -32'sd492033;
    localparam signed [31:0] S0_F13 = -32'sd293183;
    localparam signed [31:0] S0_F14 = -32'sd230663;
    localparam        [1:0]  S0_EXP =  2'd0;

    // ── Class 0: Healthy - Line 6 ─────────────────────────────────────────────
    localparam signed [31:0] S1_F0  =  32'sd541101;
    localparam signed [31:0] S1_F1  = -32'sd541765;
    localparam signed [31:0] S1_F2  =  32'sd541766;
    localparam signed [31:0] S1_F3  =  32'sd11998089;
    localparam signed [31:0] S1_F4  = -32'sd541765;
    localparam signed [31:0] S1_F5  =  32'sd24019536;
    localparam signed [31:0] S1_F6  = -32'sd8718584;
    localparam signed [31:0] S1_F7  =  32'sd42786269;
    localparam signed [31:0] S1_F8  = -32'sd2119822;
    localparam signed [31:0] S1_F9  = -32'sd3145867;
    localparam signed [31:0] S1_F10 = -32'sd9667964;
    localparam signed [31:0] S1_F11 = -32'sd2157754;
    localparam signed [31:0] S1_F12 = -32'sd763594;
    localparam signed [31:0] S1_F13 = -32'sd534731;
    localparam signed [31:0] S1_F14 = -32'sd434922;
    localparam        [1:0]  S1_EXP =  2'd0;

    // ── Class 1: Fault 1 - Line 4 ─────────────────────────────────────────────
    localparam signed [31:0] S2_F0  =  32'sd541089;
    localparam signed [31:0] S2_F1  = -32'sd541765;
    localparam signed [31:0] S2_F2  =  32'sd541766;
    localparam signed [31:0] S2_F3  = -32'sd15061909;
    localparam signed [31:0] S2_F4  = -32'sd541765;
    localparam signed [31:0] S2_F5  = -32'sd22779039;
    localparam signed [31:0] S2_F6  =  32'sd23047555;
    localparam signed [31:0] S2_F7  = -32'sd8607886;
    localparam signed [31:0] S2_F8  = -32'sd9271679;
    localparam signed [31:0] S2_F9  =  32'sd5062325;
    localparam signed [31:0] S2_F10 = -32'sd10224153;
    localparam signed [31:0] S2_F11 = -32'sd2916906;
    localparam signed [31:0] S2_F12 = -32'sd1535793;
    localparam signed [31:0] S2_F13 = -32'sd1305048;
    localparam signed [31:0] S2_F14 = -32'sd1202077;
    localparam        [1:0]  S2_EXP =  2'd1;

    // ── Class 1: Fault 1 - Line 7 ─────────────────────────────────────────────
    localparam signed [31:0] S3_F0  =  32'sd541014;
    localparam signed [31:0] S3_F1  = -32'sd541765;
    localparam signed [31:0] S3_F2  =  32'sd541766;
    localparam signed [31:0] S3_F3  = -32'sd15202031;
    localparam signed [31:0] S3_F4  = -32'sd541765;
    localparam signed [31:0] S3_F5  = -32'sd4300073;
    localparam signed [31:0] S3_F6  = -32'sd828352;
    localparam signed [31:0] S3_F7  = -32'sd6570108;
    localparam signed [31:0] S3_F8  =  32'sd2960541;
    localparam signed [31:0] S3_F9  = -32'sd7657446;
    localparam signed [31:0] S3_F10 = -32'sd8864478;
    localparam signed [31:0] S3_F11 = -32'sd2513746;
    localparam signed [31:0] S3_F12 = -32'sd1130583;
    localparam signed [31:0] S3_F13 = -32'sd908529;
    localparam signed [31:0] S3_F14 = -32'sd817156;
    localparam        [1:0]  S3_EXP =  2'd1;

    // ── Class 2: Fault 2 - Line 2 ─────────────────────────────────────────────
    localparam signed [31:0] S4_F0  =  32'sd541570;
    localparam signed [31:0] S4_F1  = -32'sd541765;
    localparam signed [31:0] S4_F2  =  32'sd541766;
    localparam signed [31:0] S4_F3  = -32'sd5136719;
    localparam signed [31:0] S4_F4  = -32'sd541765;
    localparam signed [31:0] S4_F5  = -32'sd1732921;
    localparam signed [31:0] S4_F6  =  32'sd918156;
    localparam signed [31:0] S4_F7  = -32'sd4337152;
    localparam signed [31:0] S4_F8  = -32'sd9567891;
    localparam signed [31:0] S4_F9  =  32'sd5312055;
    localparam signed [31:0] S4_F10 = -32'sd10250062;
    localparam signed [31:0] S4_F11 = -32'sd2858428;
    localparam signed [31:0] S4_F12 = -32'sd1476031;
    localparam signed [31:0] S4_F13 = -32'sd1244971;
    localparam signed [31:0] S4_F14 = -32'sd1141614;
    localparam        [1:0]  S4_EXP =  2'd2;

    // ── Class 2: Fault 2 - Line 10 ────────────────────────────────────────────
    localparam signed [31:0] S5_F0  =  32'sd545787;
    localparam signed [31:0] S5_F1  = -32'sd541765;
    localparam signed [31:0] S5_F2  =  32'sd541766;
    localparam signed [31:0] S5_F3  =  32'sd75221121;
    localparam signed [31:0] S5_F4  = -32'sd541765;
    localparam signed [31:0] S5_F5  =  32'sd10939312;
    localparam signed [31:0] S5_F6  = -32'sd8590644;
    localparam signed [31:0] S5_F7  = -32'sd2215780;
    localparam signed [31:0] S5_F8  = -32'sd6261055;
    localparam signed [31:0] S5_F9  =  32'sd918215;
    localparam signed [31:0] S5_F10 = -32'sd10087282;
    localparam signed [31:0] S5_F11 = -32'sd2341153;
    localparam signed [31:0] S5_F12 = -32'sd947897;
    localparam signed [31:0] S5_F13 = -32'sd714892;
    localparam signed [31:0] S5_F14 = -32'sd609289;
    localparam        [1:0]  S5_EXP =  2'd2;

    // ── Class 3: Fault 3 - Line 3 ─────────────────────────────────────────────
    localparam signed [31:0] S6_F0  =  32'sd541541;
    localparam signed [31:0] S6_F1  = -32'sd541765;
    localparam signed [31:0] S6_F2  =  32'sd541766;
    localparam signed [31:0] S6_F3  = -32'sd7610793;
    localparam signed [31:0] S6_F4  = -32'sd541765;
    localparam signed [31:0] S6_F5  =  32'sd7479160;
    localparam signed [31:0] S6_F6  = -32'sd18299897;
    localparam signed [31:0] S6_F7  = -32'sd6662205;
    localparam signed [31:0] S6_F8  =  32'sd21689874;
    localparam signed [31:0] S6_F9  = -32'sd15567391;
    localparam signed [31:0] S6_F10 =  32'sd26957022;
    localparam signed [31:0] S6_F11 =  32'sd3032263;
    localparam signed [31:0] S6_F12 =  32'sd271697;
    localparam signed [31:0] S6_F13 = -32'sd427797;
    localparam signed [31:0] S6_F14 = -32'sd640990;
    localparam        [1:0]  S6_EXP =  2'd3;

    // ── Class 3: Fault 3 - Line 5 ─────────────────────────────────────────────
    localparam signed [31:0] S7_F0  =  32'sd541409;
    localparam signed [31:0] S7_F1  = -32'sd541765;
    localparam signed [31:0] S7_F2  =  32'sd541766;
    localparam signed [31:0] S7_F3  = -32'sd7712178;
    localparam signed [31:0] S7_F4  = -32'sd541765;
    localparam signed [31:0] S7_F5  =  32'sd7721334;
    localparam signed [31:0] S7_F6  = -32'sd18838047;
    localparam signed [31:0] S7_F7  = -32'sd4467275;
    localparam signed [31:0] S7_F8  =  32'sd22942206;
    localparam signed [31:0] S7_F9  = -32'sd15533064;
    localparam signed [31:0] S7_F10 =  32'sd37013284;
    localparam signed [31:0] S7_F11 =  32'sd2224407;
    localparam signed [31:0] S7_F12 =  32'sd36869;
    localparam signed [31:0] S7_F13 = -32'sd59502;
    localparam signed [31:0] S7_F14 = -32'sd277925;
    localparam        [1:0]  S7_EXP =  2'd3;

    // -------------------------------------------------------------------------
    // Button debounce - 20 ms @ 100 MHz
    // -------------------------------------------------------------------------
    localparam DEBOUNCE_MAX = 20'd2_000_000;

    reg [19:0] db_cntC, db_cntR, db_cntL;
    reg btnC_sync0, btnC_sync1, btnC_db, btnC_prev;
    reg btnR_sync0, btnR_sync1, btnR_db, btnR_prev;
    reg btnL_sync0, btnL_sync1, btnL_db, btnL_prev;

    always @(posedge clk) begin
        btnC_sync0 <= btnC; btnC_sync1 <= btnC_sync0;
        btnR_sync0 <= btnR; btnR_sync1 <= btnR_sync0;
        btnL_sync0 <= btnL; btnL_sync1 <= btnL_sync0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            db_cntC <= 0; btnC_db <= 0;
            db_cntR <= 0; btnR_db <= 0;
            db_cntL <= 0; btnL_db <= 0;
        end else begin
            if (btnC_sync1 != btnC_db) begin
                if (db_cntC == DEBOUNCE_MAX-1) begin btnC_db <= btnC_sync1; db_cntC <= 0; end
                else db_cntC <= db_cntC + 1;
            end else db_cntC <= 0;

            if (btnR_sync1 != btnR_db) begin
                if (db_cntR == DEBOUNCE_MAX-1) begin btnR_db <= btnR_sync1; db_cntR <= 0; end
                else db_cntR <= db_cntR + 1;
            end else db_cntR <= 0;

            if (btnL_sync1 != btnL_db) begin
                if (db_cntL == DEBOUNCE_MAX-1) begin btnL_db <= btnL_sync1; db_cntL <= 0; end
                else db_cntL <= db_cntL + 1;
            end else db_cntL <= 0;
        end
    end

    always @(posedge clk) begin
        btnC_prev <= btnC_db;
        btnR_prev <= btnR_db;
        btnL_prev <= btnL_db;
    end

    wire btnC_pulse = btnC_db & ~btnC_prev;
    wire btnR_pulse = btnR_db & ~btnR_prev;
    wire btnL_pulse = btnL_db & ~btnL_prev;

    // -------------------------------------------------------------------------
    // Sample selector
    // -------------------------------------------------------------------------
    reg [2:0] sample_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          sample_sel <= 3'd0;
        else if (btnR_pulse) sample_sel <= (sample_sel == 3'd7) ? 3'd0 : sample_sel + 3'd1;
        else if (btnL_pulse) sample_sel <= (sample_sel == 3'd0) ? 3'd7 : sample_sel - 3'd1;
    end

    // -------------------------------------------------------------------------
    // Feature MUX
    // -------------------------------------------------------------------------
    reg signed [31:0] feat_data_0,  feat_data_1,  feat_data_2,  feat_data_3;
    reg signed [31:0] feat_data_4,  feat_data_5,  feat_data_6,  feat_data_7;
    reg signed [31:0] feat_data_8,  feat_data_9,  feat_data_10, feat_data_11;
    reg signed [31:0] feat_data_12, feat_data_13, feat_data_14;
    reg [1:0] expected_class;

    always @(*) begin
        case (sample_sel)
            3'd0: begin
                feat_data_0=S0_F0; feat_data_1=S0_F1; feat_data_2=S0_F2;
                feat_data_3=S0_F3; feat_data_4=S0_F4; feat_data_5=S0_F5;
                feat_data_6=S0_F6; feat_data_7=S0_F7; feat_data_8=S0_F8;
                feat_data_9=S0_F9; feat_data_10=S0_F10; feat_data_11=S0_F11;
                feat_data_12=S0_F12; feat_data_13=S0_F13; feat_data_14=S0_F14;
                expected_class = S0_EXP;
            end
            3'd1: begin
                feat_data_0=S1_F0; feat_data_1=S1_F1; feat_data_2=S1_F2;
                feat_data_3=S1_F3; feat_data_4=S1_F4; feat_data_5=S1_F5;
                feat_data_6=S1_F6; feat_data_7=S1_F7; feat_data_8=S1_F8;
                feat_data_9=S1_F9; feat_data_10=S1_F10; feat_data_11=S1_F11;
                feat_data_12=S1_F12; feat_data_13=S1_F13; feat_data_14=S1_F14;
                expected_class = S1_EXP;
            end
            3'd2: begin
                feat_data_0=S2_F0; feat_data_1=S2_F1; feat_data_2=S2_F2;
                feat_data_3=S2_F3; feat_data_4=S2_F4; feat_data_5=S2_F5;
                feat_data_6=S2_F6; feat_data_7=S2_F7; feat_data_8=S2_F8;
                feat_data_9=S2_F9; feat_data_10=S2_F10; feat_data_11=S2_F11;
                feat_data_12=S2_F12; feat_data_13=S2_F13; feat_data_14=S2_F14;
                expected_class = S2_EXP;
            end
            3'd3: begin
                feat_data_0=S3_F0; feat_data_1=S3_F1; feat_data_2=S3_F2;
                feat_data_3=S3_F3; feat_data_4=S3_F4; feat_data_5=S3_F5;
                feat_data_6=S3_F6; feat_data_7=S3_F7; feat_data_8=S3_F8;
                feat_data_9=S3_F9; feat_data_10=S3_F10; feat_data_11=S3_F11;
                feat_data_12=S3_F12; feat_data_13=S3_F13; feat_data_14=S3_F14;
                expected_class = S3_EXP;
            end
            3'd4: begin
                feat_data_0=S4_F0; feat_data_1=S4_F1; feat_data_2=S4_F2;
                feat_data_3=S4_F3; feat_data_4=S4_F4; feat_data_5=S4_F5;
                feat_data_6=S4_F6; feat_data_7=S4_F7; feat_data_8=S4_F8;
                feat_data_9=S4_F9; feat_data_10=S4_F10; feat_data_11=S4_F11;
                feat_data_12=S4_F12; feat_data_13=S4_F13; feat_data_14=S4_F14;
                expected_class = S4_EXP;
            end
            3'd5: begin
                feat_data_0=S5_F0; feat_data_1=S5_F1; feat_data_2=S5_F2;
                feat_data_3=S5_F3; feat_data_4=S5_F4; feat_data_5=S5_F5;
                feat_data_6=S5_F6; feat_data_7=S5_F7; feat_data_8=S5_F8;
                feat_data_9=S5_F9; feat_data_10=S5_F10; feat_data_11=S5_F11;
                feat_data_12=S5_F12; feat_data_13=S5_F13; feat_data_14=S5_F14;
                expected_class = S5_EXP;
            end
            3'd6: begin
                feat_data_0=S6_F0; feat_data_1=S6_F1; feat_data_2=S6_F2;
                feat_data_3=S6_F3; feat_data_4=S6_F4; feat_data_5=S6_F5;
                feat_data_6=S6_F6; feat_data_7=S6_F7; feat_data_8=S6_F8;
                feat_data_9=S6_F9; feat_data_10=S6_F10; feat_data_11=S6_F11;
                feat_data_12=S6_F12; feat_data_13=S6_F13; feat_data_14=S6_F14;
                expected_class = S6_EXP;
            end
            default: begin
                feat_data_0=S7_F0; feat_data_1=S7_F1; feat_data_2=S7_F2;
                feat_data_3=S7_F3; feat_data_4=S7_F4; feat_data_5=S7_F5;
                feat_data_6=S7_F6; feat_data_7=S7_F7; feat_data_8=S7_F8;
                feat_data_9=S7_F9; feat_data_10=S7_F10; feat_data_11=S7_F11;
                feat_data_12=S7_F12; feat_data_13=S7_F13; feat_data_14=S7_F14;
                expected_class = S7_EXP;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // feat_valid pulse
    // -------------------------------------------------------------------------
    reg feat_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) feat_valid <= 1'b0;
        else        feat_valid <= btnC_pulse;
    end

    // -------------------------------------------------------------------------
    // SVM inference core
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Result latch
    // -------------------------------------------------------------------------
    reg [1:0]  last_class;
    reg [11:0] last_votes;
    reg [1:0]  last_expected;
    reg        result_latched;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_class     <= 2'd0;
            last_votes     <= 12'd0;
            last_expected  <= 2'd0;
            result_latched <= 1'b0;
        end else if (result_valid) begin
            last_class     <= result_class;
            last_votes     <= result_votes_dbg;
            last_expected  <= expected_class;
            result_latched <= 1'b1;
        end else if (btnR_pulse || btnL_pulse) begin
            result_latched <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Running accuracy counter
    // -------------------------------------------------------------------------
    reg [3:0] pass_count;
    reg [3:0] total_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pass_count  <= 4'd0;
            total_count <= 4'd0;
        end else if (result_valid) begin
            total_count <= total_count + 4'd1;
            if (result_class == expected_class)
                pass_count <= pass_count + 4'd1;
        end
    end

    // =========================================================================
    // UART TX  -  115200 baud, 8N1, 100 MHz clock
    //
    // On result_valid, loads a 35-byte message into uart_buf and sends it
    // byte by byte.
    //
    // Message format (35 bytes including \r\n):
    //   "SVM: Smp=N Pred=X [LBL] Exp=Y PASS\r\n"   (PASS or FAIL)
    //   e.g. "SVM: Smp=3 Pred=1 [F1 ] Exp=1 PASS\r\n"
    //
    // Label table:
    //   0 -> "H  "  (Healthy)
    //   1 -> "F1 "  (Fault1)
    //   2 -> "F2 "  (Fault2)
    //   3 -> "F3 "  (Fault3)
    // =========================================================================

    // 100 MHz / 115200 = 868.055... -> use 868
    localparam BAUD_DIV = 868;

    // TX state machine
    localparam TX_IDLE   = 2'd0;
    localparam TX_START  = 2'd1;
    localparam TX_DATA   = 2'd2;
    localparam TX_STOP   = 2'd3;

    // Buffer: 35 bytes max
    localparam MSG_LEN = 35;

    reg [7:0]  uart_buf [0:MSG_LEN-1];
    reg [5:0]  uart_byte_idx;   // which byte we're sending
    reg [5:0]  uart_total_bytes;
    reg [9:0]  baud_cnt;
    reg [2:0]  bit_idx;
    reg [1:0]  tx_state;
    reg        tx_reg;          // the actual TX line

    assign uart_tx = tx_reg;

    // Capture sample_sel at result_valid (pipeline: sample_sel is stable then)
    reg [2:0] uart_smp;
    reg [1:0] uart_pred;
    reg [1:0] uart_exp;
    reg       uart_pass;

    // ---- Build message helper wires ----
    // ASCII digit from 2-bit value
    function [7:0] to_digit;
        input [3:0] v;
        begin to_digit = 8'd48 + {4'd0, v}; end
    endfunction

    // ASCII for class label, 3 chars
    // Returns one character at a time: pos 0,1,2
    function [7:0] class_char;
        input [1:0] cls;
        input [1:0] pos;
        begin
            case ({cls, pos})
                4'b00_00: class_char = "H";
                4'b00_01: class_char = " ";
                4'b00_10: class_char = " ";
                4'b01_00: class_char = "F";
                4'b01_01: class_char = "1";
                4'b01_10: class_char = " ";
                4'b10_00: class_char = "F";
                4'b10_01: class_char = "2";
                4'b10_10: class_char = " ";
                4'b11_00: class_char = "F";
                4'b11_01: class_char = "3";
                4'b11_10: class_char = " ";
                default:  class_char = " ";
            endcase
        end
    endfunction

    // ---- Trigger: capture result, build buffer ----
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_smp   <= 3'd0;
            uart_pred  <= 2'd0;
            uart_exp   <= 2'd0;
            uart_pass  <= 1'b0;
            uart_total_bytes <= 0;
        end else if (result_valid) begin
            uart_smp  <= sample_sel;
            uart_pred <= result_class;
            uart_exp  <= expected_class;
            uart_pass <= (result_class == expected_class);

            // "SVM: Smp=N Pred=X [LBL] Exp=Y PASS\r\n"
            //  0123456789...
            uart_buf[0]  <= "S";
            uart_buf[1]  <= "V";
            uart_buf[2]  <= "M";
            uart_buf[3]  <= ":";
            uart_buf[4]  <= " ";
            uart_buf[5]  <= "S";
            uart_buf[6]  <= "m";
            uart_buf[7]  <= "p";
            uart_buf[8]  <= "=";
            uart_buf[9]  <= to_digit({1'b0, sample_sel});
            uart_buf[10] <= " ";
            uart_buf[11] <= "P";
            uart_buf[12] <= "r";
            uart_buf[13] <= "e";
            uart_buf[14] <= "d";
            uart_buf[15] <= "=";
            uart_buf[16] <= to_digit({2'b00, result_class});
            uart_buf[17] <= " ";
            uart_buf[18] <= "[";
            uart_buf[19] <= class_char(result_class, 2'd0);
            uart_buf[20] <= class_char(result_class, 2'd1);
            uart_buf[21] <= class_char(result_class, 2'd2);
            uart_buf[22] <= "]";
            uart_buf[23] <= " ";
            uart_buf[24] <= "E";
            uart_buf[25] <= "x";
            uart_buf[26] <= "p";
            uart_buf[27] <= "=";
            uart_buf[28] <= to_digit({2'b00, expected_class});
            uart_buf[29] <= " ";
            // PASS or FAIL
            uart_buf[30] <= (result_class == expected_class) ? "P" : "F";
            uart_buf[31] <= (result_class == expected_class) ? "A" : "A";
            uart_buf[32] <= (result_class == expected_class) ? "S" : "I";
            uart_buf[33] <= (result_class == expected_class) ? "S" : "L";
            uart_buf[34] <= 8'h0D;  // \r
            uart_buf[35-1] <= 8'h0A; // \n  -- same index 34, \n sent last

            uart_total_bytes <= MSG_LEN;  // 35 bytes
        end
    end

    // ---- UART TX state machine ----
    // Triggered when uart_total_bytes is set and TX is idle
    reg uart_send_req;
    reg uart_send_req_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) uart_send_req <= 1'b0;
        else        uart_send_req <= result_valid;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state       <= TX_IDLE;
            tx_reg         <= 1'b1;
            baud_cnt       <= 10'd0;
            bit_idx        <= 3'd0;
            uart_byte_idx  <= 6'd0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx_reg <= 1'b1;
                    if (uart_send_req) begin
                        uart_byte_idx <= 6'd0;
                        baud_cnt      <= 10'd0;
                        tx_reg        <= 1'b0;   // start bit
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
                            tx_reg   <= 1'b1;   // stop bit
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
                            tx_state <= TX_IDLE;  // all bytes sent
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

    // -------------------------------------------------------------------------
    // dp - decimal point blinks on result_valid
    // -------------------------------------------------------------------------
    reg [23:0] dp_timer;
    reg        dp_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dp_timer <= 24'd0;
            dp_reg   <= 1'b1;
        end else if (result_valid) begin
            dp_timer <= 24'd10_000_000;
            dp_reg   <= 1'b0;
        end else if (dp_timer != 0) begin
            dp_timer <= dp_timer - 24'd1;
        end else begin
            dp_reg <= 1'b1;
        end
    end
    assign dp = dp_reg;

    // -------------------------------------------------------------------------
    // LED assignments
    // -------------------------------------------------------------------------
    wire mismatch = result_latched & (last_class != last_expected);

    assign led[1:0]  = result_latched ? last_class : 2'b00;
    assign led[3:2]  = expected_class;
    assign led[4]    = result_latched;
    assign led[5]    = mismatch;
    assign led[6]    = feat_valid;
    assign led[7]    = 1'b0;
    assign led[10:8] = sample_sel;
    assign led[11]   = 1'b0;
    assign led[12]   = result_latched & (last_class == 2'd0);
    assign led[13]   = result_latched & (last_class == 2'd1);
    assign led[14]   = result_latched & (last_class == 2'd2);
    assign led[15]   = result_latched & (last_class == 2'd3);

    // -------------------------------------------------------------------------
    // 7-segment - single rightmost digit on AN0
    // -------------------------------------------------------------------------
    assign an = 4'b1110;

    function [6:0] digit_to_seg;
        input [3:0] d;
        begin
            case (d)
                4'd0: digit_to_seg = 7'b1000000;
                4'd1: digit_to_seg = 7'b1111001;
                4'd2: digit_to_seg = 7'b0100100;
                4'd3: digit_to_seg = 7'b0110000;
                4'd4: digit_to_seg = 7'b0011001;
                4'd5: digit_to_seg = 7'b0010010;
                4'd6: digit_to_seg = 7'b0000010;
                4'd7: digit_to_seg = 7'b1111000;
                default: digit_to_seg = 7'b1111111;
            endcase
        end
    endfunction

    localparam SEG_E = 7'b0000110;

    reg [6:0] seg_reg;
    always @(*) begin
        if (!result_latched)
            seg_reg = digit_to_seg({1'b0, sample_sel});
        else if (mismatch)
            seg_reg = SEG_E;
        else
            seg_reg = digit_to_seg({2'b00, last_class});
    end

    assign seg = seg_reg;

endmodule
