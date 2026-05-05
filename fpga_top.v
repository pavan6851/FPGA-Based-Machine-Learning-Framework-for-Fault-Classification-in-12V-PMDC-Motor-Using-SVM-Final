// =============================================================================
//  fpga_top.v  -  FPGA Demo Wrapper  (v3 - 96.25% accuracy build)
//  PMDC Motor Fault Detection - SVM Inference Core (OvO, LinearSVC)
//
//  Board : Digilent Nexys 4 (XC7A100T-1CSG324C)
//  Clock : 100 MHz onboard crystal
//
//  Changes from v2 (95.00% build):
//    - 8 hardcoded representative samples updated from new dataset
//      (tb_vectors.txt from regenerate_params.py, new svm_model.pkl)
//    - Accuracy reference updated to 96.25%
//    - svm_dot_product.v v6: registered multipliers fixed timing (WNS ok)
//      pipeline latency still 5 cycles, no interface change here
//
//  Controls:
//    SW0   - active-high reset
//    btnR  - next sample  (0→1→…→7→0)
//    btnL  - prev sample
//    btnC  - trigger inference on current sample
//
//  LED output:
//    led[1:0]   predicted class   (00=H  01=F1  10=F2  11=F3)
//    led[3:2]   expected class    (same encoding)
//    led[4]     result_latched    (inference done)
//    led[5]     mismatch flag     (1 = wrong prediction)
//    led[6]     feat_valid        (inference triggered this cycle)
//    led[7]     0 (spare)
//    led[10:8]  sample_sel[2:0]
//    led[11]    0 (spare)
//    led[12]    Healthy won
//    led[13]    Fault1  won
//    led[14]    Fault2  won
//    led[15]    Fault3  won
//
//  7-segment (AN0, rightmost digit):
//    Before inference : selected sample index (0-7)
//    After correct    : predicted class  (0/1/2/3)
//    After mismatch   : 'E'
//    dp               : pulses on result_valid
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
    output wire        dp
);

    // -------------------------------------------------------------------------
    // Reset
    // -------------------------------------------------------------------------
    wire rst_n = ~sw0;

    // -------------------------------------------------------------------------
    // 8 representative samples (2 per class) from new tb_vectors.txt
    // All values are Q8.24 fixed-point signed integers.
    // Source lines: Class0=L1,L6 | Class1=L4,L7 | Class2=L2,L10 | Class3=L3,L5
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
    // Button debounce - 20 ms @ 100 MHz = 2,000,000 cycles
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
    // Sample selector - 3-bit for 8 samples (0-7)
    // -------------------------------------------------------------------------
    reg [2:0] sample_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          sample_sel <= 3'd0;
        else if (btnR_pulse) sample_sel <= (sample_sel == 3'd7) ? 3'd0 : sample_sel + 3'd1;
        else if (btnL_pulse) sample_sel <= (sample_sel == 3'd0) ? 3'd7 : sample_sel - 3'd1;
    end

    // -------------------------------------------------------------------------
    // Feature MUX - 8 samples
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
    // feat_valid - one-clock pulse when btnC pressed
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
    // Running accuracy counter (0-8 samples)
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

    // -------------------------------------------------------------------------
    // dp - decimal point blinks ~100 ms on result_valid
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
            dp_reg   <= 1'b1;
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
