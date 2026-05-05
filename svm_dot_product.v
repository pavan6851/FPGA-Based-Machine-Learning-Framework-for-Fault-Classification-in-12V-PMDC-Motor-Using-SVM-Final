// =============================================================================
//  svm_dot_product.v  (v10 - Correct DSP48E1 inference)
//
//  FIXES FROM v9:
//
//  BUG 1 - (* use_dsp = "yes" *) on wire is ignored by Vivado:
//    Vivado only honours this attribute on `reg` declarations whose
//    assignment is a multiply expression inside an always block.
//    Applying it to a combinational `wire` lets the synthesiser fall
//    through to its default heuristic (LUT constant-multiplier).
//    FIX: Remove the combinational prod_comb wires entirely.
//         Absorb the multiply directly into the Stage-0 always block
//         and place (* use_dsp = "yes" *) on the `prod` reg array.
//
//  BUG 2 - Weight constant seen as literal, triggering LUT path:
//    When one operand of a multiply is a compile-time constant the
//    synthesiser always prefers a LUT constant-multiplier over a DSP.
//    Even with use_dsp="yes" on the wire Vivado was ignoring it because
//    w18[] was effectively a constant vector of wires.
//    FIX: Register w18 into a `reg` (w18_reg) in Stage -1 alongside
//         x_reg.  Both operands are then FFs, and Vivado maps the
//         product FF <- FF*FF cleanly onto a single DSP48E1.
//
//  BUG 3 - Attribute syntax:
//    (* use_dsp = "yes" *) requires Vivado 2014.1+.
//    Older flows need (* use_dsp48 = "yes" *).
//    We emit both forms so the file works across tool versions.
//
//  UNCHANGED from v9:
//    - Fixed-point format: Q8.24 inputs, 18-bit truncation, <<4 correction
//    - 5-stage pipeline (stages -1 .. 4), total latency 6 registered cycles
//    - Adder tree widths and sign-extension
//    - Bias addition and output
//    - svm_fault_top.v / tb_svm_fault_top.v interface (PIPE_CYCLES=7,
//      valid_shift[5:0], valid_fire=[5]) -- all unchanged
//
//  EXPECTED TIMING AFTER FIX:
//    x_reg FF --> DSP48E1(A*B, output registered) --> prod FF
//    This is a single DSP48E1 delay = ~3.5-4 ns. Passes 100 MHz easily.
// =============================================================================

`timescale 1ns/1ps
`include "svm_params.vh"

module svm_dot_product #(
    parameter integer CLASSIFIER = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,

    input  wire signed [31:0] x0,
    input  wire signed [31:0] x1,
    input  wire signed [31:0] x2,
    input  wire signed [31:0] x3,
    input  wire signed [31:0] x4,
    input  wire signed [31:0] x5,
    input  wire signed [31:0] x6,
    input  wire signed [31:0] x7,
    input  wire signed [31:0] x8,
    input  wire signed [31:0] x9,
    input  wire signed [31:0] x10,
    input  wire signed [31:0] x11,
    input  wire signed [31:0] x12,
    input  wire signed [31:0] x13,
    input  wire signed [31:0] x14,

    output reg  signed [63:0] score,
    output reg                out_valid
);

    // -----------------------------------------------------------------------
    // Weight lookup - returns full Q8.24 32-bit signed constant
    // -----------------------------------------------------------------------
    function automatic signed [31:0] get_w;
        input integer cls;
        input integer feat;
        reg [5:0] key;
        begin
            key = {cls[1:0], feat[3:0]};
            case (key)
                6'h00: get_w=`W_0_0;  6'h01: get_w=`W_0_1;  6'h02: get_w=`W_0_2;
                6'h03: get_w=`W_0_3;  6'h04: get_w=`W_0_4;  6'h05: get_w=`W_0_5;
                6'h06: get_w=`W_0_6;  6'h07: get_w=`W_0_7;  6'h08: get_w=`W_0_8;
                6'h09: get_w=`W_0_9;  6'h0A: get_w=`W_0_10; 6'h0B: get_w=`W_0_11;
                6'h0C: get_w=`W_0_12; 6'h0D: get_w=`W_0_13; 6'h0E: get_w=`W_0_14;

                6'h10: get_w=`W_1_0;  6'h11: get_w=`W_1_1;  6'h12: get_w=`W_1_2;
                6'h13: get_w=`W_1_3;  6'h14: get_w=`W_1_4;  6'h15: get_w=`W_1_5;
                6'h16: get_w=`W_1_6;  6'h17: get_w=`W_1_7;  6'h18: get_w=`W_1_8;
                6'h19: get_w=`W_1_9;  6'h1A: get_w=`W_1_10; 6'h1B: get_w=`W_1_11;
                6'h1C: get_w=`W_1_12; 6'h1D: get_w=`W_1_13; 6'h1E: get_w=`W_1_14;

                6'h20: get_w=`W_2_0;  6'h21: get_w=`W_2_1;  6'h22: get_w=`W_2_2;
                6'h23: get_w=`W_2_3;  6'h24: get_w=`W_2_4;  6'h25: get_w=`W_2_5;
                6'h26: get_w=`W_2_6;  6'h27: get_w=`W_2_7;  6'h28: get_w=`W_2_8;
                6'h29: get_w=`W_2_9;  6'h2A: get_w=`W_2_10; 6'h2B: get_w=`W_2_11;
                6'h2C: get_w=`W_2_12; 6'h2D: get_w=`W_2_13; 6'h2E: get_w=`W_2_14;

                6'h30: get_w=`W_3_0;  6'h31: get_w=`W_3_1;  6'h32: get_w=`W_3_2;
                6'h33: get_w=`W_3_3;  6'h34: get_w=`W_3_4;  6'h35: get_w=`W_3_5;
                6'h36: get_w=`W_3_6;  6'h37: get_w=`W_3_7;  6'h38: get_w=`W_3_8;
                6'h39: get_w=`W_3_9;  6'h3A: get_w=`W_3_10; 6'h3B: get_w=`W_3_11;
                6'h3C: get_w=`W_3_12; 6'h3D: get_w=`W_3_13; 6'h3E: get_w=`W_3_14;

                default: get_w = 32'sh00000000;
            endcase
        end
    endfunction

    // -----------------------------------------------------------------------
    // Bias lookup - full Q8.24 64-bit signed
    // -----------------------------------------------------------------------
    function automatic signed [63:0] get_b;
        input integer cls;
        reg signed [63:0] tmp;
        begin
            case (cls)
                0: begin tmp = $signed(`B_0); get_b = tmp; end
                1: begin tmp = $signed(`B_1); get_b = tmp; end
                2: begin tmp = $signed(`B_2); get_b = tmp; end
                3: begin tmp = $signed(`B_3); get_b = tmp; end
                default: get_b = 64'sd0;
            endcase
        end
    endfunction

    // -----------------------------------------------------------------------
    // Pack flat ports into internal wire array
    // -----------------------------------------------------------------------
    wire signed [31:0] x [0:14];
    assign x[0]=x0;   assign x[1]=x1;   assign x[2]=x2;
    assign x[3]=x3;   assign x[4]=x4;   assign x[5]=x5;
    assign x[6]=x6;   assign x[7]=x7;   assign x[8]=x8;
    assign x[9]=x9;   assign x[10]=x10; assign x[11]=x11;
    assign x[12]=x12; assign x[13]=x13; assign x[14]=x14;

    // -----------------------------------------------------------------------
    // STAGE -1: Register feature inputs AND weight constants.
    //
    // KEY FIX: w18_reg is registered here alongside x_reg.
    // With both multiply operands as FFs (not a FF * constant-wire),
    // Vivado maps the Stage-0 multiply directly onto DSP48E1 A*B input FFs,
    // producing a clean FF->DSP->FF path with no LUT involvement.
    // -----------------------------------------------------------------------
    reg signed [17:0] x18_reg [0:14];   // top 18 bits of x (Q8.10)
    reg signed [17:0] w18_reg [0:14];   // top 18 bits of W (Q8.10)
    reg               valid_s_neg1;

    genvar j;
    generate
        for (j = 0; j < 15; j = j + 1) begin : g_stage_neg1
            // Truncate Q8.24 -> Q8.10 by keeping bits [31:14]
            wire signed [17:0] x18_next = x[j][31:14];
            wire signed [17:0] w18_next = $signed(get_w(CLASSIFIER, j)) >>> 14;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    x18_reg[j] <= 18'sd0;
                    w18_reg[j] <= 18'sd0;
                end else begin
                    if (in_valid) begin
                        x18_reg[j] <= x18_next;
                        w18_reg[j] <= w18_next;
                    end
                end
            end
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_s_neg1 <= 1'b0;
        else        valid_s_neg1 <= in_valid;
    end

    // -----------------------------------------------------------------------
    // STAGE 0: 18x18 signed multiply, registered into prod[].
    //
    // (* use_dsp = "yes" *) on the reg tells Vivado to map each
    // "prod[j] <= x18_reg[j] * w18_reg[j]" onto a single DSP48E1.
    // Both (* use_dsp = "yes" *) (Vivado 2014.1+) and
    // (* use_dsp48 = "yes" *) (older Vivado) are emitted for compatibility.
    //
    // Result width: 18+18 = 36 bits signed.
    // Numeric meaning: prod[j] = (x[j] >> 14) * (W[j] >> 14)
    //                           = x[j]*W[j] >> 28
    // (4 bits less than the target >>24 -- corrected by <<4 in Stage 4)
    // -----------------------------------------------------------------------
    (* use_dsp = "yes" *) (* use_dsp48 = "yes" *)
    reg signed [35:0] prod [0:14];
    reg               valid_s0;

    generate
        for (j = 0; j < 15; j = j + 1) begin : g_mul
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    prod[j] <= 36'sd0;
                else if (valid_s_neg1)
                    prod[j] <= x18_reg[j] * w18_reg[j];
            end
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) valid_s0 <= 1'b0;
        else        valid_s0 <= valid_s_neg1;
    end

    // -----------------------------------------------------------------------
    // STAGE 1: Level-0 adder tree (8 partial sums, 40-bit)
    // 36-bit signed + 36-bit signed: need 37 bits minimum; use 40 for margin.
    // -----------------------------------------------------------------------
    reg signed [39:0] l0 [0:7];
    reg               valid_l0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_l0 <= 1'b0;
            l0[0]<=0; l0[1]<=0; l0[2]<=0; l0[3]<=0;
            l0[4]<=0; l0[5]<=0; l0[6]<=0; l0[7]<=0;
        end else begin
            valid_l0 <= valid_s0;
            if (valid_s0) begin
                l0[0] <= {{4{prod[0][35]}},  prod[0]}  + {{4{prod[1][35]}},  prod[1]};
                l0[1] <= {{4{prod[2][35]}},  prod[2]}  + {{4{prod[3][35]}},  prod[3]};
                l0[2] <= {{4{prod[4][35]}},  prod[4]}  + {{4{prod[5][35]}},  prod[5]};
                l0[3] <= {{4{prod[6][35]}},  prod[6]}  + {{4{prod[7][35]}},  prod[7]};
                l0[4] <= {{4{prod[8][35]}},  prod[8]}  + {{4{prod[9][35]}},  prod[9]};
                l0[5] <= {{4{prod[10][35]}}, prod[10]} + {{4{prod[11][35]}}, prod[11]};
                l0[6] <= {{4{prod[12][35]}}, prod[12]} + {{4{prod[13][35]}}, prod[13]};
                l0[7] <= {{4{prod[14][35]}}, prod[14]};   // odd one out
            end
        end
    end

    // -----------------------------------------------------------------------
    // STAGE 2: Level-1 adder tree (4 partial sums, 41-bit)
    // -----------------------------------------------------------------------
    reg signed [40:0] l1 [0:3];
    reg               valid_l1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_l1 <= 1'b0;
            l1[0]<=0; l1[1]<=0; l1[2]<=0; l1[3]<=0;
        end else begin
            valid_l1 <= valid_l0;
            if (valid_l0) begin
                l1[0] <= {l0[0][39], l0[0]} + {l0[1][39], l0[1]};
                l1[1] <= {l0[2][39], l0[2]} + {l0[3][39], l0[3]};
                l1[2] <= {l0[4][39], l0[4]} + {l0[5][39], l0[5]};
                l1[3] <= {l0[6][39], l0[6]} + {l0[7][39], l0[7]};
            end
        end
    end

    // -----------------------------------------------------------------------
    // STAGE 3: Level-2 adder tree (2 partial sums, 42-bit)
    // -----------------------------------------------------------------------
    reg signed [41:0] l2 [0:1];
    reg               valid_l2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_l2 <= 1'b0;
            l2[0] <= 0; l2[1] <= 0;
        end else begin
            valid_l2 <= valid_l1;
            if (valid_l1) begin
                l2[0] <= {l1[0][40], l1[0]} + {l1[1][40], l1[1]};
                l2[1] <= {l1[2][40], l1[2]} + {l1[3][40], l1[3]};
            end
        end
    end

    // -----------------------------------------------------------------------
    // STAGE 4: Final sum + scale correction + bias
    //
    // dot_sum  = l2[0] + l2[1]   (43-bit to hold the final carry)
    // dot_sum represents sum(x[j]*W[j]) >> 28
    //
    // Target score format is Q8.24, i.e. sum(x[j]*W[j]) >> 24.
    // Correction: score_dot = dot_sum << 4.
    //
    // Bias get_b() is already in Q8.24 (sign-extended to 64-bit), so
    // adding it directly to score_dot gives the final Q8.24 score.
    //
    // Width check:
    //   dot_sum is 43-bit signed.
    //   dot_sum <<< 4 occupies bits [46:0] of a 64-bit signed word (safe).
    //   No overflow possible given the 18-bit input range.
    // -----------------------------------------------------------------------
    wire signed [42:0] dot_sum = {l2[0][41], l2[0]} + {l2[1][41], l2[1]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            score     <= 64'sd0;
        end else begin
            out_valid <= valid_l2;
            if (valid_l2)
                score <= ({{21{dot_sum[42]}}, dot_sum} <<< 4) + get_b(CLASSIFIER);
        end
    end

endmodule
