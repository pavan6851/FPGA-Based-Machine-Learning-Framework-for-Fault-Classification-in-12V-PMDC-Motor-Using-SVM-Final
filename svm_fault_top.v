// =============================================================================
//  svm_fault_top.v  -  OvR (One-vs-Rest) top-level for LinearSVC
//  PMDC Motor Fault Detection - SVM Inference Core
//
//  v9 - Updated to match svm_dot_product.v v8 (6-stage pipeline)
//
//  CHANGE vs v8 (the ONLY change):
//    svm_dot_product v8 added Stage -1 (input register), so its internal
//    depth is now 6 cycles instead of 5.  valid_shift must be 1 bit wider
//    so valid_fire fires at cycle +6, not cycle +5.
//
//    valid_shift : [4:0] --> [5:0]
//    valid_fire  : valid_shift[4] --> valid_shift[5]
//
//  Pipeline latency: 7 clock cycles from feat_valid to result_valid
//    Stages -1..4 in svm_dot_product  (6 cycles)
//    +1 argmax register stage          (1 cycle)
//
//  Testbench must use PIPE_CYCLES = 7  (was 6 for v8).
//  fpga_top.v needs NO change (it waits on result_valid, not cycle count).
// =============================================================================
`timescale 1ns/1ps
`include "svm_params.vh"

module svm_fault_top (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        feat_valid,

    input  wire signed [31:0] feat_data_0,
    input  wire signed [31:0] feat_data_1,
    input  wire signed [31:0] feat_data_2,
    input  wire signed [31:0] feat_data_3,
    input  wire signed [31:0] feat_data_4,
    input  wire signed [31:0] feat_data_5,
    input  wire signed [31:0] feat_data_6,
    input  wire signed [31:0] feat_data_7,
    input  wire signed [31:0] feat_data_8,
    input  wire signed [31:0] feat_data_9,
    input  wire signed [31:0] feat_data_10,
    input  wire signed [31:0] feat_data_11,
    input  wire signed [31:0] feat_data_12,
    input  wire signed [31:0] feat_data_13,
    input  wire signed [31:0] feat_data_14,

    output reg         result_valid,
    output reg  [1:0]  result_class,
    output reg  [11:0] result_votes_dbg
);

    // -------------------------------------------------------------------------
    // 4 OvR dot-product units (CLASSIFIER 0..3)  [unchanged from v8]
    // -------------------------------------------------------------------------
    wire signed [63:0] score [0:3];
    wire               dp_valid [0:3];

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : DP
            svm_dot_product #(.CLASSIFIER(i)) u_dp (
                .clk      (clk),
                .rst_n    (rst_n),
                .in_valid (feat_valid),
                .x0 (feat_data_0),  .x1 (feat_data_1),  .x2 (feat_data_2),
                .x3 (feat_data_3),  .x4 (feat_data_4),  .x5 (feat_data_5),
                .x6 (feat_data_6),  .x7 (feat_data_7),  .x8 (feat_data_8),
                .x9 (feat_data_9),  .x10(feat_data_10), .x11(feat_data_11),
                .x12(feat_data_12), .x13(feat_data_13), .x14(feat_data_14),
                .score    (score[i]),
                .out_valid(dp_valid[i])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // CHANGE: 6-bit shift register (was [4:0] in v8)
    //
    // svm_dot_product v8 has 6 pipeline stages (Stage -1 through Stage 4).
    // valid_fire must fire at cycle +6 so the argmax block latches at cycle +7,
    // one cycle after Stage 4 settles score[] at the cycle-6 rising edge.
    //
    //   v8: [4:0], valid_fire=valid_shift[4]  -> fires at cycle +5  (for 5-stage dp)
    //   v9: [5:0], valid_fire=valid_shift[5]  -> fires at cycle +6  (for 6-stage dp)
    // -------------------------------------------------------------------------
    reg [5:0] valid_shift;    // was [4:0] in v8

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid_shift <= 6'b0;
        else
            valid_shift <= {valid_shift[4:0], feat_valid};
    end

    wire valid_fire = valid_shift[5];    // was valid_shift[4] in v8

    // -------------------------------------------------------------------------
    // Argmax registered stage  [logic unchanged from v8]
    //
    // valid_fire fires at cycle +6.
    // This always @(posedge clk) block adds 1 cycle: result_valid asserts at +7.
    // At cycle +7's rising edge, score[] has been stable since cycle +6,
    // so setup time is cleanly met.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result_valid     <= 1'b0;
            result_class     <= 2'd0;
            result_votes_dbg <= 12'd0;
        end else begin
            result_valid <= 1'b0;

            if (valid_fire) begin
                result_valid <= 1'b1;

                // Argmax with tie-breaking: lower index wins
                if      (score[0] >= score[1] && score[0] >= score[2] && score[0] >= score[3])
                    result_class <= 2'd0;
                else if (score[1] >= score[2] && score[1] >= score[3])
                    result_class <= 2'd1;
                else if (score[2] >= score[3])
                    result_class <= 2'd2;
                else
                    result_class <= 2'd3;

                result_votes_dbg <= {
                        8'd0,
                        !score[3][63],
                        !score[2][63],
                        !score[1][63],
                        !score[0][63]
                    };
            end
        end
    end

endmodule
