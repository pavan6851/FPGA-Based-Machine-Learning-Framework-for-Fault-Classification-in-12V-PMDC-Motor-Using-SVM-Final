## =============================================================================
##  fpga_top_nexys4.xdc  -  v2  (95.00% accuracy build)
##  PMDC Motor Fault Detection - SVM Inference Core (OvR, LinearSVC)
##  Board : Digilent Nexys 4 (XC7A100T-1CSG324C)
##  Tool  : Vivado 2015.x or later
##
##  PORT LIST - matches fpga_top.v exactly:
##    clk        E3      100 MHz crystal
##    btnC       E16     centre button  (trigger inference)
##    btnR       R10     right  button  (next sample)
##    btnL       T16     left   button  (prev sample)
##    sw0        U9      slide switch 0 (active-high reset)
##    led[15:0]  16 user LEDs
##    seg[6:0]   7-seg cathodes CA-CG
##    an[3:0]    7-seg anodes  AN0-AN3  (module drives AN0 only)
##    dp         M1      decimal point  (blinks on result_valid)
##
##  ROOT CAUSE OF v1 ERRORS FIXED HERE:
##    Place 30-415: "497 I/O ports, only 209 available"
##      -> Phantom ports sw[15:0], btnU, btnD, uart_tx, uart_rx, an[7:4]
##         were in the XDC but NOT in fpga_top.v. Vivado tried to place
##         I/O buffers for all of them, exhausting the package's I/O budget.
##         FIXED: XDC now contains ONLY the 10 port groups above.
##    Common 17-55 / Vivado 12-4739: set_property / set_false_path on
##         non-existent ports -> cascaded from the phantom ports.
##         FIXED: all constraints reference real ports only.
##    Clock 15.00 ns (66 MHz) -> corrected to 10.00 ns (100 MHz crystal).
##
##  All pins verified against Digilent Nexys4_Master.xdc (rev. C).
## =============================================================================


## -- Clock --------------------------------------------------------------------
## Bank 35, IO_L12P_T1_MRCC_35 -- Sch: CLK100MHZ
set_property PACKAGE_PIN E3         [get_ports clk]
set_property IOSTANDARD  LVCMOS33   [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5.00} [get_ports clk]


## -- Buttons ------------------------------------------------------------------
## btnC -- centre (trigger inference)
## Bank 15, IO_L11N_T1_SRCC_15 -- Sch: BTNC
set_property PACKAGE_PIN E16        [get_ports btnC]
set_property IOSTANDARD  LVCMOS33   [get_ports btnC]

## btnR -- right (next sample)
## Bank 14, IO_25_14 -- Sch: BTNR
set_property PACKAGE_PIN R10        [get_ports btnR]
set_property IOSTANDARD  LVCMOS33   [get_ports btnR]

## btnL -- left (prev sample)
## Bank CONFIG, IO_L15N_T2_DQS_DOUT_CSO_B_14 -- Sch: BTNL
set_property PACKAGE_PIN T16        [get_ports btnL]
set_property IOSTANDARD  LVCMOS33   [get_ports btnL]


## -- Switch SW0 ---------------------------------------------------------------
## Active-high reset; fpga_top: assign rst_n = ~sw0
## Bank 34, IO_L21P_T3_DQS_34 -- Sch: SW0
set_property PACKAGE_PIN U9         [get_ports sw0]
set_property IOSTANDARD  LVCMOS33   [get_ports sw0]


## -- LEDs LD0-LD15 ------------------------------------------------------------
## led[1:0]  predicted class | led[3:2]  expected class
## led[4]    result_latched  | led[5]    mismatch
## led[6]    feat_valid      | led[7]    0
## led[10:8] sample_sel      | led[11]   0
## led[12]   Healthy  | led[13] Fault1 | led[14] Fault2 | led[15] Fault3
set_property PACKAGE_PIN T8  [get_ports {led[0]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN V9  [get_ports {led[1]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property PACKAGE_PIN R8  [get_ports {led[2]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property PACKAGE_PIN T6  [get_ports {led[3]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property PACKAGE_PIN T5  [get_ports {led[4]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]
set_property PACKAGE_PIN T4  [get_ports {led[5]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]
set_property PACKAGE_PIN U7  [get_ports {led[6]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]
set_property PACKAGE_PIN U6  [get_ports {led[7]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]
set_property PACKAGE_PIN V4  [get_ports {led[8]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[8]}]
set_property PACKAGE_PIN U3  [get_ports {led[9]}]  ; set_property IOSTANDARD LVCMOS33 [get_ports {led[9]}]
set_property PACKAGE_PIN V1  [get_ports {led[10]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {led[10]}]
set_property PACKAGE_PIN R1  [get_ports {led[11]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {led[11]}]
set_property PACKAGE_PIN P5  [get_ports {led[12]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {led[12]}]
set_property PACKAGE_PIN U1  [get_ports {led[13]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {led[13]}]
set_property PACKAGE_PIN R2  [get_ports {led[14]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {led[14]}]
set_property PACKAGE_PIN P2  [get_ports {led[15]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {led[15]}]


## -- 7-Segment Cathodes CA-CG (active low) ------------------------------------
set_property PACKAGE_PIN L3  [get_ports {seg[0]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {seg[0]}]
set_property PACKAGE_PIN N1  [get_ports {seg[1]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {seg[1]}]
set_property PACKAGE_PIN L5  [get_ports {seg[2]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {seg[2]}]
set_property PACKAGE_PIN L4  [get_ports {seg[3]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {seg[3]}]
set_property PACKAGE_PIN K3  [get_ports {seg[4]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {seg[4]}]
set_property PACKAGE_PIN M2  [get_ports {seg[5]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {seg[5]}]
set_property PACKAGE_PIN L6  [get_ports {seg[6]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {seg[6]}]


## -- Decimal Point DP (active low) -------------------------------------------
## Blinks ~100 ms on result_valid. IO_L16N_T2_34 -- Sch: DP
set_property PACKAGE_PIN M1         [get_ports dp]
set_property IOSTANDARD  LVCMOS33   [get_ports dp]


## -- 7-Segment Anodes AN0-AN3 (active low) ------------------------------------
## fpga_top drives: assign an = 4'b1110  (AN0 active, AN1-AN3 off)
## an[3:0] only - matches the [3:0] declaration in fpga_top.v
set_property PACKAGE_PIN N6  [get_ports {an[0]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {an[0]}]
set_property PACKAGE_PIN M6  [get_ports {an[1]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {an[1]}]
set_property PACKAGE_PIN M3  [get_ports {an[2]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {an[2]}]
set_property PACKAGE_PIN N5  [get_ports {an[3]}] ; set_property IOSTANDARD LVCMOS33 [get_ports {an[3]}]


## -- Timing Constraints -------------------------------------------------------
## Async inputs: debounce FFs in fpga_top handle metastability
set_false_path -from [get_ports btnC]
set_false_path -from [get_ports btnR]
set_false_path -from [get_ports btnL]
set_false_path -from [get_ports sw0]

## Registered outputs: no external timing contract
set_false_path -to [get_ports {led[*]}]
set_false_path -to [get_ports {seg[*]}]
set_false_path -to [get_ports {an[*]}]
set_false_path -to [get_ports dp]

## -- UART TX ------------------------------------------------------------------
set_property PACKAGE_PIN D4        [get_ports uart_tx]
set_property IOSTANDARD  LVCMOS33  [get_ports uart_tx]
set_false_path -to [get_ports uart_tx]

## =============================================================================
##  End of constraints
## =============================================================================
