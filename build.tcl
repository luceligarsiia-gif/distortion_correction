cd [file dirname [info script]]
source configure_project.tcl
compile -top_module HDMI_IN_DDR3_HDMI_OUT_top -force_to_run
synthesize -ads -selected_syn_tool_opt 2 -force_to_run
dev_map -force_to_run
pnr -fix_hold_violation -fix_hold_violation_iterations 500 -fix_hold_violation_in_route TRUE -fix_hold_violation_threshold_for_clock_path_cross_srb 200 -gplace_seed 41 -groute_seed 0 -gplace_times 1 -seed_step 7 -parallel 1 -saved_outcome 1 -max_ctrl_chain_length 8 -optimize_multi_corner_timing -optimize_gp_recovery -optimize_dp_recovery -optimize_rp_recovery -optimize_routing_recovery TRUE -slack_prior_in_global_router TRUE -enable_route_constraint_iterate -route_constraint_iterate_times 6 -route_constraint_max_timing_path 300 -force_to_run
report_timing -force_to_run
set timing_file [open "report_timing/HDMI_IN_DDR3_HDMI_OUT_top.rtr" r]
set timing_text [read $timing_file]
close $timing_file
if {![string match {*Design Summary : All Constraints Met.*} $timing_text]} {
    error "Timing constraints not met; bitstream generation stopped."
}
gen_bit_stream -force_to_run
puts {PASS: resource-optimized video build finished}
exit
