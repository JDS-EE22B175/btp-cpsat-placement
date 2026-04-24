# eval_metrics.tcl
read_lef /home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/techlef/sky130_fd_sc_hd__nom.tlef
read_lef /home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/lef/sky130_fd_sc_hd.lef
read_def $::env(BTP_DEF_FILE)

# Get Design Area / Utilization
report_design_area

# Get Wirelength (HPWL)
# OpenROAD natively estimates wirelength based on the bounding box of the nets
report_wire_length