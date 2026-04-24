# run_opendp.tcl
read_lef /home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/techlef/sky130_fd_sc_hd__nom.tlef
read_lef /home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/lef/sky130_fd_sc_hd.lef

# Load the RAW global placement
read_def ../designs/deffiles/c7552_global_placed.def

# Run OpenROAD's native legalizer
detailed_placement

# Save the OpenDP output
write_def ../outputDefs/c7552_opendp.def