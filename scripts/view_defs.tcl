# view_defs.tcl

# Fetch the DEF file path from the Python environment variable
if {![info exists ::env(BTP_DEF_FILE)]} {
    puts "Error: BTP_DEF_FILE environment variable not set. Please run via the Python script."
    exit 1
}
set def_file $::env(BTP_DEF_FILE)

set PDK_HOME "/home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd"

# Load Technology and Macro LEFs
read_lef $PDK_HOME/techlef/sky130_fd_sc_hd__nom.tlef
read_lef $PDK_HOME/lef/sky130_fd_sc_hd.lef

# Load the Placed DEF
read_def $def_file