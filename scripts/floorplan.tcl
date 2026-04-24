# --- PATH SETUP ---
# Volare stores the PDK in a hidden folder in your home directory.
# We use the 'env(HOME)' variable to make this script work on any Linux machine.
set PDK_ROOT "$env(HOME)/.volare/sky130A/libs.ref/sky130_fd_sc_hd"

# --- 0. READ ARGS ---
# Check for the 'DENSITY' environment variable
if { [info exists env(DENSITY)] } {
    set density $env(DENSITY)
} else {
    # Default fallback
    set density 0.3
}
# --- 1. READ TECHNOLOGY LEF ---
# This defines the metal layers, vias, and the placement 'sites'.
read_lef $PDK_ROOT/techlef/sky130_fd_sc_hd__nom.tlef

# --- 2. READ CELL MASTER LEF ---
# This defines the physical dimensions (Width/Height) of every gate.
# This fixes the "master not found" warnings.
read_lef $PDK_ROOT/lef/sky130_fd_sc_hd.lef

# --- 3. READ NETLIST & LINK ---
read_verilog synth_fourBitRCA.v
link_design fourBitRCA

# --- 4. INITIALIZE FLOORPLAN ---
# -core_space 2 provides the boundary for your CP-SAT variables.
initialize_floorplan -utilization 30 -aspect_ratio 1.0 -site unithd -core_space 2

# Place IO pins (Skip for now, tracks to be added to techlef)
# place_pins -hor_layers {met1 met2 met3} -ver_layers {met2 met3 met4}

# --- 5. INITIAL PLACEMENT ---
# Spreads cells out so the parallel g-cells have a 'starting population'.
global_placement -density $density -skip_io

# --- 6. EXPORT FOR CP-SAT ---
write_def unplaced.def
exit
