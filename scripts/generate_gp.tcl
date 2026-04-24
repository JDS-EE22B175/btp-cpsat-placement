# 1. Read LEF and Liberty Files
read_lef /home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/techlef/sky130_fd_sc_hd__nom.tlef
read_lef /home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/lef/sky130_fd_sc_hd.lef
read_liberty /home/jdattatreya/iitm/ispd26/OpenROAD/src/odb/test/data/sky130hd/sky130_fd_sc_hd__tt_025C_1v80.lib

# 2. Load the synthesized netlist and SDC constraints
read_verilog ../designs/synth/synth_c7552.v
link_design c7552
read_sdc ../designs/sdc/c7552.sdc

# 3. Initialize the floorplan (Define die and core area)
initialize_floorplan -site unithd -die_area "0 0 100 100" -core_area "10 10 90 90"

# 4. Insert Tapcells (required for physical legality)
tapcell -distance 14 -tapcell_master sky130_fd_sc_hd__tapvpwrvgnd_1

# 5. Build the routing grid for the pins to snap to
make_tracks

# 6. Place I/O pins on the die boundary
# Assigning horizontal pins to metal 3 and vertical pins to metal 2
place_pins -hor_layers met3 -ver_layers met2

# 7. Run Global Placement (This will create the overlaps!)
global_placement -density 0.7

# 8. Save the raw, overlapping layout to a DEF file
write_def ../designs/deffiles/c7552_global_placed.def