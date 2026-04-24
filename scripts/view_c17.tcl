set PDK_HOME "/home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd"
read_lef $PDK_HOME/techlef/sky130_fd_sc_hd__nom.tlef
read_lef $PDK_HOME/lef/sky130_fd_sc_hd.lef
read_def ../designs/deffiles/c17_placed.def
gui::show
