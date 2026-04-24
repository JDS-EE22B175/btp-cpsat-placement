# c7552 is purely combinational, so we create a "virtual" clock 
# to give OpenROAD a timing reference point.
create_clock -name vclk -period 10.0

# Tell the tool to assume the inputs arrive 2ns after the virtual clock ticks
set_input_delay 2.0 -clock vclk [all_inputs]

# Tell the tool the outputs need to be ready 2ns before the next clock tick
set_output_delay 2.0 -clock vclk [all_outputs]

# Set a tiny dummy load (capacitance) on the output pins so the tool 
# doesn't think they are infinitely fast.
set_load 0.05 [all_outputs]