#!/bin/bash
# ==============================================================================
# BTP Master Flow Orchestrator
# Usage: ./master_flow.sh <design_name>
# ==============================================================================

if [ -z "$1" ]; then
    echo "Error: Please provide a design name (e.g., c7552)."
    exit 1
fi

DESIGN=$1

# Resolve paths based on the script being run from ~/iitm/btp/scripts/
BASE_DIR=$(realpath ..)

# PDK Paths
TECH_LEF="/home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/techlef/sky130_fd_sc_hd__nom.tlef"
MACRO_LEF="/home/jdattatreya/.volare/sky130A/libs.ref/sky130_fd_sc_hd/lef/sky130_fd_sc_hd.lef"
LIB_FILE="/home/jdattatreya/iitm/ispd26/OpenROAD/src/odb/test/data/sky130hd/sky130_fd_sc_hd__tt_025C_1v80.lib"

# Project Directories
VERILOG_DIR="$BASE_DIR/designs/verilog"
SYNTH_DIR="$BASE_DIR/designs/synth"
SDC_DIR="$BASE_DIR/designs/sdc"
DEF_DIR="$BASE_DIR/designs/deffiles"
OUT_DEF_DIR="$BASE_DIR/outputDefs"
LOG_DIR="$BASE_DIR/logs/$DESIGN"
PY_CODE_DIR="$BASE_DIR/v2/code"

# Define the Virtual Environment Python Path
VENV_PYTHON="$BASE_DIR/.venv/bin/python3"

mkdir -p $SYNTH_DIR $SDC_DIR $DEF_DIR $OUT_DEF_DIR $LOG_DIR

echo "=================================================="
echo " Starting Physical Design Flow for: $DESIGN"
echo "=================================================="

# Safety Check: Ensure the virtual environment exists before starting a 5-minute flow
if [ ! -f "$VENV_PYTHON" ]; then
    echo "[ERROR] Virtual environment not found at $VENV_PYTHON"
    echo "Please ensure your .venv is set up in the root of the btp folder."
    exit 1
fi

# 1. SDC Generation
SDC_FILE="$SDC_DIR/${DESIGN}.sdc"
if [ ! -f "$SDC_FILE" ]; then
    echo "[1/6] Generating default SDC file..."
    cat <<EOF > "$SDC_FILE"
create_clock -name vclk -period 10.0
set_input_delay 2.0 -clock vclk [all_inputs]
set_output_delay 2.0 -clock vclk [all_outputs]
set_load 0.05 [all_outputs]
EOF
else
    echo "[1/6] SDC file already exists for $DESIGN. Skipping generation."
fi

# 2. Logic Synthesis
echo "[2/6] Running Yosys Synthesis..."
YOSYS_SCRIPT="$LOG_DIR/${DESIGN}_synth.ys"
cat <<EOF > $YOSYS_SCRIPT
read_verilog $VERILOG_DIR/${DESIGN}.v
hierarchy -check -top $DESIGN
synth -top $DESIGN
dfflibmap -liberty $LIB_FILE
abc -liberty $LIB_FILE
clean
write_verilog $SYNTH_DIR/synth_${DESIGN}.v
EOF
yosys -s $YOSYS_SCRIPT -l $LOG_DIR/yosys.log > /dev/null

# 3. Global Placement
echo "[3/6] Running OpenROAD Global Placement..."
GP_DEF="$DEF_DIR/${DESIGN}_global_placed.def"
GP_TCL="$LOG_DIR/${DESIGN}_gp.tcl"
cat <<EOF > $GP_TCL
read_lef $TECH_LEF
read_lef $MACRO_LEF
read_liberty $LIB_FILE
read_verilog $SYNTH_DIR/synth_${DESIGN}.v
link_design $DESIGN
read_sdc $SDC_FILE
initialize_floorplan -site unithd -die_area "0 0 100 100" -core_area "10 10 90 90"
tapcell -distance 14 -tapcell_master sky130_fd_sc_hd__tapvpwrvgnd_1
make_tracks
place_pins -hor_layers met3 -ver_layers met2
global_placement -density 0.7
write_def $GP_DEF
EOF
openroad -exit $GP_TCL > $LOG_DIR/openroad_gp.log

# 4. OpenDP Legalization
echo "[4/6] Running OpenROAD Detailed Placement (Baseline)..."
OPENDP_DEF="$OUT_DEF_DIR/${DESIGN}_opendp.def"
DP_TCL="$LOG_DIR/${DESIGN}_dp.tcl"
cat <<EOF > $DP_TCL
read_lef $TECH_LEF
read_lef $MACRO_LEF
read_def $GP_DEF
detailed_placement
write_def $OPENDP_DEF
EOF
openroad -exit $DP_TCL > $LOG_DIR/openroad_dp.log

# 5. CP-SAT Legalization
echo "[5/6] Running CP-SAT Detailed Placement Engine..."
CPSAT_DEF="$OUT_DEF_DIR/${DESIGN}_cpsat.def"
cd $PY_CODE_DIR
$VENV_PYTHON CPSATPlacement.py $GP_DEF $CPSAT_DEF > $LOG_DIR/cpsat_execution.log
cd $BASE_DIR/scripts

# 6. Benchmarking
echo "[6/6] Generating Benchmark Reports..."
REPORT_FILE="$LOG_DIR/${DESIGN}_benchmark_report.txt"

echo "==================================================" > $REPORT_FILE
echo " BTP Legalization Benchmark: $DESIGN" >> $REPORT_FILE
echo "==================================================" >> $REPORT_FILE

cd $PY_CODE_DIR
echo -e "\n>>> BASELINE: OpenROAD Detailed Placement (OpenDP)" >> $REPORT_FILE
$VENV_PYTHON benchmark_metrics.py $GP_DEF $OPENDP_DEF >> $REPORT_FILE

echo -e "\n>>> EXPERIMENTAL: CP-SAT Detailed Placement" >> $REPORT_FILE
$VENV_PYTHON benchmark_metrics.py $GP_DEF $CPSAT_DEF >> $REPORT_FILE
cd $BASE_DIR/scripts

echo "=================================================="
echo " Flow Complete! Report saved to:"
echo " $REPORT_FILE"
echo "=================================================="