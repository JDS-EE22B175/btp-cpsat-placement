#!/usr/bin/env python
# coding: utf-8

# # CP-SAT Parallel Detailed Placer (G-Cell Partitioned)
# This notebook implements a scalable, multi-threaded detailed placement optimizer. It divides the core area into smaller G-Cell bins and dispatches them to parallel CPU workers to optimize HPWL and legality at breakneck speeds.

# In[1]:


import sys
import time
import os
import subprocess
import multiprocessing
import LEFDEFParser
from ortools.sat.python import cp_model


# In[2]:


# --- Initialization & Parsing ---
deffile = "../designs/deffiles/project/def/c7552.def"
leffile = "../designs/leffiles/sky130.lef"
outfile = "../outputDefs/c7552_placed.def"

print("Parsing LEF and DEF...")
d_parser = LEFDEFParser.DEFReader()
d_parser.readDEF(deffile)
l_parser = LEFDEFParser.LEFReader()
l_parser.readLEF(leffile)

dimLookup = {m.name(): (m.xdim(), m.ydim()) for m in l_parser.macros()}
site_height = 2720 # Sky130 Row Height

# Build coordinate lookups for fast anchoring
comp_coords = {c.name(): (c.location().x, c.location().y) for c in d_parser.components()}
pin_coords = {p.name(): (p.origin().x, p.origin().y) for p in d_parser.pins()}

chip_bbox = d_parser.bbox()
x_max = chip_bbox.ur.x
y_max = chip_bbox.ur.y
print(f"Parsed design. Chip BBox: (0,0) to ({x_max}, {y_max})")


# In[3]:


# --- Dynamic Core Boundary Extraction ---
left_edge_coords = []
right_edge_coords = []
y_coords = []

for comp in d_parser.components():
    name = comp.name()
    if "Left" in name:
        w, _ = dimLookup[comp.macro()]
        left_edge_coords.append(comp.location().x + w)
    elif "Right" in name:
        right_edge_coords.append(comp.location().x)

    if "PHY_EDGE" in name or "TAP" in name:
        y_coords.append(comp.location().y)

core_x_min = max(left_edge_coords) if left_edge_coords else 0
core_x_max = min(right_edge_coords) if right_edge_coords else x_max
core_y_min = min(y_coords) if y_coords else 0
core_y_max = max(y_coords) if y_coords else y_max

print(f"Extracted Core Zone: X({core_x_min} to {core_x_max}), Y({core_y_min} to {core_y_max})")


# In[4]:


# --- The G-Cell Worker Function (Runs on isolated threads) ---
def solve_local_gcell(bin_id, gcell_bbox, local_gates, local_obstacles, local_nets, core_y_min, site_height):
    model = cp_model.CpModel()
    nodes = {}
    x_intervals, y_intervals = [], []

    # 1. Add Fixed Obstacles (TAP/EDGE)
    for obs in local_obstacles:
        # Only add obstacles that intersect this bin
        if not (obs['x'] > gcell_bbox[2] or obs['x'] + obs['w'] < gcell_bbox[0] or \
                obs['y'] > gcell_bbox[3] or obs['y'] + obs['h'] < gcell_bbox[1]):
            x_intervals.append(model.NewFixedSizeIntervalVar(obs['x'], obs['w'], 'obs_x'))
            y_intervals.append(model.NewFixedSizeIntervalVar(obs['y'], obs['h'], 'obs_y'))

    # 2. Add Movable Gates
    for c_name, data in local_gates.items():
        w, h = data['w'], data['h']
        x_var = model.new_int_var(gcell_bbox[0], gcell_bbox[2] - w, f"x_{c_name}")

        # Y alignment relative to the GLOBAL core rows, not just the local bin
        min_global_row = max(0, (gcell_bbox[1] - core_y_min) // site_height)
        max_global_row = (gcell_bbox[3] - core_y_min) // site_height
        row_idx = model.new_int_var(min_global_row, max_global_row, f"row_{c_name}")
        y_var = model.new_int_var(gcell_bbox[1], gcell_bbox[3], f"y_{c_name}")
        model.add(y_var == core_y_min + (row_idx * site_height))

        x_intervals.append(model.NewIntervalVar(x_var, w, x_var + w, f'x_int_{c_name}'))
        y_intervals.append(model.NewIntervalVar(y_var, h, y_var + h, f'y_int_{c_name}'))
        nodes[c_name] = {'x': x_var, 'y': y_var}

    model.add_no_overlap_2d(x_intervals, y_intervals)

    # 3. HPWL with External Anchors
    net_costs = []
    for net_name, net_data in local_nets.items():
        pin_x_vars = [nodes[p]['x'] for p in net_data['internal_pins']]
        pin_y_vars = [nodes[p]['y'] for p in net_data['internal_pins']]
        ext_x = [coord[0] for coord in net_data['external_coords']]
        ext_y = [coord[1] for coord in net_data['external_coords']]

        if len(pin_x_vars) + len(ext_x) < 2: continue

        min_x = model.new_int_var(0, 999999, f"min_x_{net_name}")
        max_x = model.new_int_var(0, 999999, f"max_x_{net_name}")
        model.add_min_equality(min_x, pin_x_vars + ext_x)
        model.add_max_equality(max_x, pin_x_vars + ext_x)
        net_costs.append(max_x - min_x)

        min_y = model.new_int_var(0, 999999, f"min_y_{net_name}")
        max_y = model.new_int_var(0, 999999, f"max_y_{net_name}")
        model.add_min_equality(min_y, pin_y_vars + ext_y)
        model.add_max_equality(max_y, pin_y_vars + ext_y)
        net_costs.append(max_y - min_y)

    model.minimize(sum(net_costs))

    # 4. Solve
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 20.0 # Time limit per G-Cell
    status = solver.solve(model)

    results = {}
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        for c_name in local_gates.keys():
            results[c_name] = (solver.value(nodes[c_name]['x']), solver.value(nodes[c_name]['y']))

    return bin_id, status, results


# In[5]:


# --- G-Cell Partitioning & Data Extraction ---
GCELL_W = 30000  # Size of the window in DBU
GCELL_H = 30000

bins = {}
fixed_obstacles = []
gate_to_bin = {}

print("Partitioning chip and sorting components...")
# 1. Sort Components into Bins and separate Obstacles
for comp in d_parser.components():
    c_name = comp.name()
    w, h = dimLookup[comp.macro()]
    loc_x, loc_y = comp.location().x, comp.location().y

    if c_name.startswith("TAP_") or c_name.startswith("PHY_EDGE_"):
        fixed_obstacles.append({'x': loc_x, 'y': loc_y, 'w': w, 'h': h})
        continue

    col = (loc_x - core_x_min) // GCELL_W
    row = (loc_y - core_y_min) // GCELL_H
    bin_id = (col, row)
    gate_to_bin[c_name] = bin_id

    if bin_id not in bins:
        bin_min_x = core_x_min + (col * GCELL_W)
        bin_min_y = core_y_min + (row * GCELL_H)
        bins[bin_id] = {
            'bbox': (bin_min_x, bin_min_y, min(bin_min_x + GCELL_W, core_x_max), min(bin_min_y + GCELL_H, core_y_max)),
            'gates': {},
            'nets': {}
        }

    bins[bin_id]['gates'][c_name] = {'w': w, 'h': h, 'init_x': loc_x, 'init_y': loc_y}

# 2. Build Net Map with External Anchors
for net in d_parser.nets():
    net_pins = net.pins()
    bins_touched = set()

    for p in net_pins:
        if p[0] != 'PIN' and p[0] in gate_to_bin:
            bins_touched.add(gate_to_bin[p[0]])

    for b_id in bins_touched:
        internal_pins = []
        external_coords = []

        for p in net_pins:
            if p[0] == 'PIN':
                external_coords.append(pin_coords[p[1]])
            elif p[0] in gate_to_bin and gate_to_bin[p[0]] == b_id:
                internal_pins.append(p[0])
            else:
                external_coords.append(comp_coords[p[0]])

        if len(internal_pins) > 0 and (len(internal_pins) + len(external_coords) > 1):
            bins[b_id]['nets'][net.name()] = {'internal_pins': internal_pins, 'external_coords': external_coords}

print(f"Partitioned into {len(bins)} active G-Cells.")


# In[6]:


# --- Multiprocessing Orchestrator ---
pool_args = []
for bin_id, data in bins.items():
    pool_args.append((bin_id, data['bbox'], data['gates'], fixed_obstacles, data['nets'], core_y_min, site_height))

print("Dispatching G-Cells to 8 CPU cores...")
start_time = time.time()

# Execute parallel solvers
with multiprocessing.Pool(processes=8) as pool:
    completed_bins = pool.starmap(solve_local_gcell, pool_args)

print(f"All {len(completed_bins)} G-Cells processed in {time.time() - start_time:.2f} seconds.")


# In[ ]:


# ---- Build a fast name→component lookup ONCE (O(n)) ----
comp_by_name = {comp.name(): comp for comp in d_parser.components()}

successful_moves = 0
for bin_id, status, results in completed_bins:
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        for c_name, (new_x, new_y) in results.items():
            comp = comp_by_name.get(c_name)
            if comp:
                comp.setLocation(new_x, new_y)
                successful_moves += 1
    else:
        print(f"Warning: Bin {bin_id} was INFEASIBLE. Gates left at global placement.")

d_parser.writeDEF(outfile)
print(f"Stitching complete. Placed {successful_moves} logic gates.")

# ---- Fix the OpenROAD path ----
abs_outfile = os.path.abspath(outfile)
os.makedirs(os.path.dirname(abs_outfile), exist_ok=True)
env = os.environ.copy()
env["BTP_DEF_FILE"] = abs_outfile
subprocess.Popen(["openroad", "-gui", "../scripts/view_defs.tcl"], env=env)
# Use Popen instead of run so it doesn't block the notebook kernel


# In[ ]:


def calculate_displacement(gp_def_path, dp_def_path):
    print(f"Parsing Global Placement: {gp_def_path}")
    gp_parser = LEFDEFParser.DEFReader()
    gp_parser.readDEF(gp_def_path)

    # Store initial coordinates of movable gates
    gp_coords = {}
    for comp in gp_parser.components():
        c_name = comp.name()
        # Ignore fixed infrastructure
        if not (c_name.startswith("TAP_") or c_name.startswith("PHY_EDGE_")):
            gp_coords[c_name] = (comp.location().x, comp.location().y)

    print(f"Parsing Detailed Placement: {dp_def_path}")
    dp_parser = LEFDEFParser.DEFReader()
    dp_parser.readDEF(dp_def_path)

    total_displacement = 0
    max_displacement = 0
    moved_cells = 0
    zero_movement_cells = 0

    for comp in dp_parser.components():
        c_name = comp.name()
        if c_name in gp_coords:
            old_x, old_y = gp_coords[c_name]
            new_x, new_y = comp.location().x, comp.location().y

            # Manhattan Distance calculation
            displacement = abs(new_x - old_x) + abs(new_y - old_y)

            total_displacement += displacement
            if displacement > max_displacement:
                max_displacement = displacement

            if displacement > 0:
                moved_cells += 1
            else:
                zero_movement_cells += 1

    total_evaluated = moved_cells + zero_movement_cells
    avg_displacement = total_displacement / total_evaluated if total_evaluated > 0 else 0

    print("\n" + "="*30)
    print("      DISPLACEMENT METRICS      ")
    print("="*30)
    print(f"Total Gates Evaluated : {total_evaluated}")
    print(f"Gates Moved           : {moved_cells}")
    print(f"Gates Unchanged       : {zero_movement_cells}")
    print(f"Total Displacement    : {total_displacement} DBU")
    print(f"Max Displacement      : {max_displacement} DBU")
    print(f"Average Displacement  : {avg_displacement:.2f} DBU")
    print("="*30)

# --- Execution ---
original_def = "../deffiles/project/def/c7552.def"
legalized_def = "../outputDefs/c7552_placed.def"

calculate_displacement(original_def, legalized_def)

