#!/usr/bin/env python3
import sys
import time
import os
import multiprocessing
import LEFDEFParser
from ortools.sat.python import cp_model

# --- CLI Argument Handling ---
# This allows master_flow.sh to call: python3 CPSATPlacement.py <input.def> <output.def>
if len(sys.argv) == 3:
    deffile = sys.argv[1]
    outfile = sys.argv[2]
else:
    print("Usage: python3 CPSATPlacement.py <input_def> <output_def>")
    print("Falling back to default paths for manual testing...")
    deffile = "../../designs/deffiles/c7552_global_placed.def"
    outfile = "../../outputDefs/c7552_placed.def"

leffile = "../../designs/leffiles/sky130.lef"

# --- Parsing ---
print(f"Reading LEF: {leffile}")
l_parser = LEFDEFParser.LEFReader()
l_parser.readLEF(leffile)
dimLookup = {m.name(): (m.xdim(), m.ydim()) for m in l_parser.macros()}
site_height = 2720 # Sky130 Row Height

print(f"Reading DEF: {deffile}")
d_parser = LEFDEFParser.DEFReader()
d_parser.readDEF(deffile)

# Fast lookups for nets and pins
comp_coords = {c.name(): (c.location().x, c.location().y) for c in d_parser.components()}
pin_coords = {p.name(): (p.origin().x, p.origin().y) for p in d_parser.pins()}

# Chip boundaries
chip_bbox = d_parser.bbox()
x_max, y_max = chip_bbox.ur.x, chip_bbox.ur.y

# --- Core Boundary Extraction ---
left_edge, right_edge = [], []
y_coords = []
for comp in d_parser.components():
    name = comp.name()
    if "Left" in name:
        w, _ = dimLookup[comp.macro()]
        left_edge.append(comp.location().x + w)
    elif "Right" in name:
        right_edge.append(comp.location().x)
    if "PHY_EDGE" in name or "TAP" in name:
        y_coords.append(comp.location().y)

core_x_min = max(left_edge) if left_edge else 0
core_x_max = min(right_edge) if right_edge else x_max
core_y_min = min(y_coords) if y_coords else 0
core_y_max = max(y_coords) if y_coords else y_max

# --- Solver Worker Function ---
def solve_local_gcell(bin_id, gcell_bbox, local_gates, local_obstacles, local_nets, core_y_min, site_height):
    model = cp_model.CpModel()
    nodes = {}
    x_intervals, y_intervals = [], []
    displacement_vars = []

    # 1. Fixed Obstacles
    for obs in local_obstacles:
        if not (obs['x'] > gcell_bbox[2] or obs['x'] + obs['w'] < gcell_bbox[0] or \
                obs['y'] > gcell_bbox[3] or obs['y'] + obs['h'] < gcell_bbox[1]):
            x_intervals.append(model.NewFixedSizeIntervalVar(obs['x'], obs['w'], 'obs_x'))
            y_intervals.append(model.NewFixedSizeIntervalVar(obs['y'], obs['h'], 'obs_y'))

    # 2. Movable Gates + Bounding Box + Displacement Penalty
    for c_name, data in local_gates.items():
        w, h = data['w'], data['h']
        orig_x, orig_y = data['init_x'], data['init_y']

        # CLAMP: Keep gate strictly inside G-Cell boundary
        x_var = model.NewIntVar(gcell_bbox[0], gcell_bbox[2] - w, f"x_{c_name}")
        
        # Snap to Rows
        min_row = max(0, (gcell_bbox[1] - core_y_min) // site_height)
        max_row = (gcell_bbox[3] - core_y_min) // site_height
        row_idx = model.NewIntVar(min_row, max_row, f"row_{c_name}")
        y_var = model.NewIntVar(gcell_bbox[1], gcell_bbox[3], f"y_{c_name}")
        model.Add(y_var == core_y_min + (row_idx * site_height))

        x_intervals.append(model.NewIntervalVar(x_var, w, x_var + w, f'xi_{c_name}'))
        y_intervals.append(model.NewIntervalVar(y_var, h, y_var + h, f'yi_{c_name}'))
        nodes[c_name] = {'x': x_var, 'y': y_var}

        # DISPLACEMENT PENALTY: dx = |x - orig_x|
        dx = model.NewIntVar(0, 1000000, f'dx_{c_name}')
        dy = model.NewIntVar(0, 1000000, f'dy_{c_name}')
        model.AddAbsEquality(dx, x_var - orig_x)
        model.AddAbsEquality(dy, y_var - orig_y)
        displacement_vars.extend([dx, dy])

    model.AddNoOverlap2D(x_intervals, y_intervals)

    # 3. HPWL with External Anchors
    net_costs = []
    for net_name, net_data in local_nets.items():
        pins_x = [nodes[p]['x'] for p in net_data['internal_pins']]
        pins_y = [nodes[p]['y'] for p in net_data['internal_pins']]
        ext_x = [c[0] for c in net_data['external_coords']]
        ext_y = [c[1] for c in net_data['external_coords']]

        if len(pins_x) + len(ext_x) < 2: continue

        min_x, max_x = model.NewIntVar(0, 2000000, 'min_x'), model.NewIntVar(0, 2000000, 'max_x')
        model.AddMinEquality(min_x, pins_x + ext_x)
        model.AddMaxEquality(max_x, pins_x + ext_x)
        
        min_y, max_y = model.NewIntVar(0, 2000000, 'min_y'), model.NewIntVar(0, 2000000, 'max_y')
        model.AddMinEquality(min_y, pins_y + ext_y)
        model.AddMaxEquality(max_y, pins_y + ext_y)
        
        net_costs.append(max_x - min_x + max_y - min_y)

    # 4. Multi-Objective Minimize: HPWL + beta * Displacement
    beta = 4 
    model.Minimize(sum(net_costs) + (beta * sum(displacement_vars)))

    # 5. Solve
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 15.0
    status = solver.Solve(model)

    res = {}
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        for c_name in local_gates.keys():
            res[c_name] = (solver.Value(nodes[c_name]['x']), solver.Value(nodes[c_name]['y']))
    return bin_id, status, res

# --- Partitioning ---
GCELL_SIZE = 30000
bins = {}
fixed_obs = []
gate_to_bin = {}

print("Partitioning G-Cells...")
for comp in d_parser.components():
    c_name, m_name = comp.name(), comp.macro()
    w, h = dimLookup[m_name]
    lx, ly = comp.location().x, comp.location().y

    if "TAP" in c_name or "PHY_EDGE" in c_name:
        fixed_obs.append({'x': lx, 'y': ly, 'w': w, 'h': h})
        continue

    col = (lx - core_x_min) // GCELL_SIZE
    row = (ly - core_y_min) // GCELL_SIZE
    bid = (col, row)
    gate_to_bin[c_name] = bid

    if bid not in bins:
        bx = core_x_min + (col * GCELL_SIZE)
        by = core_y_min + (row * GCELL_SIZE)
        bins[bid] = {
            'bbox': (bx, by, min(bx + GCELL_SIZE, core_x_max), min(by + GCELL_SIZE, core_y_max)),
            'gates': {}, 'nets': {}
        }
    bins[bid]['gates'][c_name] = {'w': w, 'h': h, 'init_x': lx, 'init_y': ly}

# Build Local Net Map
for net in d_parser.nets():
    touched = {gate_to_bin[p[0]] for p in net.pins() if p[0] != 'PIN' and p[0] in gate_to_bin}
    for b in touched:
        int_p, ext_c = [], []
        for p in net.pins():
            if p[0] == 'PIN': ext_c.append(pin_coords[p[1]])
            elif p[0] in gate_to_bin and gate_to_bin[p[0]] == b: int_p.append(p[0])
            else: ext_c.append(comp_coords.get(p[0], (0,0)))
        if int_p and (len(int_p) + len(ext_c) > 1):
            bins[b]['nets'][net.name()] = {'internal_pins': int_p, 'external_coords': ext_c}

# --- Parallel Execution ---
args = [(bid, d['bbox'], d['gates'], fixed_obs, d['nets'], core_y_min, site_height) for bid, d in bins.items()]
print(f"Solving {len(bins)} bins on 8 cores...")
start = time.time()
with multiprocessing.Pool(8) as pool:
    results = pool.starmap(solve_local_gcell, args)

# --- Stitching ---
comp_map = {c.name(): c for c in d_parser.components()}
for bid, status, res in results:
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        for name, (nx, ny) in res.items():
            comp_map[name].setLocation(nx, ny)
    else:
        print(f"Warning: Bin {bid} failed. Keeping GP coords.")

d_parser.writeDEF(outfile)
print(f"Success! Legalized DEF saved to {outfile} in {time.time()-start:.2f}s")