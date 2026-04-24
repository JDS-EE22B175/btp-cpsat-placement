import sys
import time
import LEFDEFParser
from ortools.sat.python import cp_model

# --- Progress Callback Class ---
class PlacementCallback(cp_model.CpSolverSolutionCallback):
    def __init__(self):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self._solution_count = 0
        self._start_time = time.time()

    def on_solution_callback(self):
        self._solution_count += 1
        print(f"[{time.time() - self._start_time:.2f}s] Solution {self._solution_count}: "
              f"Objective = {self.ObjectiveValue()}")

# --- Initialization ---
print(f"Python version: {sys.version}")
deffile = "../designs/deffiles/c17.def"
leffile = "../designs/leffiles/sky130.lef"
outfile = "../designs/deffiles/c17_placed_fixed.def"

d_parser = LEFDEFParser.DEFReader()
d_parser.readDEF(deffile)
l_parser = LEFDEFParser.LEFReader()
l_parser.readLEF(leffile)

chip_bbox = d_parser.bbox()
x_max = chip_bbox.ur.x
y_max = chip_bbox.ur.y
site_height = 2720  # Sky130 row height in DBU

width  = chip_bbox.ur.x - chip_bbox.ll.x
height = chip_bbox.ur.y - chip_bbox.ll.y
print(f"Chip dimensions: {width} x {height} DBU")

# --- Dynamic Core Boundary Extraction ---
left_edge_coords  = []
right_edge_coords = []
y_coords          = []

dimLookup = {m.name(): (m.xdim(), m.ydim()) for m in l_parser.macros()}

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
core_y_min = min(y_coords)         if y_coords          else 0
# +site_height because y_coords holds row origins, not row tops
core_y_max = max(y_coords) + site_height if y_coords   else y_max

print(f"Core placement zone: X({core_x_min} to {core_x_max}), Y({core_y_min} to {core_y_max})")

# --- CP-SAT Model ---
model = cp_model.CpModel()

nodes      = {}       # movable gates only
x_intervals = []      # all intervals (fixed + movable) for NoOverlap2D
y_intervals = []

for comp in d_parser.components():
    c_name = comp.name()
    w, h   = dimLookup[comp.macro()]
    loc    = comp.location()

    # ── Fixed infrastructure: TAP cells and PHY_EDGE cells ──────────────────
    # These are added as FixedSizeIntervalVars so the solver treats them as
    # hard obstacles. Without this, logic gates can legally overlap them.
    if c_name.startswith("TAP_") or c_name.startswith("PHY_EDGE_"):
        x_intervals.append(
            model.new_fixed_size_interval_var(loc.x, w, f'fx_{c_name}')
        )
        y_intervals.append(
            model.new_fixed_size_interval_var(loc.y, h, f'fy_{c_name}')
        )
        continue

    # ── Movable logic gates ──────────────────────────────────────────────────
    x_var   = model.new_int_var(core_x_min, core_x_max - w, f'x_{c_name}')
    num_rows = (core_y_max - core_y_min) // site_height
    row_idx  = model.new_int_var(0, num_rows - 1, f'row_{c_name}')
    y_var    = model.new_int_var(core_y_min, core_y_max - h, f'y_{c_name}')
    model.add(y_var == core_y_min + row_idx * site_height)

    x_int = model.new_interval_var(x_var, w, x_var + w, f'xi_{c_name}')
    y_int = model.new_interval_var(y_var, h, y_var + h, f'yi_{c_name}')
    x_intervals.append(x_int)
    y_intervals.append(y_int)

    nodes[c_name] = {'x': x_var, 'y': y_var, 'w': w, 'h': h}

# --- Legality: no overlaps among movable gates AND fixed obstacles -----------
model.add_no_overlap_2d(x_intervals, y_intervals)

# --- Objective 1: Minimize HPWL ---------------------------------------------
net_costs = []
for net in d_parser.nets():
    pin_x_vars = []
    pin_y_vars = []
    for pin in net.pins():
        comp_name = pin[0]
        if comp_name != 'PIN' and comp_name in nodes:
            pin_x_vars.append(nodes[comp_name]['x'])
            pin_y_vars.append(nodes[comp_name]['y'])
    if len(pin_x_vars) < 2:
        continue

    min_x = model.new_int_var(core_x_min, core_x_max, f'min_x_{net.name()}')
    max_x = model.new_int_var(core_x_min, core_x_max, f'max_x_{net.name()}')
    min_y = model.new_int_var(core_y_min, core_y_max, f'min_y_{net.name()}')
    max_y = model.new_int_var(core_y_min, core_y_max, f'max_y_{net.name()}')

    model.add_min_equality(min_x, pin_x_vars)
    model.add_max_equality(max_x, pin_x_vars)
    model.add_min_equality(min_y, pin_y_vars)
    model.add_max_equality(max_y, pin_y_vars)

    net_costs.append((max_x - min_x) + (max_y - min_y))

# --- Objective 2: Minimize total occupied span (compact clustering) ----------
g_min_x = model.new_int_var(core_x_min, core_x_max, 'g_min_x')
g_max_x = model.new_int_var(core_x_min, core_x_max, 'g_max_x')
g_min_y = model.new_int_var(core_y_min, core_y_max, 'g_min_y')
g_max_y = model.new_int_var(core_y_min, core_y_max, 'g_max_y')

model.add_min_equality(g_min_x, [n['x']           for n in nodes.values()])
model.add_max_equality(g_max_x, [n['x'] + n['w']  for n in nodes.values()])
model.add_min_equality(g_min_y, [n['y']           for n in nodes.values()])
model.add_max_equality(g_max_y, [n['y'] + n['h']  for n in nodes.values()])

occupied_span = (g_max_x - g_min_x) + (g_max_y - g_min_y)

# Weights: increase the span multiplier for tighter area packing
model.minimize(sum(net_costs) + occupied_span)

# --- HPWL snapshot before solving (uses original GP locations) ---------------
hpwl_before = 0
for net in d_parser.nets():
    xs, ys = [], []
    for pin in net.pins():
        comp_name = pin[0]
        if comp_name != 'PIN' and comp_name in nodes:
            loc = next(
                c.location() for c in d_parser.components()
                if c.name() == comp_name
            )
            xs.append(loc.x)
            ys.append(loc.y)
    if len(xs) >= 2:
        hpwl_before += (max(xs) - min(xs)) + (max(ys) - min(ys))
print(f"HPWL before solving: {hpwl_before}")

# --- Solve ------------------------------------------------------------------
solver = cp_model.CpSolver()
solver.parameters.num_search_workers = 8
solver.parameters.log_search_progress = True

print("Solving...")
status = solver.solve(model, PlacementCallback())
print(f"Solver status: {solver.status_name(status)}")

# --- Write results -----------------------------------------------------------
if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
    hpwl_after = 0

    for comp in d_parser.components():
        c_name    = comp.name()
        node_data = nodes.get(c_name)
        if node_data:
            new_x = solver.value(node_data['x'])
            new_y = solver.value(node_data['y'])
            comp.setLocation(new_x, new_y)

    # Compute final HPWL from placed coordinates
    comp_locs = {c.name(): (c.location().x, c.location().y)
                 for c in d_parser.components()}
    for net in d_parser.nets():
        xs, ys = [], []
        for pin in net.pins():
            comp_name = pin[0]
            if comp_name != 'PIN' and comp_name in comp_locs:
                xs.append(comp_locs[comp_name][0])
                ys.append(comp_locs[comp_name][1])
        if len(xs) >= 2:
            hpwl_after += (max(xs) - min(xs)) + (max(ys) - min(ys))

    d_parser.writeDEF(outfile)
    print(f"HPWL before: {hpwl_before}")
    print(f"HPWL after:  {hpwl_after}")
    improvement = (hpwl_before - hpwl_after) / hpwl_before * 100 if hpwl_before else 0
    print(f"HPWL improvement: {improvement:.1f}%")
    print(f"Final objective (HPWL + span): {solver.objective_value}")
    print(f"Written to {outfile}")

elif status == cp_model.INFEASIBLE:
    print("Solver found no feasible solution. Writing debug DEF with original locations...")
    base = outfile.rsplit(".", 1)[0]
    d_parser.writeDEF(f"{base}_debug_infeasible.def")
    print(f"Written to {base}_debug_infeasible.def")

else:
    print(f"Solver returned status: {solver.status_name(status)}")