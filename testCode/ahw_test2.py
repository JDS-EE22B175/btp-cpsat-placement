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
              f"Current Objective = {self.ObjectiveValue()}")

# --- Initialization ---
deffile = "../designs/deffiles/c17.def"
leffile = "../designs/leffiles/sky130.lef"
outfile = "../designs/deffiles/c17_placed.def"

d_parser = LEFDEFParser.DEFReader()
d_parser.readDEF(deffile)
l_parser = LEFDEFParser.LEFReader()
l_parser.readLEF(leffile)

dimLookup = {m.name(): (m.xdim(), m.ydim()) for m in l_parser.macros()}
model = cp_model.CpModel()

# --- 1. DYNAMIC CORE EXTRACTION (X and Y) ---
# We find the legal core area by scanning PHY_EDGE cells and ROW definitions
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
    
    # Track existing Y placements to find row range
    if "PHY_EDGE" in name or "TAP" in name:
        y_coords.append(comp.location().y)

core_x_min = max(left_edge_coords) if left_edge_coords else 0
core_x_max = min(right_edge_coords) if right_edge_coords else d_parser.bbox().ur.x
core_y_min = min(y_coords) if y_coords else 0
core_y_max = max(y_coords) if y_coords else d_parser.bbox().ur.y
site_height = 2720 # Sky130 Row Height

print(f"Core Placement Zone: X({core_x_min} to {core_x_max}), Y({core_y_min} to {core_y_max})")

# --- Updated Variable Creation with Obstacle Awareness ---
x_intervals = []
y_intervals = []
nodes = {}

for comp in d_parser.components():
    c_name = comp.name()
    w, h = dimLookup[comp.macro()]
    loc = comp.location()

    # Case 1: Infrastructure (TAP and PHY_EDGE)
    if c_name.startswith("TAP_") or c_name.startswith("PHY_EDGE_"):
        # Create FIXED intervals. These have no variables, just constants.
        # This tells the solver: "This space is already occupied."
        x_int = model.new_fixed_size_interval_var(loc.x, w, f'fixed_x_{c_name}')
        y_int = model.new_fixed_size_interval_var(loc.y, h, f'fixed_y_{c_name}')
        x_intervals.append(x_int)
        y_intervals.append(y_int)
        continue 

    # Case 2: Movable Logic Gates
    # Use the dynamic core boundaries we calculated
    x_var = model.new_int_var(core_x_min, core_x_max - w, f"x_{c_name}")
    num_rows = (core_y_max - core_y_min) // site_height + 1
    row_idx = model.new_int_var(0, num_rows - 1, f"row_{c_name}")
    y_var = model.new_int_var(core_y_min, core_y_max, f"y_{c_name}")
    model.add(y_var == core_y_min + (row_idx * site_height))

    # Create movable intervals
    x_int = model.new_interval_var(x_var, w, x_var + w, f'x_int_{c_name}')
    y_int = model.new_interval_var(y_var, h, y_var + h, f'y_int_{c_name}')
    
    x_intervals.append(x_int)
    y_intervals.append(y_int)
    
    nodes[c_name] = {'x': x_var, 'y': y_var, 'w': w, 'h': h}

# --- Apply NoOverlap to BOTH movable and fixed intervals ---
# This is what prevents the logic gates from sitting on top of TAP cells
model.add_no_overlap_2d(x_intervals, y_intervals)

# HPWL Calculation
net_costs = []
for net in d_parser.nets():
    pin_x_vars = [nodes[p[0]]['x'] for p in net.pins() if p[0] in nodes]
    pin_y_vars = [nodes[p[0]]['y'] for p in net.pins() if p[0] in nodes]
    if len(pin_x_vars) < 2: continue
    
    min_x, max_x = model.new_int_var(core_x_min, core_x_max, ""), model.new_int_var(core_x_min, core_x_max, "")
    model.add_min_equality(min_x, pin_x_vars); model.add_max_equality(max_x, pin_x_vars)
    min_y, max_y = model.new_int_var(core_y_min, core_y_max, ""), model.new_int_var(core_y_min, core_y_max, "")
    model.add_min_equality(min_y, pin_y_vars); model.add_max_equality(max_y, pin_y_vars)
    net_costs.append((max_x - min_x) + (max_y - min_y))

# Area Span Optimization
g_min_x, g_max_x = model.new_int_var(core_x_min, core_x_max, ""), model.new_int_var(core_x_min, core_x_max, "")
g_min_y, g_max_y = model.new_int_var(core_y_min, core_y_max, ""), model.new_int_var(core_y_min, core_y_max, "")
model.add_min_equality(g_min_x, [n['x'] for n in nodes.values()])
model.add_max_equality(g_max_x, [n['x'] + n['w'] for n in nodes.values()])
model.add_min_equality(g_min_y, [n['y'] for n in nodes.values()])
model.add_max_equality(g_max_y, [n['y'] + n['h'] for n in nodes.values()])

occupied_span = (g_max_x - g_min_x) + (g_max_y - g_min_y)
model.minimize(sum(net_costs) + occupied_span)

# --- 4. Solving & Writing ---
solver = cp_model.CpSolver()
solver.parameters.num_search_workers = 8
solver.parameters.log_search_progress = True 
status = solver.solve(model, PlacementCallback())

if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
    for comp in d_parser.components():
        node_data = nodes.get(comp.name())
        if node_data:
            comp.setLocation(solver.value(node_data['x']), solver.value(node_data['y']))
    d_parser.writeDEF(outfile)
    print(f"Success! Final Objective: {solver.objective_value}")