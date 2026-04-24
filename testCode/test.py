import sys
import LEFDEFParser
from collections import namedtuple
from ortools.sat.python import cp_model



print(f"The current version is {sys.version}")
deffile = "../designs/deffiles/c17.def"
leffile = "../designs/leffiles/sky130.lef"
outfile = "../designs/deffiles/c17_placed.def"
d_parser = LEFDEFParser.DEFReader()
d_parser.readDEF(deffile)
l_parser = LEFDEFParser.LEFReader()
l_parser.readLEF(leffile)

chip_bbox = d_parser.bbox()
# Define bounds in DBU
x_max = chip_bbox.ur.x
y_max = chip_bbox.ur.y
site_height = 2720 # for Sky130 PDK

width = chip_bbox.ur.x - chip_bbox.ll.x
height = chip_bbox.ur.y - chip_bbox.ll.y
print(width, height)

model = cp_model.CpModel()

# Build lookup for Macro dimensions (Width, Height)
dimLookup = {m.name(): (m.xdim(), m.ydim()) for m in l_parser.macros()}

nodes = {}
for comp in d_parser.components():
    c_name = comp.name()
    w,h = dimLookup[comp.macro()]
    
    x_var = model.new_int_var(0, x_max - w, f"x_{c_name}")
    
    # Y variable: Align to Site Rows
    # num_rows ensures we stay in the core
    num_rows = y_max // site_height
    row_idx = model.new_int_var(0, num_rows - 1, f"row_{c_name}")
    y_var = model.new_int_var(0, y_max, f"y_{c_name}")
    
    model.add(y_var == row_idx * site_height)

    # Interval variables for NoOverlap2D
    x_interval = model.NewIntervalVar(x_var, w, x_var + w, f'x_int_{c_name}')
    y_interval = model.NewIntervalVar(y_var, h, y_var + h, f'y_int_{c_name}')
    
    nodes[c_name] = {
            'x': x_var, 
            'y': y_var, 
            'x_int': x_interval, 
            'y_int': y_interval,
            'w': w, 'h': h
        }

# Legality Constraints  
model.add_no_overlap_2d(
        [n['x_int'] for n in nodes.values()], 
        [n['y_int'] for n in nodes.values()]
    )

# Objective: Minimize HPWL Cost

net_costs = []
for net in d_parser.nets():
    pin_x_vars = []
    pin_y_vars = []
    
    for pin in net.pins():
        if pin[0] != 'PIN':
            pin_x_vars.append(nodes[pin[0]]['x'])
            pin_y_vars.append(nodes[pin[0]]['y'])
        # Boundary pins are fixed constants
        
    if len(pin_x_vars) > 2:
        continue
    
    # Net Bounding Box
    min_x = model.new_int_var(0, x_max, f"min_x_{net.name()}")
    max_x = model.new_int_var(0, x_max, f"max_x_{net.name()}")
    model.add_min_equality(min_x, pin_x_vars)
    model.add_max_equality(max_x, pin_x_vars)
    
    net_costs.append(max_x - min_x)
    
    min_y = model.new_int_var(0, y_max, f"min_y_{net.name()}")
    max_y = model.new_int_var(0, y_max, f"max_y_{net.name()}")
    model.add_min_equality(min_y, pin_y_vars)
    model.add_max_equality(max_y, pin_y_vars)
    
    net_costs.append(max_y - min_y)

model.minimize(sum(net_costs))


# Solving and writing back

solver = cp_model.CpSolver()
solver.parameters.num_search_workers = 8
status = solver.solve(model)
print(status)

if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    for comp in d_parser.components():
        new_x = solver.value(nodes[comp.name()]['x'])
        new_y = solver.value(nodes[comp.name()]['y'])
        comp.setLocation(new_x, new_y)
    
    d_parser.writeDEF(outfile)
    print("Detailed Placement Complete and Legal")
    
if status == cp_model.INFEASIBLE:
    print("Solver found no solution. Exporting initial state for debugging...")
    # The components still hold the locations from 'unplaced.def'
    # Use your parser's write function to see where they are currently stuck
    base_name = outfile.split(".")[0]
    d_parser.writeDEF(f"{base_name}_debug_infeasible.def")
    print(f"Written to {base_name}_debug_infeasible.def")