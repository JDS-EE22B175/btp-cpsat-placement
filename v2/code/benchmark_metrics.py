import sys
import LEFDEFParser

def get_comp_and_pin_coords(parser):
    """Extracts coordinates for movable components and external pins."""
    comp_coords = {}
    for comp in parser.components():
        c_name = comp.name()
        # Ignore fixed infrastructure
        if not (c_name.startswith("TAP_") or c_name.startswith("PHY_EDGE_")):
            comp_coords[c_name] = (comp.location().x, comp.location().y)
    
    pin_coords = {p.name(): (p.origin().x, p.origin().y) for p in parser.pins()}
    return comp_coords, pin_coords

def calculate_displacement(gp_coords, dp_coords):
    """Calculates Manhattan displacement between global and detailed placement."""
    total_displacement = 0
    max_displacement = 0
    moved_cells = 0
    zero_movement_cells = 0

    for c_name, (old_x, old_y) in gp_coords.items():
        if c_name in dp_coords:
            new_x, new_y = dp_coords[c_name]
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
    
    return total_evaluated, moved_cells, zero_movement_cells, total_displacement, max_displacement, avg_displacement

def calculate_hpwl(parser, comp_coords, pin_coords):
    """Calculates the Half-Perimeter Wirelength (HPWL) of all routed nets."""
    total_hpwl = 0
    for net in parser.nets():
        pin_x, pin_y = [], []
        for p in net.pins():
            if p[0] == 'PIN':
                if p[1] in pin_coords:
                    pin_x.append(pin_coords[p[1]][0])
                    pin_y.append(pin_coords[p[1]][1])
            else:
                if p[0] in comp_coords:
                    pin_x.append(comp_coords[p[0]][0])
                    pin_y.append(comp_coords[p[0]][1])
        
        if len(pin_x) > 1:
            total_hpwl += (max(pin_x) - min(pin_x)) + (max(pin_y) - min(pin_y))
    return total_hpwl

def calculate_occupied_area(parser, dimLookup):
    """Calculates the bounding box span and occupied area of the logic cluster."""
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = 0, 0
    
    for comp in parser.components():
        c_name = comp.name()
        if not (c_name.startswith("TAP_") or c_name.startswith("PHY_EDGE_")):
            loc_x = comp.location().x
            loc_y = comp.location().y
            w, h = dimLookup.get(comp.macro(), (0, 0))
            
            if loc_x < min_x: min_x = loc_x
            if loc_y < min_y: min_y = loc_y
            if loc_x + w > max_x: max_x = loc_x + w
            if loc_y + h > max_y: max_y = loc_y + h
            
    width = max_x - min_x if max_x >= min_x else 0
    height = max_y - min_y if max_y >= min_y else 0
    area = width * height
    return width, height, area

def run_benchmarks(lef_path, gp_def_path, dp_def_path):
    print("Loading LEF File for Macro Dimensions...")
    l_parser = LEFDEFParser.LEFReader()
    l_parser.readLEF(lef_path)
    dimLookup = {m.name(): (m.xdim(), m.ydim()) for m in l_parser.macros()}

    print(f"Parsing Global Placement DEF: {gp_def_path}")
    gp_parser = LEFDEFParser.DEFReader()
    gp_parser.readDEF(gp_def_path)
    
    print(f"Parsing Detailed Placement DEF: {dp_def_path}")
    dp_parser = LEFDEFParser.DEFReader()
    dp_parser.readDEF(dp_def_path)

    # Extract coordinates
    gp_comp_coords, gp_pin_coords = get_comp_and_pin_coords(gp_parser)
    dp_comp_coords, dp_pin_coords = get_comp_and_pin_coords(dp_parser)

    # 1. Displacement Metrics
    tot_eval, moved, zero_moved, tot_disp, max_disp, avg_disp = calculate_displacement(gp_comp_coords, dp_comp_coords)

    # 2. HPWL Metrics
    gp_hpwl = calculate_hpwl(gp_parser, gp_comp_coords, gp_pin_coords)
    dp_hpwl = calculate_hpwl(dp_parser, dp_comp_coords, dp_pin_coords)
    degradation = ((dp_hpwl - gp_hpwl) / gp_hpwl) * 100 if gp_hpwl > 0 else 0

    # 3. Area Metrics
    gp_w, gp_h, gp_area = calculate_occupied_area(gp_parser, dimLookup)
    dp_w, dp_h, dp_area = calculate_occupied_area(dp_parser, dimLookup)
    area_expansion = ((dp_area - gp_area) / gp_area) * 100 if gp_area > 0 else 0

    # Print Final Consolidated Report
    print("\n" + "="*50)
    print("           BTP LEGALIZATION BENCHMARK REPORT           ")
    print("="*50)
    
    print("\n--- 1. MANHATTAN DISPLACEMENT ---")
    print(f"Total Gates Evaluated : {tot_eval}")
    print(f"Gates Moved           : {moved}")
    print(f"Gates Unchanged       : {zero_moved}")
    print(f"Total Displacement    : {tot_disp} DBU")
    print(f"Max Displacement      : {max_disp} DBU")
    print(f"Average Displacement  : {avg_disp:.2f} DBU")
    
    print("\n--- 2. HALF-PERIMETER WIRELENGTH (HPWL) ---")
    print(f"Initial GP HPWL       : {gp_hpwl} DBU")
    print(f"Final Legal DP HPWL   : {dp_hpwl} DBU")
    print(f"HPWL Degradation      : {degradation:.2f}%")
    
    print("\n--- 3. OCCUPIED LOGIC AREA ---")
    print(f"Initial GP Span       : {gp_w} x {gp_h} DBU")
    print(f"Final Legal DP Span   : {dp_w} x {dp_h} DBU")
    print(f"Initial GP Area       : {gp_area} DBU^2")
    print(f"Final Legal DP Area   : {dp_area} DBU^2")
    print(f"Area Expansion        : {area_expansion:.2f}%")
    print("="*50 + "\n")

if __name__ == "__main__":
    # --- Execution Paths ---
    # Update these paths to match your BTP directory structure if running from terminal
    LEF_FILE = "../../designs/leffiles/sky130.lef"
    GP_DEF = "../../designs/deffiles/c7552_global_placed.def"
    DP_DEF = "../../outputDefs/c7552_placed.def"
    
    # Allows passing arguments from the terminal: python3 benchmark_metrics.py <GP_DEF> <DP_DEF>
    if len(sys.argv) == 3:
        GP_DEF = sys.argv[1]
        DP_DEF = sys.argv[2]
        
    run_benchmarks(LEF_FILE, GP_DEF, DP_DEF)