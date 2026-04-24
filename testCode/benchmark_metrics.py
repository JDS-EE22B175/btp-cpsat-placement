import LEFDEFParser

def calculate_hpwl(def_path):
    print(f"Parsing DEF: {def_path}")
    parser = LEFDEFParser.DEFReader()
    parser.readDEF(def_path)
    
    # 1. Build coordinate lookups for fast retrieval
    comp_coords = {c.name(): (c.location().x, c.location().y) for c in parser.components()}
    pin_coords = {p.name(): (p.origin().x, p.origin().y) for p in parser.pins()}
    
    total_hpwl = 0
    
    # 2. Iterate through all routed nets
    for net in parser.nets():
        pin_x = []
        pin_y = []
        
        for p in net.pins():
            # p[0] is the component name (or 'PIN' for external IO pins)
            # p[1] is the specific port name on that component
            if p[0] == 'PIN':
                if p[1] in pin_coords:
                    pin_x.append(pin_coords[p[1]][0])
                    pin_y.append(pin_coords[p[1]][1])
            else:
                if p[0] in comp_coords:
                    # For HPWL, using the component's base (x,y) is standard 
                    # academic practice when ignoring exact pin offsets
                    pin_x.append(comp_coords[p[0]][0])
                    pin_y.append(comp_coords[p[0]][1])
                    
        # 3. Calculate Bounding Box for this net
        if len(pin_x) > 1:
            net_hpwl = (max(pin_x) - min(pin_x)) + (max(pin_y) - min(pin_y))
            total_hpwl += net_hpwl

    print("="*35)
    print("           HPWL METRICS          ")
    print("="*35)
    print(f"Total HPWL: {total_hpwl} DBU")
    print("="*35)
    
    return total_hpwl

# --- Execution ---
print("--- Baseline (Global Placement) ---")
gp_hpwl = calculate_hpwl("../deffiles/project/def/c7552.def")

print("\n--- CP-SAT Legalized ---")
dp_hpwl = calculate_hpwl("../outputDefs/c7552_placed.def")

# Calculate the degradation percentage
if gp_hpwl > 0:
    degradation = ((dp_hpwl - gp_hpwl) / gp_hpwl) * 100
    print(f"\nHPWL Degradation during Legalization: {degradation:.2f}%")