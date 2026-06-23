import pandas as pd

def process_two_sided_cpl(bom_filename, mnt_filename, output_filename):
    try:
        # 1. Load the raw BOM to build a reference dictionary for component layers
        bom_df = pd.read_csv(bom_filename, sep=';')
        
        # Clean designators and create a lookup mapping: { 'R1': 'Top', 'J1': 'Bottom', ... }
        layer_lookup = {}
        for _, row in bom_df.dropna(subset=['Reference Designators']).iterrows():
            layer_val = str(row['Top/Bottom']).strip()
            
            # Convert default dash or missing values into standard 'Top'
            if layer_val in ['', '-', 'nan', 'None']:
                assigned_layer = 'Top'
            else:
                assigned_layer = layer_val.capitalize() # Ensures "Top" or "Bottom"
            
            # Split grouped designators (e.g., "D1, D2" -> ['D1', 'D2'])
            designators = [d.strip() for d in str(row['Reference Designators']).split(',')]
            for des in designators:
                if des:
                    layer_lookup[des] = assigned_layer

        # 2. Process the .mnt file using the lookup dictionary
        rows = []
        with open(mnt_filename, 'r') as f:
            for line in f:
                if not line.strip():
                    continue
                
                parts = [p.strip() for p in line.split(';')]
                if len(parts) >= 5:
                    try:
                        # Convert Imperial Mils into standard Metric MM
                        mid_x = round(float(parts[1]) * 0.0254, 3)
                        mid_y = round(float(parts[2]) * 0.0254, 3)
                    except ValueError:
                        continue
                        
                    rotation = parts[3]
                    designator = parts[4]
                    
                    # Dynamically look up the exact layer from the BOM, fallback to 'Top' if missing
                    actual_layer = layer_lookup.get(designator, 'Top')
                    
                    rows.append({
                        'Designator': designator,
                        'Mid X': mid_x,
                        'Mid Y': mid_y,
                        'Layer': actual_layer,
                        'Rotation': rotation
                    })
                    
        # 3. Create the final clean JLCPCB-compliant CSV
        df_cpl = pd.DataFrame(rows)
        df_cpl = df_cpl[['Designator', 'Mid X', 'Mid Y', 'Layer', 'Rotation']]
        
        df_cpl.to_csv(output_filename, index=False)
        print(f"🎉 Success! Processed {len(df_cpl)} components split across Top and Bottom layers.")
        print(f"📁 Multi-side CPL saved as: '{output_filename}'")
        
        # Display a quick summary of the split
        print(df_cpl['Layer'].value_counts())
        
    except Exception as e:
        print(f"❌ An error occurred: {e}")

# Run the dual-side mapping process
process_two_sided_cpl('CAD/PCB_V1_BOM.csv', 'CAD/PCB_V1_CPL.mnt', 'CAD/Fixed_PCB_V1_CPL.csv')