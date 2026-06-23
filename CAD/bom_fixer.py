import pandas as pd

def process_jlc_bom(input_filename, output_filename):
    try:
        # 1. Read your raw semicolon-delimited CSV file
        df = pd.read_csv(input_filename, sep=';')
        
        # 2. Filter out rows without reference designators and remove the "Board" metadata row
        cleaned_df = df.dropna(subset=['Reference Designators']).copy()
        cleaned_df = cleaned_df[cleaned_df['Description'] != 'Board']
        
        # 3. Build the structured DataFrame for JLCPCB requirements
        jlc_bom = pd.DataFrame()
        
        # Map raw 'Reference Designators' to 'Designator'
        jlc_bom['Designator'] = cleaned_df['Reference Designators']
        
        # Fill 'Comment' using the component Value field; fallback to Package name if blank
        jlc_bom['Comment'] = cleaned_df['Value'].fillna(cleaned_df['Package'])
        
        # Map raw 'Package' name to 'Footprint'
        jlc_bom['Footprint'] = cleaned_df['Package']
        
        # Add an empty column for LCSC Part Numbers to speed up factory verification
        jlc_bom['LCSC Part #'] = ''
        
        # 4. Export as a true comma-separated CSV file (without index columns)
        jlc_bom.to_csv(output_filename, index=False)
        print(f"🎉 Success! Cleaned BOM saved as: '{output_filename}'")
        
    except FileNotFoundError:
        print(f"❌ Error: Could not find the file '{input_filename}'. Make sure it's in the same folder.")
    except Exception as e:
        print(f"❌ An error occurred: {e}")

# Run the cleaning operation
process_jlc_bom('CAD/PCB_V1_BOM.csv', 'CAD/Fixed_PCB_V1_BOM.csv')