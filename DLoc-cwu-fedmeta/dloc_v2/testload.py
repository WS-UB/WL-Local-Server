import pyarrow.parquet as pq
import pandas as pd
from pathlib import Path

def parquet_to_txt(input_path, output_path=None, max_rows=None):
    """
    Parse a Parquet file and save as a text file
    
    Args:
        input_path (str): Path to the Parquet file
        output_path (str): Optional path for output text file
                          Defaults to same directory as input with .txt extension
        max_rows (int): Maximum number of rows to process (None for all rows)
        
    Returns:
        str: Path to the created text file
    """
    try:
        # Set default output path if not provided
        if output_path is None:
            input_path = Path(input_path)
            output_path = input_path.with_suffix('.txt')
        
        # Read Parquet file
        table = pq.read_table(input_path)
        df = table.to_pandas()
        
        # Limit rows if requested
        if max_rows is not None and max_rows < len(df):
            df = df.head(max_rows)
        
        # Create text file
        with open(output_path, 'w', encoding='utf-8') as f:
            # Write header
            f.write('\t'.join(df.columns) + '\n')
            
            # Write data rows
            for _, row in df.iterrows():
                f.write('\t'.join(str(x) for x in row.values) + '\n')
        
        print(f"Successfully converted {input_path} to {output_path}")
        print(f"Rows processed: {len(df)}")
        print(f"Columns: {', '.join(df.columns)}")
        
        return str(output_path)
        
    except Exception as e:
        print(f"Error processing {input_path}: {str(e)}")
        return None

# Usage example
input_file = "./data/2025-04-20 18_01_03.006 (1).parquet"  # Replace with your Parquet file path
output_file = "./data/new.txt"  # Optional - will auto-generate if None

result_path = parquet_to_txt(input_file, output_file)
if result_path:
    print(f"Text file created at: {result_path}")