import pandas as pd
import pyarrow.parquet as pq

def normalize_gps_from_parquet(parquet_file_path):
    """
    Read GPS data from parquet file and normalize coordinates relative to davis_box.
    
    Args:
        parquet_file_path: Path to the parquet file containing GPS data
        
    Returns:
        Tuple of (lat_new, long_new) normalized coordinates
    """
    # Hardcoded Davis box coordinates
    davis_box = [
        (43.00297463348004, -78.78674402431034),
        (43.00242950065132, -78.7868579571325),
        (43.00240355874723, -78.78777435710242),
        (43.0030251198603, -78.78781856606011)
    ]
    
    # Calculate min/max bounds from davis_box
    lats = [point[0] for point in davis_box]
    longs = [point[1] for point in davis_box]
    lat_min, lat_max = min(lats), max(lats)
    long_min, long_max = min(longs), max(longs)
    
    try:
        # Read the parquet file
        df = pd.read_parquet(parquet_file_path)
        
        # Extract GPS data (assuming column named 'GPS' with dict containing 'latitude'/'longitude')
        gps_data = df['GPS'].iloc[0]  # Get first row's GPS data
        
        # Handle case where GPS might be stored as string
        if isinstance(gps_data, str):
            import json
            gps_data = json.loads(gps_data.replace("'", '"'))
        
        gps_lat = gps_data['latitude']
        gps_long = gps_data['longitude']
        
        # Compute normalized coordinates
        lat_new = (gps_lat - lat_min) / (lat_max - lat_min)
        long_new = (gps_long - long_min) / (long_max - long_min)
        
        return lat_new, long_new
        
    except Exception as e:
        print(f"Error processing file: {str(e)}")
        return None, None

# Example usage:
# lat_norm, long_norm = normalize_gps_from_parquet('your_file.parquet')
# print(f"Normalized coordinates: ({lat_norm}, {long_norm})")

# if __name__ == "__main__":
#     # Example usage
#     lat_norm, long_norm = normalize_gps_from_parquet('./data/2025-04-01 17_57_32.769.parquet')
#     print(f"Normalized coordinates: ({lat_norm}, {long_norm})")