import pandas as pd
from io import BytesIO
from io import StringIO
from minio import Minio
from typing import List, Dict, Any

# MinIO Client Configuration
minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",      # MinIO access key
    secret_key="password",   # MinIO secret key
    secure=False             # Set to True if using HTTPS
)

def parse_automated_input(input_list):
    """
    Parse the automated input list format where:
    - Each element except last two are lists of parquet files for respective user IDs
    - Last two elements are keys
    Returns: user_ids, timestamps, and keys
    """
    if len(input_list) < 3:  # Need at least one user data list and two keys
        raise ValueError("Input list must contain at least one user data list and two keys")
    
    # Extract keys (last two elements)
    keys = input_list[-2:]
    
    # Process user data lists (all elements except last two)
    user_data = input_list[:-2]
    
    user_ids = set()
    timestamps = set()
    
    # Process each user's parquet file list
    for parquet_list in user_data:
        for parquet_file in parquet_list:
            # Extract user_id and timestamp from parquet filename
            try:
                user_id, timestamp = parquet_file.split('/')
                user_ids.add(user_id)
                # Remove .parquet extension if present
                timestamp = timestamp.replace('.parquet', '')
                timestamps.add(timestamp)
            except ValueError:
                continue
    
    return list(user_ids), list(timestamps), keys

def extract_key_data(df: pd.DataFrame, key: str) -> Any:
    """
    Extract data for a specific key from the DataFrame.
    Handles nested JSON and dictionary structures.
    """
    # Check if key is directly in columns
    if key in df.columns:
        return df[key]
    
    # Look for the key in JSON-encoded columns
    for column in df.columns:
        try:
            # Get the first row of the column
            column_data = df[column].iloc[0]
            
            # If it's a string, try to parse as JSON
            if isinstance(column_data, str):
                try:
                    column_data = pd.json_normalize(pd.read_json(StringIO(column_data)))
                    if key in column_data.columns:
                        return column_data[key]
                except:
                    continue
            
            # If it's already a dict
            elif isinstance(column_data, dict):
                if key in column_data:
                    return pd.Series([column_data[key]])
                # Check nested dictionaries
                for value in column_data.values():
                    if isinstance(value, dict) and key in value:
                        return pd.Series([value[key]])
                        
        except Exception:
            continue
            
    return pd.Series([f"Key '{key}' not found"])

def retrieve_data(user_ids: List[str], timestamps: List[str], keys: List[str], bucket_name: str = "wl-data") -> List[pd.DataFrame]:
    """
    Retrieve data from MinIO and return a list of DataFrames.
    Each DataFrame contains the data for one user-timestamp combination.
    """
    dataframes = []

    for user_id in user_ids:
        for timestamp in timestamps:
            try:
                # Construct object name and retrieve parquet file
                object_name = f"{user_id}/{timestamp}.parquet"
                response = minio_client.get_object(bucket_name, object_name)
                raw_df = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
                
                # Create a new DataFrame with user_id and timestamp
                result_df = pd.DataFrame({
                    'user_id': [user_id],
                    'timestamp': [timestamp]
                })
                
                # Extract data for each requested key
                for key in keys:
                    result_df[key] = extract_key_data(raw_df, key)
                
                dataframes.append(result_df)
                
            except Exception as e:
                # Create error DataFrame
                error_df = pd.DataFrame({
                    'user_id': [user_id],
                    'timestamp': [timestamp],
                    'error': [str(e)]
                })
                dataframes.append(error_df)

    return dataframes

def handle_automated_query(input_list: List[Any]) -> List[pd.DataFrame]:
    """
    Handle automated query processing from the input list format.
    Returns a list of DataFrames containing the results.
    
    Example input format:
    [
        ["02:00:00:00:00:00/2024-10-22 18:16:22.530491.parquet", 
         "02:00:00:00:00:00/2024-10-22 18:16:23.000001.parquet"],
        ["03:00:00:00:00:00/2024-10-22 18:16:22.530491.parquet"],
        "GPS",
        "rssi"
    ]
    """
    try:
        user_ids, timestamps, keys = parse_automated_input(input_list)
        return retrieve_data(user_ids, timestamps, keys)
    except Exception as e:
        # Return a single DataFrame with the error
        return [pd.DataFrame({'error': [str(e)]})]

if __name__ == "__main__":
    # Example usage with automated input
    example_input = [
        ["02:00:00:00:00:00/2024-10-22 18:16:22.530491.parquet"],
        ["03:00:00:00:00:00/2024-10-22 18:16:22.530491.parquet"],
        "GPS",
        "rssi"
    ]
    
    try:
        # Get list of DataFrames
        result_dfs = handle_automated_query(example_input)
        
        # Display each DataFrame
        for i, df in enumerate(result_dfs):
            print(f"\nDataFrame {i + 1}:")
            print(df)
            
    except Exception as e:
        print(f"Error processing query: {e}")