import json
import io
from io import BytesIO
from minio import Minio
from typing import List, Any, Dict, Union
from minio.error import MinioException, S3Error
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

# MinIO Client Configuration
minio_client = Minio(
    "128.205.218.189:9000",
    access_key="admin",
    secret_key="password",
    secure=False,
)

def get_friendly_error_message(e: Exception, user_id: str = None, timestamp: str = None) -> str:
    """
    Convert technical error messages into user-friendly messages.
    """
    error_str = str(e)
    
    if isinstance(e, S3Error):
        if "NoSuchKey" in error_str:
            if user_id and timestamp:
                return f"Data not found for User ID '{user_id}' at timestamp '{timestamp}'. Please verify both the User ID and timestamp are correct."
            elif user_id:
                return f"User ID '{user_id}' does not exist in the database. Please verify the User ID."
        elif "NoSuchBucket" in error_str:
            return "Database not found. Please contact system administrator."
        elif "AccessDenied" in error_str:
            return "Access denied. Please verify your credentials."
    
    if "ParquetError" in error_str:
        return "Error reading data file. The file might be corrupted or in wrong format."
    
    if "JSONDecodeError" in error_str:
        return "Error parsing JSON data. The data might be corrupted or in wrong format."
    
    return f"An unexpected error occurred: {str(e)}"


def extract_key_data(df: pd.DataFrame, key: str) -> pd.Series:
    """
    Extract data for a specific key from the DataFrame.
    Returns the complete structure for the key without requiring dot notation.
    """
    # Check if the key is directly in columns
    if key in df.columns:
        return df[key]
    
    results = []
    
    for _, row in df.iterrows():
        row_result = None
        
        # Try each column for nested data
        for column in df.columns:
            try:
                column_data = row[column]
                
                # Handle string JSON
                if isinstance(column_data, str):
                    try:
                        column_data = json.loads(column_data)
                    except json.JSONDecodeError:
                        continue
                
                # If the key exists directly in the parsed data
                if isinstance(column_data, dict) and key in column_data:
                    row_result = column_data[key]
                    break
                # If it's a list, check each item
                elif isinstance(column_data, list):
                    for item in column_data:
                        if isinstance(item, dict) and key in item:
                            row_result = item[key]
                            break
                    if row_result is not None:
                        break
                    
            except Exception as e:
                print(f"Error processing column '{column}' for key '{key}': {e}")
                continue
        
        results.append(row_result if row_result is not None else None)
    
    return pd.Series(results)

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
                    print(f"Extracting key: {key}")  # Debug print
                    extracted_data = extract_key_data(raw_df, key)

                    if extracted_data.isna().all():
                        error_message = f"No such key '{key}'."
                        print(error_message)
                        extracted_data = pd.Series([error_message] * len(raw_df))
                    
                    # If the extracted data is a dictionary or list, store it as JSON string
                    if isinstance(extracted_data.iloc[0], (dict, list)):
                        extracted_data = extracted_data.apply(lambda x: json.dumps(x) if x is not None else None)
                    
                    result_df[key] = extracted_data
                
                dataframes.append(result_df)
            except S3Error as e:
                # Check for NoSuchKey error and treat it as missing userID or timestamp
                if e.code == 'NoSuchKey':
                    error_message = f"UserID '{user_id}' or timestamp '{timestamp}' incorrect or does not exist in the database."
                else:
                    # For other S3 errors, retain the default error message
                    error_message = f"S3 operation failed; code: {e.code}, message: {e.message}"

                # Log the error and create the error DataFrame
                print(f"Error retrieving data for {user_id} at {timestamp}: {error_message}")
                error_df = pd.DataFrame({
                    'user_id': [user_id],
                    'timestamp': [timestamp],
                    'error': [error_message]
                })
                dataframes.append(error_df)

            except Exception as e:
                # Handle generic exceptions and log them as errors
                error_message = f"An unexpected error occurred: {str(e)}"
                print(f"Error retrieving data for {user_id} at {timestamp}: {error_message}")
                error_df = pd.DataFrame({
                    'user_id': [user_id],
                    'timestamp': [timestamp],
                    'error': [error_message]
                })
                dataframes.append(error_df)

    return dataframes

def get_nested_keys(data, prefix=''):
    """
    Recursively extract nested keys from a dictionary or list.
    """
    nested_keys = []
    
    if isinstance(data, dict):
        for key, value in data.items():
            full_key = f"{prefix}.{key}" if prefix else key
            nested_keys.append(full_key)
            
            # Recursively handle nested structures
            if isinstance(value, (dict, list)):
                nested_keys.extend(get_nested_keys(value, full_key))
    
    elif isinstance(data, list) and data and isinstance(data[0], dict):
        # If it's a list of dictionaries, use the first item to extract structure
        first_item = data[0]
        for key, value in first_item.items():
            full_key = f"{prefix}.{key}" if prefix else key
            nested_keys.append(full_key)
            
            # Recursively handle nested structures
            if isinstance(value, (dict, list)):
                nested_keys.extend(get_nested_keys(value, full_key))
    
    return nested_keys

def get_available_keys(user_ids: List[str], timestamps: List[str], bucket_name: str = "wl-data") -> Dict[str, List[str]]:
    """
    Get available keys from the data, showing both main keys and their nested structure.
    """
    available_keys = {}

    for user_id in user_ids:
        for timestamp in timestamps:
            object_name = f"{user_id}/{timestamp}.parquet"
            print(f"Requesting object: {object_name}")
            try:
                response = minio_client.get_object(bucket_name, object_name)
                raw_df = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
                
                # Initialize the keys list for this user_id/timestamp combination
                keys_structure = []
                
                # Process each column
                for column in raw_df.columns:
                    sample_value = raw_df[column].dropna().iloc[0] if not raw_df[column].empty else None
                    
                    if sample_value is not None:
                        if isinstance(sample_value, str):
                            try:
                                # Try to parse as JSON to get nested structure
                                parsed_value = json.loads(sample_value)
                                if isinstance(parsed_value, (dict, list)):
                                    # Get nested keys with the column as prefix
                                    nested_keys = get_nested_keys(parsed_value, column)
                                    keys_structure.extend(nested_keys)
                                else:
                                    keys_structure.append(column)
                            except json.JSONDecodeError:
                                keys_structure.append(column)
                        else:
                            keys_structure.append(column)
                
                available_keys[f"{user_id}_{timestamp}"] = sorted(set(keys_structure))
                
            except Exception as e:
                available_keys[f"{user_id}_{timestamp}"] = [f"Error: {str(e)}"]

    return available_keys

def parse_automated_input(input_list):
    """Parse the automated input list format"""
    if len(input_list) < 3:
        raise ValueError("Input list must contain at least one user data list and two keys")
    
    keys = input_list[-2:]
    user_data = input_list[:-2]
    
    user_ids = set()
    timestamps = set()
    
    for parquet_list in user_data:
        for parquet_file in parquet_list:
            try:
                user_id, timestamp = parquet_file.split('/')
                user_ids.add(user_id)
                timestamp = timestamp.replace('.parquet', '')
                timestamps.add(timestamp)
            except ValueError:
                continue
    
    return list(user_ids), list(timestamps), keys

def handle_automated_query(input_list: List[Any]) -> List[pd.DataFrame]:
    """Handle automated query processing from the input list format."""
    try:
        user_ids, timestamps, keys = parse_automated_input(input_list)
        return retrieve_data(user_ids, timestamps, keys)
    except Exception as e:
        return [pd.DataFrame({'error': [str(e)]})]

if __name__ == "__main__":
    # Example usage
    example_input = [
        ["02:00:00:00:00:00/2024-10-22 18:44:01.630797.parquet"],
        ["03:00:00:00:00:00/2024-10-28 14:16:12.123280.parquet"],
        "IMU",  # Now will return complete IMU structure
        "rssi"
    ]

    try:
        user_ids = ["02:00:00:00:00:00", "03:00:00:00:00:00"]
        timestamps = ["2024-10-22 18:44:01.630797", "2024-10-28 14:16:12.123280"]
        
        print("\nTesting get_available_keys function:")
        available_keys = get_available_keys(user_ids, timestamps)
        for key, value in available_keys.items():
            print(f"\n{key}:")
            print(json.dumps(value, indent=2))
        
        result_dfs = handle_automated_query(example_input)
        
        for i, df in enumerate(result_dfs):
            print(f"\nDataFrame {i + 1}:")
            print(df)
            
    except Exception as e:
        print(f"Error processing query: {e}")