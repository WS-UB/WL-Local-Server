import json
import pandas as pd
from io import BytesIO
from minio import Minio

# MinIO Client Configuration
minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",      # MinIO access key
    secret_key="password",   # MinIO secret key
    secure=False             # Set to True if using HTTPS
)

def convert_to_serializable(obj):
    """
    Convert numpy arrays and other non-serializable objects to JSON serializable format.
    """
    import numpy as np
    
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: convert_to_serializable(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_to_serializable(item) for item in list]
    return obj

def parse_query_list(query_list):
    """
    Parse the query list into user_ids, timestamps, and keys.
    Handles:
    - Combined user_id/timestamp entries (e.g., "02:00:00:00:00:00/2024-10-22 18:16:22.530491")
    - Multiple comma-separated entries
    - Keys without timestamps
    """
    user_ids = set()
    timestamps = set()
    keys = []
    
    for item in query_list:
        # Check if the item contains a MAC address pattern
        if ":" in item and "/" in item:
            # Split combined entries (potentially comma-separated)
            entries = item.split(",")
            
            for entry in entries:
                entry = entry.strip()
                # Remove any asterisks from the entry
                entry = entry.replace("*", "")
                
                if "/" in entry:
                    user_id, timestamp = entry.split("/", 1)
                    user_ids.add(user_id.strip())
                    timestamps.add(timestamp.strip())
        # If it's a MAC address alone
        elif ":" in item and item.count(":") == 5:
            user_ids.add(item.strip())
        # If it doesn't match user_id patterns, treat it as a key
        else:
            keys.append(item.strip())
    
    user_ids = list(user_ids)
    timestamps = list(timestamps)
    
    if not user_ids:
        raise ValueError("No valid user_id found in the query list.")
    if not keys:
        raise ValueError("No keys specified for data retrieval.")
    
    return user_ids, timestamps, keys

def retrieve_data(user_ids, timestamps, keys, bucket_name="wl-data"):
    """
    Retrieve data from MinIO based on user_ids, timestamps, and keys.
    """
    results = {}

    for user_id in user_ids:
        for timestamp in timestamps:
            object_name = f"{user_id}/{timestamp}.parquet"
            try:
                response = minio_client.get_object(bucket_name, object_name)
                data = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
                
                key_data = {}
                
                # Process each requested key
                for key in keys:
                    key_found = False
                    
                    # Check each column in the dataframe
                    for column in data.columns:
                        try:
                            column_data = data[column].iloc[0]
                            
                            # Convert string to dict if it's JSON
                            if isinstance(column_data, str):
                                try:
                                    column_data = json.loads(column_data)
                                except json.JSONDecodeError:
                                    continue
                                
                            # If column_data is a dict, search for the key
                            if isinstance(column_data, dict):
                                # Case 1: The key is the column name itself
                                if column == key:
                                    key_data[key] = convert_to_serializable(column_data)
                                    key_found = True
                                    break
                                    
                                # Case 2: The key is inside the column's dict
                                elif key in column_data:
                                    key_data[key] = convert_to_serializable(column_data[key])
                                    key_found = True
                                    break
                                    
                                # Case 3: Search for nested keys in all dictionary values
                                else:
                                    for value in column_data.values():
                                        if isinstance(value, dict) and key in value:
                                            key_data[key] = convert_to_serializable(value[key])
                                            key_found = True
                                            break
                                    if key_found:
                                        break
                            
                            # If the column itself matches the key
                            elif column == key:
                                key_data[key] = convert_to_serializable(column_data)
                                key_found = True
                                break
                                
                        except Exception as e:
                            continue
                    
                    if not key_found:
                        key_data[key] = f"Key '{key}' not found in data"

                results[f"{user_id}_{timestamp}"] = {
                    "user_id": user_id,
                    "timestamp": timestamp,
                    "data": key_data
                }

            except Exception as e:
                results[f"{user_id}_{timestamp}"] = {"error": f"Unable to retrieve data: {e}"}

    return results

def handle_query(query_string):
    """
    Main handler for the query. Takes a query string and processes it.
    Example query format:
    "02:00:00:00:00:00/2024-10-22 18:16:22.530491,02:00:00:00:00:00/2024-10-22 18:16:23.000001,GPS"
    """
    try:
        # Split the query string into a list
        query_list = query_string.split(",")
        user_ids, timestamps, keys = parse_query_list(query_list)
        return retrieve_data(user_ids, timestamps, keys)
    except Exception as e:
        return {"error": str(e)}

if __name__ == "__main__":
    while True:
        try:
            # Get query input from user
            query_string = input("Enter query (or 'quit' to exit): ")
            
            # Check for exit condition
            if query_string.lower() == 'quit':
                break
                
            # Process query and print results
            result = handle_query(query_string)
            print(json.dumps(result, indent=4))
            
        except Exception as e:
            print(f"Error processing query: {e}")