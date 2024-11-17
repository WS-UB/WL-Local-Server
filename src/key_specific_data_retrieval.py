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

# Function to parse the list of strings into the expected format
def parse_query_list(query_list):
    """
    Parses the query list into requests (user_id, timestamps) and fields.
    """
    try:
        user_ids = list({item for item in query_list if "02:" in item})  # User IDs typically contain colons
        timestamps = [item for item in query_list if " " in item]  # Identify timestamps based on space
        fields = query_list[len(user_ids) + len(timestamps):]  # Remaining items are fields

        print("User IDs:", user_ids)
        print("Timestamps:", timestamps)
        print("Fields:", fields)

        if not user_ids:
            raise ValueError("No user_id provided in the query.")

        # Generate requests for all combinations of user_ids and timestamps
        requests = []
        for user_id in user_ids:
            if timestamps:
                requests.extend([{"user_id": user_id, "timestamp": ts} for ts in timestamps])
            else:
                requests.append({"user_id": user_id})  # Retrieve all timestamps for this user_id if none provided

        print("Requests:", requests)
        print("Fields to Retrieve:", fields)

        return requests, fields
    except Exception as e:
        print(f"Error parsing query list: {str(e)}")
        return None, None


# Function to list all objects in MinIO for a given user_id
def list_user_objects(user_id, bucket_name="wl-data"):
    """
    List all objects for a given user_id in the specified bucket.
    """
    try:
        objects = minio_client.list_objects(bucket_name, prefix=f"{user_id}/", recursive=True)
        return [obj.object_name.split("/")[-1].replace(".parquet", "") for obj in objects]
    except Exception as e:
        print(f"Error listing objects for user_id {user_id}: {e}")
        return []


# Function to retrieve data for multiple user_id and timestamp pairs with specific fields
def retrieve_data(requests, fields=None, bucket_name="wl-data"):
    """
    Retrieve data from MinIO based on user_id, timestamp, and fields.
    """
    all_results = {}

    for request in requests:
        user_id = request.get("user_id")
        timestamp = request.get("timestamp")

        if not timestamp:
            timestamps = list_user_objects(user_id, bucket_name)
            if not timestamps:
                all_results[user_id] = {"error": "No data available for this user_id"}
                continue
        else:
            timestamps = [timestamp]

        for ts in timestamps:
            object_name = f"{user_id}/{ts}.parquet"
            try:
                response = minio_client.get_object(bucket_name, object_name)
                data = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")

                results = {}
                for field in fields:
                    field_found = False
                    for column in data.columns:
                        try:
                            field_data = data[column].iloc[0]
                            if isinstance(field_data, str):
                                field_data = json.loads(field_data)

                            if isinstance(field_data, dict) and field in field_data:
                                results[field] = field_data[field]
                                field_found = True
                                break
                        except Exception as e:
                            print(f"Error processing field {field}: {e}")

                    if not field_found:
                        results[field] = f"No data found for '{field}'"

                all_results[f"{user_id}_{ts}"] = {
                    "user_id": user_id,
                    "timestamp": ts,
                    "data": results if results else "No data found for requested fields",
                }
            except Exception as e:
                print(f"Error retrieving object {object_name}: {e}")
                all_results[f"{user_id}_{ts}"] = f"Error: Unable to retrieve data for {object_name}"

    return all_results if all_results else None


# Main function to handle query and return results
def handle_query(query_list):
    """
    Main handler for the query. Parses the query and retrieves the requested data.
    """
    requests, fields_to_retrieve = parse_query_list(query_list)
    if requests and fields_to_retrieve:
        results = retrieve_data(requests, fields_to_retrieve)
        return results
    else:
        print("Invalid query format or missing data in the query list.")
        return None


# Test cases
def run_tests():
    """
    Run a series of test cases to validate the functionality.
    """
    test_cases = [
        {"description": "Query without timestamp (retrieve all timestamps)", "query_list": ["02:00:00:00:00:00", "GPS"]},
        {"description": "Multiple keys in query", "query_list": ["02:00:00:00:00:00", "2024-10-22 18:16:22", "GPS", "rssi"]},
        {"description": "Only keys provided (no timestamps)", "query_list": ["02:00:00:00:00:00", "GPS", "rssi"]},
        {"description": "Invalid timestamp format", "query_list": ["02:00:00:00:00:00", "2024-10-22 18:16", "GPS"]},
        {"description": "Non-existent timestamp", "query_list": ["02:00:00:00:00:00", "2024-10-22 18:16:59", "GPS"]},
        {"description": "Empty query list", "query_list": []},
        {"description": "No field provided, only timestamps", "query_list": ["02:00:00:00:00:00", "2024-10-22 18:16:22"]},
    ]

    for i, test_case in enumerate(test_cases, 1):
        print(f"\nRunning Test Case {i}: {test_case['description']}")
        result = handle_query(test_case["query_list"])
        print("Query Result:", json.dumps(result, indent=4))


if __name__ == "__main__":
    print("1. Run Example Query")
    print("2. Run All Tests")
    choice = input("Enter your choice: ")

    if choice == "1":
        query_list = ["02:00:00:00:00:00", "2024-10-22 18:16:22.530491", "GPS", "rssi"]
        result = handle_query(query_list)
        print("Example Query Result:", json.dumps(result, indent=4))
    elif choice == "2":
        run_tests()
    else:
        print("Invalid choice. Exiting.")