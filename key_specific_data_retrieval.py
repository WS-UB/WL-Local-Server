import json
import pandas as pd
from io import BytesIO
from minio import Minio
import paho.mqtt.client as mqtt

# MinIO Client Configuration
minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",      # MinIO access key
    secret_key="password",   # MinIO secret key
    secure=False             # Set to True if using HTTPS
)

# MQTT Broker Configuration
MQTT_BROKER = "128.205.218.189"  # Replace with your MQTT broker address
MQTT_PORT = 1883
MQTT_INPUT_TOPIC = "data/request"
MQTT_OUTPUT_TOPIC = "data/response"

# Available keys and their sub-keys for display
available_keys = {
    "IMU": ["gyro", "accel"],
    "GPS": ["latitude", "longitude"],
    "WiFi": ["csi_imag", "csi_real", "rssi", "ap_id"],
    "Channel": ["chan", "channel", "bw", "nss", "ntx", "nrx", "mcs"]
}

# Function to retrieve specific data based on user_id, timestamp, key, and optional sub_key
def retrieve_data(user_id=None, timestamp=None, key=None, sub_key=None, bucket_name="wl-data"):
    try:
        # Case 1: Only user_id provided (no timestamp)
        if user_id and not timestamp:
            # Retrieve all objects for the specified user_id
            objects = minio_client.list_objects(bucket_name, prefix=f"{user_id}/", recursive=True)
            results = []
            for obj in objects:
                # Check if object exists and is readable as a Parquet file
                response = minio_client.get_object(bucket_name, obj.object_name)
                try:
                    data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')
                except Exception as e:
                    print(f"Warning: Could not read object {obj.object_name} as Parquet. Skipping. Error: {e}")
                    continue
                
                # Check if the key exists in the data and sub_key within the JSON/dictionary
                if key in data.columns:
                    # Check if the data is a JSON string or a dictionary
                    main_data = data[key].iloc[0]
                    if isinstance(main_data, str):
                        main_data = json.loads(main_data)
                    elif not isinstance(main_data, dict):
                        print(f"Warning: Unexpected data format in '{key}' column. Skipping this object.")
                        continue
                    
                    if sub_key in main_data:
                        timestamp = obj.object_name.split("/")[-1].replace(".parquet", "")
                        results.append({ "Timestamp": timestamp, sub_key: main_data[sub_key] })

            # Return collected data for all timestamps
            return pd.DataFrame(results) if results else None

        # Case 2: No user_id or timestamp provided (only key and sub_key)
        elif not user_id and not timestamp and key and sub_key:
            # Retrieve all objects in the bucket
            objects = minio_client.list_objects(bucket_name, recursive=True)
            results = []
            for obj in objects:
                response = minio_client.get_object(bucket_name, obj.object_name)
                try:
                    data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')
                except Exception as e:
                    print(f"Warning: Could not read object {obj.object_name} as Parquet. Skipping. Error: {e}")
                    continue
                
                if key in data.columns:
                    main_data = data[key].iloc[0]
                    if isinstance(main_data, str):
                        main_data = json.loads(main_data)
                    elif not isinstance(main_data, dict):
                        print(f"Warning: Unexpected data format in '{key}' column. Skipping this object.")
                        continue
                    
                    if sub_key in main_data:
                        user_id = obj.object_name.split("/")[0]
                        timestamp = obj.object_name.split("/")[-1].replace(".parquet", "")
                        results.append({ "UserId": user_id, "Timestamp": timestamp, sub_key: main_data[sub_key] })
            
            # Return collected data for all user IDs and timestamps
            return pd.DataFrame(results) if results else None

        # Original case: Both user_id and timestamp provided
        elif user_id and timestamp:
            object_name = f"{user_id}/{timestamp}.parquet"
            response = minio_client.get_object(bucket_name, object_name)
            try:
                data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')
            except Exception as e:
                print(f"Error: Could not read object {object_name} as Parquet. Check if the file exists and is valid. Error: {e}")
                return None
            
            if key in data.columns:
                main_data = data[key].iloc[0]
                if isinstance(main_data, str):
                    main_data = json.loads(main_data)
                elif not isinstance(main_data, dict):
                    print(f"Warning: Unexpected data format in '{key}' column.")
                    return None
                
                if sub_key in main_data:
                    return pd.DataFrame({sub_key: [main_data[sub_key]]})
                else:
                    print(f"Sub-key '{sub_key}' not found in '{key}' data.")
                    return None
            else:
                print(f"Key '{key}' not found in the data.")
                return None

    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        return None


# Function to handle incoming MQTT messages
def on_message(client, userdata, message):
    try:
        payload = json.loads(message.payload.decode('utf-8'))
        user_id = payload.get("user_id")
        timestamp = payload.get("timestamp")
        key_to_retrieve = payload.get("key")
        sub_key_to_retrieve = payload.get("sub_key")

        if user_id:
            data = retrieve_data(user_id, timestamp, key_to_retrieve, sub_key_to_retrieve)

            if data is not None:
                # Convert data to JSON format
                response_data = data.to_json(orient='records')
            else:
                response_data = f"Requested sub-key '{sub_key_to_retrieve}' not found in key '{key_to_retrieve}'."

            # Publish response back to MQTT
            client.publish(MQTT_OUTPUT_TOPIC, response_data)
            print(f"Sent data for user_id {user_id}, timestamp {timestamp} for key {key_to_retrieve} and sub-key {sub_key_to_retrieve}.")
        else:
            print("User ID not provided in the message.")
    except Exception as e:
        print(f"Error processing message: {str(e)}")

# Initialize MQTT Client and configure message handling
def setup_mqtt_client():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT)
    client.subscribe(MQTT_INPUT_TOPIC)
    client.loop_start()
    print(f"Listening on topic '{MQTT_INPUT_TOPIC}' for data requests...")

    return client

def list_objects_in_bucket(bucket_name="wl-data"):
    print(f"\nListing objects in bucket: {bucket_name}")
    objects = minio_client.list_objects(bucket_name, recursive=True)
    for obj in objects:
        print(f" - {obj.object_name}")

def retrieve_data_from_minio():
    user_id = input("Please enter the User ID (or press Enter to skip): ")
    timestamp = input("Please enter the Timestamp (format: YYYY-MM-DDTHH:MM:SS, or press Enter to skip): ")

    # Display available keys and prompt for key
    print("\nAvailable Keys:", list(available_keys.keys()))
    key = input("Enter the main key (e.g., IMU, GPS, WiFi): ")
    
    if key not in available_keys:
        print(f"Invalid key. Choose from {list(available_keys.keys())}")
        return

    # Display available sub-keys for the chosen key
    print(f"\nAvailable sub-keys for '{key}': {available_keys[key]}")
    sub_key = input(f"Enter the sub-key for '{key}' (e.g., gyro for IMU): ")
    
    if sub_key not in available_keys[key]:
        print(f"Invalid sub-key. Choose from {available_keys[key]}")
        return

    # Retrieve and display the data
    result = retrieve_data(user_id if user_id else None, timestamp if timestamp else None, key, sub_key)
    if result is not None:
        print("\nRetrieved Data:")
        print(result.to_string(index=False))  # Suppress index in output
    else:
        print("No data found or error in retrieval.")

# Main function with menu options
def main():
    while True:
        print("\nOptions:")
        print("1: List all stored objects.")
        print("2: Retrieve data by entering User ID, Timestamp, Key, and Sub-key.")
        print("3: Exit.")
        choice = input("Enter your choice: ")

        if choice == "1":
            list_objects_in_bucket()
        elif choice == "2":
            retrieve_data_from_minio()
        elif choice == "3":
            print("Exiting the program.")
            break
        else:
            print("Invalid choice. Please select a valid option.")

if __name__ == "__main__":
    main()
