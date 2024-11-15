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

# Function to retrieve data for multiple user_id and timestamp pairs with specific fields
def retrieve_data(requests, fields=None, bucket_name="wl-data"):
    all_results = {}

    # Loop through each user_id and timestamp pair in the requests
    for request in requests:
        user_id = request.get("user_id")
        timestamp = request.get("timestamp")
        if not user_id or not timestamp:
            continue  # Skip if user_id or timestamp is missing

        try:
            # Define the object path based on user_id and timestamp
            object_name = f"{user_id}/{timestamp}.parquet"
            response = minio_client.get_object(bucket_name, object_name)

            # Read data and check for requested fields
            try:
                data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')
            except Exception as e:
                print(f"Error: Could not read object {object_name} as Parquet. Check if the file exists and is valid. Error: {e}")
                all_results[f"{user_id}_{timestamp}"] = "Error: Could not read data"
                continue

            # Collect data for each requested field
            results = {}
            for field in fields:
                field_found = False
                # Search each column for the field if it exists in the data
                for column in data.columns:
                    main_data = data[column].iloc[0]
                    # Convert to dictionary if stored as JSON string
                    if isinstance(main_data, str):
                        main_data = json.loads(main_data)
                    elif not isinstance(main_data, dict):
                        continue  # Skip columns that are not JSON-like

                    # Check if field is atomic (direct match) or complex (nested dictionary)
                    if field in main_data:
                        # If field is atomic, retrieve it directly
                        results[field] = main_data[field]
                        field_found = True
                        break
                    elif isinstance(main_data, dict) and field in main_data:
                        # If field is complex, add all nested fields to results
                        for sub_key, sub_value in main_data.items():
                            results[sub_key] = sub_value
                        field_found = True
                        break

                if not field_found:
                    results[field] = f"No data found for '{field}'"

            # Store results for this user_id and timestamp
            all_results[f"{user_id}_{timestamp}"] = results if results else "No data found for requested fields"

        except Exception as e:
            print(f"Error retrieving data for {user_id} at {timestamp}: {str(e)}")
            all_results[f"{user_id}_{timestamp}"] = "Error: Unable to retrieve data"

    return all_results if all_results else None

# Function to handle incoming MQTT messages and process multiple user_id/timestamp pairs
def on_message(client, userdata, message):
    try:
        # Load payload from the incoming MQTT message
        payload = json.loads(message.payload.decode('utf-8'))
        requests = payload.get("requests")  # Expecting [{"user_id": "example_user1", "timestamp": "2024-10-07T12:40:00"}, ...]
        fields_to_retrieve = payload.get("fields")  # Expecting fields in the form ["csi_imag", "nss", ...]

        # Validate required fields
        if requests and fields_to_retrieve:
            # Retrieve data for the specified requests and fields
            results = retrieve_data(requests, fields_to_retrieve)

            # Convert results to JSON format for MQTT response
            if results is not None:
                response_data = json.dumps(results)
            else:
                response_data = "No data found for requested fields."

            # Publish response back to MQTT
            client.publish(MQTT_OUTPUT_TOPIC, response_data)
            print("Sent data for requested user_id/timestamp pairs and fields.")
        else:
            print("Invalid request: requests and fields are required in the payload.")
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

# Main function to start the MQTT client
def main():
    setup_mqtt_client()
    # Keep the program running
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        print("Stopping MQTT client.")
        mqtt.Client().loop_stop()

if __name__ == "__main__":
    main()