import json
import urllib3
import pandas as pd
import paho.mqtt.client as mqtt
from io import BytesIO
from minio import Minio

# Configure MinIO Client
minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",      # MinIO access key
    secret_key="password",   # MinIO secret key
    secure=False             # Set to True if using HTTPS
)

# Configuration for MQTT broker
MQTT_BROKER = "128.205.218.189"  # Replace with your MQTT broker address
MQTT_PORT = 1883                   # Default MQTT port
MQTT_INPUT_TOPIC = "test/topic"    # Topic to receive data
MQTT_OUTPUT_TOPIC = "coordinate/topic"  # Topic to send data

# Suppress all warnings
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# * Feature 1: Retrieve parquet files from MinIO based on user_id and timestamp sent from MQTT.
def read_parquet_data(user_id, timestamp, bucket_name="wl-data"):
    """Retrieve the data in MinIO with specific user_id and timestamp."""
    object_name = f"{user_id}/{timestamp}.parquet"
    print(f"\nAttempting to retrieve object: {object_name} from bucket: {bucket_name}")

    try:
        # Retrieve the object from MinIO
        response = minio_client.get_object(bucket_name, object_name)
        print("Object retrieved successfully:\n")
        
        # Read Parquet data into a DataFrame
        data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')

        # Check the structure of the DataFrame
        print(data.head())  # This will help you debug the structure
        
        # Return the retrieved data
        return data
    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print("Please check if the object exists and ensure the User ID and Timestamp are correct.")
        print("-------------------------------------------------------------------------------------")
        return None

# * Feature 2: Display the content of the parquet files in MinIO based on user_id.
def display_content(bucket_name, folder):
    """Display the content of each object in the specified folder in MinIO."""
    try:
        # List all objects in the specified folder
        objects = minio_client.list_objects(bucket_name, prefix=folder + "/", recursive=True)
        
        # Process each object in the folder
        for obj in objects:
            print(f"\nRetrieving and displaying content for object: {obj.object_name}")
            
            # Retrieve the object from MinIO
            response = minio_client.get_object(bucket_name, obj.object_name)
            
            try:
                # Read Parquet data into a DataFrame
                data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')
                
                # Display the content of the DataFrame
                print(data)
            except Exception as e:
                print(f"Error reading object {obj.object_name}: {str(e)}")
            finally:
                # Close the response to release resources
                response.close()
                response.release_conn()
                
    except Exception as e:
        print(f"Error listing objects in folder {folder}: {str(e)}")

# * Feature 3: Get live user_id and timestamp from MQTT
# Feature 3: Get live user_id and optional timestamp from MQTT
def on_message(client, userdata, message):
    try:
        # Decode the message payload
        payload = message.payload.decode('utf-8')
        print(f"Received message:\n{payload}")  # Log the message

        # Manually parse the payload to extract user_id and optional timestamp
        lines = payload.splitlines()
        user_id = None
        timestamp = None

        for line in lines:
            if line.startswith("macAddress:"):
                user_id = line.split(": ")[1].strip()
            elif line.startswith("timestamp:"):
                # timestamp = line.split(": ")[1].strip()
                timestamp = None

        if user_id:
            if timestamp is not None:
                # Case 1: Both user_id and timestamp are provided
                print("Receive userId and timestamp")
                print(f"Retrieving data for user_id: {user_id} and timestamp: {timestamp}")
                retrieved_data = read_parquet_data(user_id, timestamp)
            else:
                # Case 2: Only user_id is provided
                print(f"Retrieving all data for user_id: {user_id}")
                retrieved_data = display_content("wl-data", user_id)  # No timestamp provided

            if retrieved_data is not None:
                # Send the retrieved data back to MQTT
                send_data_to_mqtt(retrieved_data)

    except Exception as e:
        print(f"Error processing message: {str(e)}")

# * Feature 4: Send data back to MQTT
def send_data_to_mqtt(data, broker=MQTT_BROKER, port=MQTT_PORT, topic=MQTT_OUTPUT_TOPIC):
    """Send GPS data to the specified MQTT topic."""
    # Check if the data is empty or not in the expected format
    if data is not None and not data.empty:
        gps_coordinates_pairs = []

        for entry in data.to_dict(orient='records'):
            # Extract latitude and longitude from the GPS column
            try:
                # Check if GPS is a dict or a JSON string
                gps_info = entry["GPS"]
                if isinstance(gps_info, str):
                    gps_info = json.loads(gps_info)  # Load the GPS data as a dictionary

                if isinstance(gps_info, dict) and "latitude" in gps_info and "longitude" in gps_info:
                    gps_coordinates_pairs.append((gps_info["latitude"], gps_info["longitude"]))
            except json.JSONDecodeError:
                print("Error decoding JSON in GPS column for entry:", entry)
            except Exception as e:
                print(f"Error processing entry: {e}")

        if gps_coordinates_pairs:
            # Convert the GPS data to JSON format for MQTT
            data_payload = json.dumps(gps_coordinates_pairs)

            # Initialize the MQTT client
            client = mqtt.Client()
            
            try:
                # Connect to the MQTT broker
                client.connect(broker, port)
                
                # Publish data to the specified MQTT topic
                result = client.publish(topic, data_payload)
                
                # Check if the publish was successful
                if result.rc == mqtt.MQTT_ERR_SUCCESS:
                    print(f"\nData published to MQTT topic '{topic}' successfully.")
                    print("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
                else:
                    print(f"Failed to publish data to MQTT topic '{topic}'. Error code: {result.rc}\n")
                    print("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
            
            except Exception as e:
                print(f"Error sending data to MQTT: {e}")
            
            finally:
                # Disconnect from the MQTT broker
                client.disconnect()
        else:
            print("No valid GPS coordinates found to send to MQTT.")
    else:
        print("No data to send to MQTT.")

# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------

def main():
    # Create an MQTT client
    client = mqtt.Client()

    # Assign the on_message callback
    client.on_message = on_message

    try:
        # Connect to the MQTT broker
        client.connect(MQTT_BROKER, MQTT_PORT)

        # Subscribe to the input topic
        client.subscribe(MQTT_INPUT_TOPIC)

        # Start the loop to listen for messages
        client.loop_start()
        print(f"Listening for messages on topic: {MQTT_INPUT_TOPIC}\n")

        # Keep the function running to listen for messages
        while True:
            pass  # Keep running until interrupted

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()