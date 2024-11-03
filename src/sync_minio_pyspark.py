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

# * Feature 1: Retrieve data from MinIO 
def get_data_from_minio(bucket_name, user_id, timestamp):
    """Retrieve and return data from MinIO using User ID and Timestamp."""
    object_name = f"{user_id}/{timestamp}.parquet"
    print(f"Attempting to retrieve object: {object_name} from bucket: {bucket_name}")

    try:
        # Retrieve the object from MinIO
        response = minio_client.get_object(bucket_name, object_name)
        print("Object retrieved successfully.")

        # Read Parquet data into a DataFrame
        data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')

        # Display the retrieved data
        print(f"Data for User ID: {user_id} at {timestamp}:")
        
        return data

    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        return None

# * Feature 2: Retrieve data from MinIO with user_id and timestamp
def read_parquet_data(user_id, timestamp, bucket_name="wl-data"):
    """Retrieve the data in MinIO with specific user_id and timestamp."""
    object_name = f"{user_id}/{timestamp}.parquet"
    print(f"Attempting to retrieve object: {object_name} from bucket: {bucket_name}")

    try:
        # Retrieve the object from MinIO
        response = minio_client.get_object(bucket_name, object_name)
        print("Object retrieved successfully.")
        
        # Read Parquet data into a DataFrame
        data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')

        # Check the structure of the DataFrame
        print(data.head())  # This will help you debug the structure
        
        # Return the retrieved data
        return data
    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print("Please check if the object exists and ensure the User ID and Timestamp are correct.")
        return None

# * Feature 3: Get live user_id and timestamp from MQTT
def on_message(client, userdata, message):
    try:
        # Decode the message payload
        payload = message.payload.decode('utf-8')
        print(f"Received message: {payload}")  # Log the message

        # Store the received data in MinIO
        store_received_data(payload)

        # Manually parse the payload to extract user_id and timestamp
        # Assuming the format is:
        # macAddress: <user_id>
        # timestamp: <timestamp>
        lines = payload.splitlines()
        user_id = None
        timestamp = None

        for line in lines:
            if line.startswith("macAddress:"):
                user_id = line.split(": ")[1].strip()
            elif line.startswith("timestamp:"):
                timestamp = line.split(": ")[1].strip()

        if user_id and timestamp:
            # Retrieve data from MinIO
            retrieved_data = read_parquet_data(user_id, timestamp)

            if retrieved_data is not None:
                # Send the retrieved data back to MQTT
                send_data_to_mqtt(retrieved_data)

    except Exception as e:
        print(f"Error processing message: {str(e)}")

# * Feature 4: Send data back to MQTT
def send_data_to_mqtt(data, broker=MQTT_BROKER, port=MQTT_PORT, topic=MQTT_OUTPUT_TOPIC):
    """Send data to the specified MQTT topic."""
    # Check if the data is empty or not in the expected format
    if data is not None and not data.empty:
        gps_coordinates_pairs = []

        for entry in data.to_dict(orient='records'):
            # Extract latitude and longitude from the GPS column
            gps_info = json.loads(entry["GPS"])  # Load the GPS data as a dictionary
            if isinstance(gps_info, dict) and "latitude" in gps_info and "longitude" in gps_info:
                gps_coordinates_pairs.append((gps_info["latitude"], gps_info["longitude"]))

        if gps_coordinates_pairs:
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
                    print(f"Data published to MQTT topic '{topic}' successfully.")
                else:
                    print(f"Failed to publish data to MQTT topic '{topic}'. Error code: {result.rc}")
            
            except Exception as e:
                print(f"Error sending data to MQTT: {e}")
            
            finally:
                # Disconnect from the MQTT broker
                client.disconnect()
        else:
            print("No valid GPS coordinates found to send to MQTT.")
    else:
        print("No data to send to MQTT.")

def store_received_data(received_data, bucket_name="wl-data"):
    """Parses the received data and stores it in MinIO in Parquet format with User ID and Timestamp."""
    print("Storing received data...")
    try:
        # Initialize the variables
        user_id = None
        timestamp = None
        imu_data = {}
        gps_data = {}

        # Split the received data into lines and extract values
        for line in received_data.splitlines():
            if line.startswith("macAddress:"):
                user_id = line.split(": ")[1].strip()
            elif line.startswith("timestamp:"):
                timestamp = line.split(": ")[1].strip()
            elif line.startswith("gyro:"):
                # Assuming gyro data is a list
                imu_data['gyro'] = [float(value) for value in line.split(": ")[1].strip().split(",")]
            elif line.startswith("accel:"):
                # Assuming accel data is a list
                imu_data['accel'] = [float(value) for value in line.split(": ")[1].strip().split(",")]
            elif line.startswith("GPS:"):
                # Split the GPS coordinates
                lat_long = line.split(": ")[1].strip().split(", ")
                gps_data = {
                    "latitude": float(lat_long[0]),
                    "longitude": float(lat_long[1])
                }

        # If user_id or timestamp is not found, use defaults
        user_id = user_id or "unknown_user"
        timestamp = timestamp or datetime.utcnow().isoformat()

        # Convert the data into a DataFrame
        data_entry = {
            "user_id": [user_id],
            "timestamp": [timestamp],
            "IMU": [json.dumps(imu_data)],
            "GPS": [json.dumps(gps_data)],
        }
        df = pd.DataFrame(data_entry)

        # Save DataFrame as Parquet in memory
        parquet_buffer = BytesIO()
        df.to_parquet(parquet_buffer, engine='pyarrow', index=False)
        parquet_buffer.seek(0)

        # Define the object name using User ID and Timestamp in the format "UserId/Timestamp"
        object_name = f"{user_id}/{timestamp}.parquet"

        # Store the data in MinIO
        minio_client.put_object(
            bucket_name,
            object_name,
            parquet_buffer,
            length=parquet_buffer.getbuffer().nbytes,
            content_type="application/x-parquet"
        )
        print(f"Successfully stored data for {user_id} at {timestamp} in Parquet format.")
    except Exception as e:
        print(f"Error storing data in MinIO: {str(e)}")

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
        print(f"Listening for messages on topic: {MQTT_INPUT_TOPIC}")

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
