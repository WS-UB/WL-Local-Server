import json
import urllib3
from minio import Minio
import paho.mqtt.client as mqtt
from pyspark.sql import SparkSession
from io import BytesIO
import pandas as pd
import pyarrow.parquet as pq

# Configure MinIO Client
minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",      # MinIO access key
    secret_key="password",   # MinIO secret key
    secure=False             # Set to True if using HTTPS
)

# Configuration for MQTT broker
MQTT_BROKER = "128.205.218.189"  # Replace with your MQTT broker address
MQTT_PORT = 1883                     # Default MQTT port
MQTT_TOPIC = "test/topic"             # Topic to publish data

# Supress all warnings
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# * -------------------------------------------------------------------------------------------- START BUILDING FEATURES HERE --------------------------------------------------------------------------------------------
# * Feature 1: Retrieve data from MinIO
# TODO: Make sure to create an input for the user_id and timestamp. 
# * This function will retrieve the data from MinIO using the requested User ID and Timestamp
def read_parquet_data(bucket_name="wl-data"):
    """Retrieve the data in MinIO with specific user_id and timestamp."""
    
    # ! Use one specific User ID and Timestamp for demonstration
    user_id = "02:00:00:00:00:00"
    timestamp = "2024-10-22 18:16:22.530491"
    
    try:
        # Retrieve the dataframe from MinIO
        response = get_data_from_minio(bucket_name, user_id, timestamp)
        
        # Get the GPS data from the retrieved dataframe
        gps_data = response["GPS"]

        return gps_data 
        
    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print("Please check if the object exists in the bucket and ensure the User ID and Timestamp are correct.")


# * Feature 2: Send data back to MQTT
def send_data_to_mqtt(data, broker=MQTT_BROKER, port=MQTT_PORT, topic=MQTT_TOPIC):
    """Send data to the specified MQTT topic."""

    # Initialize the list to store GPS coordinates
    gps_coordinates_pairs = [
        (gps_entry["latitude"], gps_entry["longitude"])
        for gps_entry in data if isinstance(gps_entry, dict)
    ]
        
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


# * Feature 3: Retrieve data from MinIO (Thanks to the function that is provided in minio_script.py) 
def get_data_from_minio(bucket_name, user_id, timestamp):
    """Retrieve and print the data from MinIO using User ID and Timestamp."""

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
        
        # Return the retrieved data
        return data
    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print("Please check if the object exists and ensure the User ID and Timestamp are correct.")

# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------
# !! This is the main function that runs the program
def main():
    # Create a Spark session
    spark = SparkSession.builder.appName("SyncMinIO").getOrCreate()

    # Retrieve the GPS data from MinIO
    retrieved_data = read_parquet_data()

    # Send data from MinIO back to MQTT. In this case, we send the GPS coordiantes back to the MQTT broker.
    send_data_to_mqtt(retrieved_data)

if __name__ == "__main__":
    main()