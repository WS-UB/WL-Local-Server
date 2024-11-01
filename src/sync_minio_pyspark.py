import json
import urllib3
from minio import Minio
import paho.mqtt.client as mqtt
from pyspark.sql import SparkSession

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
def retrieve_data_from_minio(bucket_name="wl-data"):
    """Retrieve and print the data from MinIO using User ID and Timestamp."""
    
    # ! Use one specific User ID and Timestamp for demonstration
    user_id = "user2"
    timestamp = "2024-10-07T12:40:00"

    object_name = f"{user_id}/{timestamp}.json"
    print(f"Attempting to retrieve object: {object_name} from bucket: {bucket_name}")
    
    try:
        # Retrieve the object from MinIO
        response = minio_client.get_object(bucket_name, object_name)
        print("Object retrieved successfully.")
        
        # Read and decode the object data
        data = response.read().decode("utf-8")
        json_data = json.loads(data)
        
    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print("Please check if the object exists in the bucket and ensure the User ID and Timestamp are correct.")
    return user_id, timestamp, json_data

# * Feature 2: Use the newly created spark and the JSON data from MinIO to create a new dataframe.
def create_dataframe_from_json(spark, json_data):
    """Create a DataFrame from the JSON data."""

    # Create a DataFrame from the JSON data
    df = spark.createDataFrame([json_data])

    # Return the PySpark DataFrame
    return df

# * Feature 3: Send data back to MQTT
def send_data_to_mqtt(data, broker=MQTT_BROKER, port=MQTT_PORT, topic=MQTT_TOPIC):
    """Send JSON data to the specified MQTT topic."""
    
    # Convert the data to JSON format if it's not already
    if not isinstance(data, str):
        data = json.dumps(data)
    
    # Initialize the MQTT client
    client = mqtt.Client()
    
    try:
        # Connect to the MQTT broker
        client.connect(broker, port)
        
        # Publish data to the specified MQTT topic
        result = client.publish(topic, data)
        
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
    

# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------
# !! This is the main function that runs the program
def main():
    # Create a Spark session
    spark = SparkSession.builder.appName("SyncMinIO").getOrCreate()

    # Retrieve the JSON data from MinIO
    user_id, timestamp, json_data = retrieve_data_from_minio()

    # Send data from MinIO back to MQTT. In this case, we send the GPS coordiantes back to the MQTT broker.
    send_data_to_mqtt(json_data["GPS"])

    # Create a DataFrame from the inputted JSON data
    exampleDF = create_dataframe_from_json(spark, json_data)

    # Show the DataFrame
    exampleDF.show()

if __name__ == "__main__":
    main()