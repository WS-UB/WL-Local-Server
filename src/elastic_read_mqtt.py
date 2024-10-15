import json
import urllib3
import paho.mqtt.client as mqtt
from elasticsearch import Elasticsearch


# Define Elasticsearch connection
es = Elasticsearch(
    "https://128.205.218.189:9200", 
    basic_auth=("elastic", "mwLUsUm3IJd=ljk8Sq7P"), 
    verify_certs=False  # Disable certificate verification
)

# Supress all warnings
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# * -------------------------------------------------------------------------------------------- START BUILDING FEATURES HERE --------------------------------------------------------------------------------------------
# * Function 1: Define the MQTT callback functions
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe("test/topic")  # Subscribe to the desired MQTT topic

# * Function 2: Get the message from MQTT broker, parse the message into a query, and send the query to Elasticsearch
def on_message(client, userdata, message):
    print(f"Received message on topic: {message.topic}")

    # Decode the message payload and print it
    message_payload = message.payload.decode()
    print(f"Raw message payload: {message_payload}")

    # Assuming the payload is a string, we can still log it and prepare for Elasticsearch
    data = parse_string_to_query(message_payload)
    if data:
        print(f"Parsed data: {data}")  # Print the parsed data
        send_to_elasticsearch(data)
    else:
        print("No valid data to send to Elasticsearch.")

# * Function 3: Parse the string payload into a query
# ! Note that this function is only appliciable for parsing accelerometer and gyroscope data
def parse_string_to_query(input_string):
    """ Parse the string payload into a dictionary. """
    data_dict = {}

    # Step 1: Parse the input string
    for line in input_string.strip().splitlines():
        # Split the line into key and value
        try:
            key, value = line.split(": ")
            # Store the key-value pairs in the dictionary
            data_dict[key] = [float(v) for v in value.split(", ")]
        except ValueError as e:
            print(f"Error processing line '{line}': {e}")

    print(f"Data dictionary: {data_dict}")

    # Step 2: Construct the IMU dictionary
    imu_data = {}

    # Step 3: Add key-value pairs directly to the IMU dictionary
    for key, values in data_dict.items():
        imu_data[key] = values  # Assign the values to the key

    return {"IMU": imu_data}  # Return as a dictionary

# * Function 4: Send the queried to Elasticsearch
def send_to_elasticsearch(data):
    index_name = 'mqtt-data'  # Ensure this matches the index you're querying
    try:
        # Create or update the document in Elasticsearch
        response = es.index(index=index_name, document=data)
        print(f"Data has been sent to Elasticsearch!: {response['_id']}")
        print(f"ID of the sent data: {response['_id']}")  # Print the document ID
    except Exception as e:
        print(f"Failed to send data to Elasticsearch: {e}")

# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------
# !! This is the main function that runs the program
def main():
    # Set up the MQTT client
    client = mqtt.Client()

    # Assign the callback functions
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to the MQTT broker
    mqtt_broker_address = "128.205.218.189"  # Replace with your MQTT broker address
    client.connect(mqtt_broker_address, 1883)

    # Start the MQTT client loop
    client.loop_start()

    # Keep the script running
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Disconnecting from MQTT broker...")
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
