import urllib3
import paho.mqtt.client as mqtt
from pyspark.sql import SparkSession

# Supress all warnings
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# Create a list to hold the data dictionaries
data_list = []

# Create a Spark session
spark = SparkSession.builder.appName("MQTT to DataFrame").getOrCreate()

# Global variable to hold the DataFrame
global_df = None


# * -------------------------------------------------------------------------------------------- START BUILDING FEATURES HERE --------------------------------------------------------------------------------------------
# * Function 1: Define the MQTT callback functions
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe("test/topic")  # Subscribe to the desired MQTT topic

# * Function 2: Get the message from MQTT broker, parse the message into a query, and create a dataframe out of the query.
def on_message(client, userdata, message):
    # Reference the global DataFrame
    global global_df

    # Decode the message payload and print it
    message_payload = message.payload.decode()
    print(f"\n\nRaw message payload:\n----------------------------------\n{message_payload}")

    # Parse the payload into a query and append to the list
    query_data = parse_string_to_query(message_payload)
    if query_data:
        data_list.append(query_data)  # Append the dictionary to the list

    # Create DataFrame in batches of 10 messages
    if len(data_list) >= 10:  # Example: convert to DataFrame every 10 messages
        # Create a DataFrame from the list of dictionaries
        df = create_dataframe_from_list(spark, data_list)
        print("\nDataFrame created from collected messages:")
        
        # Display the DataFrame
        df.show()  
        
        # Update the global DataFrame
        global_df = df
        
        # Clear the list after processing
        data_list.clear()


# * Function 3: Parse the string payload into a query
def parse_string_to_query(input_string):
    """Parse the string payload into a dictionary."""
    data_dict = {}

    # Step 1: Parse the input string
    for line in input_string.strip().splitlines():
        # Split the line into key and value
        try:
            key, value = line.split(": ")
            # Store the key-value pairs in the dictionary
            if key == "timestamp":
                value = value[:19].replace(" ", "T")
                data_dict[key] = value
            elif key == "macAddress" or key == "server_id" or key == "user_id" or key == "password" or key == "comment":
                data_dict[key] = value
            else:
                data_dict[key] = [float(v) for v in value.split(", ")]
        except ValueError as e:
            print(f"Error processing line '{line}': {e}")

    # Return the dictionary
    return data_dict  # Return as a dictionary

# * Function 4: Create a DataFrame from a list of dictionaries
def create_dataframe_from_list(spark, data_list):
    """Create a DataFrame from a list of dictionaries."""
    # Create a DataFrame from the list of dictionaries
    df = spark.createDataFrame(data_list)

    # Return the PySpark DataFrame
    return df

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
