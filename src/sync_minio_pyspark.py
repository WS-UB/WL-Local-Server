import json
import urllib3
from minio import Minio
from pyspark.sql import SparkSession

# Configure MinIO Client
minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",      # MinIO access key
    secret_key="password",   # MinIO secret key
    secure=False             # Set to True if using HTTPS
)

# Supress all warnings
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# * -------------------------------------------------------------------------------------------- START BUILDING FEATURES HERE --------------------------------------------------------------------------------------------
# * Feature 1: Retrieve data from MinIO and index in Elasticsearch 
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

# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------
# !! This is the main function that runs the program
def main():
    # Create a Spark session
    spark = SparkSession.builder.appName("SyncMinIO").getOrCreate()

    # Retrieve the JSON data from MinIO
    user_id, timestamp, json_data = retrieve_data_from_minio()

    # Create a DataFrame from the inputted JSON data
    exampleDF = create_dataframe_from_json(spark, json_data)

    # Show the DataFrame
    exampleDF.show()

if __name__ == "__main__":
    main()