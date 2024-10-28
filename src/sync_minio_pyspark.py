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
def create_parquet_from_json(spark, json_data, parquet_path):
    """Create a DataFrame from JSON data (string/dictionary) and save it as Parquet."""
    
    # If input is a JSON string, parse it into a Python dictionary
    if isinstance(json_data, str):
        json_data = json.loads(json_data)  # Convert JSON string to dictionary

    # If the JSON is a single dictionary, wrap it in a list to create a DataFrame
    if isinstance(json_data, dict):
        json_data = [json_data]

    # Create DataFrame from JSON data
    df = spark.createDataFrame(json_data)

    # Write the DataFrame to Parquet format
    df.write.parquet(parquet_path)
    print(f"Data saved to Parquet at: {parquet_path}")

    # Return the DataFrame for further use if needed
    return df

# * Feature 3: Upload Parquet file back to MinIO
def upload_to_minio(local_file_path, bucket_name, object_name):
    """Upload a local file to MinIO."""
    try:
        # Upload the Parquet file to the specified bucket
        minio_client.fput_object(
            bucket_name, object_name, local_file_path
        )
        print(f"Uploaded {local_file_path} to {bucket_name}/{object_name}")
    except Exception as e:
        print(f"Error uploading file: {str(e)}")


# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------
# !! This is the main function that runs the program
def main():
    # Create a Spark session
    spark = SparkSession.builder.appName("SyncMinIO").getOrCreate()

    # Retrieve the JSON data from MinIO
    json_data = []
    user_id, timestamp, json_data = retrieve_data_from_minio()

        # Generate Parquet filename and path
    parquet_file_name = f"{timestamp.replace(':', '-')}.parquet"
    local_parquet_path = f"./{parquet_file_name}"  # Save locally first

     # Create a DataFrame and save as Parquet
    exampleDF = create_parquet_from_json(spark, json_data, local_parquet_path)

    # Show the DataFrame
    exampleDF.show()

    # Define MinIO object path under 'user123/' folder
    object_name = f"user123/{parquet_file_name}"

    # Upload Parquet file to MinIO
    upload_to_minio(local_parquet_path, "wl-data", object_name)

if __name__ == "__main__":
    main()