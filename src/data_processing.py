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

# Suppress all warnings
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# * ------------------------------------------------------------------------------------------------ FEATURES RUNS HERE -------------------------------------------------------------------------------------------------

# * Feature 1: Retrieve all parquet files from MinIO and then add a new column to each DataFrame. 
def add_new_data(key, value, user_id, bucket_name="wl-data"):
    """Display the content of each object in the specified folder in MinIO."""
    try:
        # List all objects in the specified folder
        objects = minio_client.list_objects(bucket_name, prefix=user_id + "/", recursive=True)
        
        # Process each object in the folder
        for obj in objects:
            print(f"\nRetrieving and displaying content for object: {obj.object_name}")
            
            # Retrieve the object from MinIO
            response = minio_client.get_object(bucket_name, obj.object_name)
            
            try:
                # Read Parquet data into a DataFrame
                data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')
                
                # Add a new column of data in each DataFrame
                data[key] = [value] * len(data)
                print(f"Added new column '{key}' with value '{value}' to DataFrame.")

                print(data)

            except Exception as e:
                print(f"Error reading object {obj.object_name}: {str(e)}")
            
            finally:
                # Close the response to release resources
                response.close()
                response.release_conn()
                
    except Exception as e:
        print(f"Error listing objects in folder {user_id}: {str(e)}")

# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------

def main():
    key = "ground_truth"
    value = (-34.397, 150.644)
    user_id = "02:00:00:00:00:00"

    # * Call the function to add a new column of data in every parquet files in MinIO
    add_new_data(key, value, user_id)

if __name__ == "__main__":
    main()
