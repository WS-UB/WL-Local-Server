import json
import urllib3
import pandas as pd
import paho.mqtt.client as mqtt
from io import BytesIO
from minio import Minio
from datetime import datetime

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

# * Feature 1: Add a new column of data to a given list of dataframe. 
def add_new_data(key, value, data_list):
    """Display the content of each object in the specified folder in MinIO."""
    try:
        # Iterate through each DataFrame in the list annd add a new column with the specified key and value
        for data in data_list:
            # Add a new column to the DataFrame
            data[key] = [value] * len(data)
        print(data_list)
        return data_list
                
    except Exception as e:
        print(f"Error adding new data: {str(e)}")
        return None

# * Feature 2: Retrieve certain range of parquet files from MinIO based on the given range of hour and minute.
def retrieve_hour_range_data(bucket_name, user_id, start_hour, end_hour, start_minute, end_minute):
    # Check if the hour and minute are within the valid range
    if (start_hour >= 0 or start_hour <= 23) and (end_hour >= 0 or end_hour <= 23) and (start_hour <= end_hour):
        if (start_minute >= 0 or start_minute <= 59) and (end_minute >= 0 or end_minute <= 59) and (start_minute <= end_minute):
            # Create an empty list to store the filtered files
            filtered_files = []
            
            # List objects in the specified bucket and prefix
            objects = minio_client.list_objects(bucket_name, prefix=user_id + "/", recursive=True)

            # * The format of the file is: "YYYY-MM-DD HH:MM:SS.SSSSSS.parquet"
            # ! The index of the hour in the filename is from 29 to 31, and the minute is from 32 to 34.
            for obj in objects:
                # Filter out non-parquet files by checking the file extension of the object. All parquet files end with ".parquet", so that's 8 characters long
                if obj.object_name[-8:] == '.parquet':
                    # Extract the hour from the filename
                    try:
                        # Extract the datetime into a datetime object
                        file_datetime = datetime.strptime(obj.object_name[18:37], "%Y-%m-%d %H:%M:%S")

                        # Obtain the hour and minute from the datetime object
                        file_hour = file_datetime.hour
                        file_minute = file_datetime.minute
                        
                        # Check if file hour is within the specified range
                        if (start_hour <= file_hour <= end_hour) and (start_minute <= file_minute <= end_minute):
                            # Retrieve the object from MinIO
                            response = minio_client.get_object(bucket_name, obj.object_name)

                            # Read Parquet data into a DataFrame
                            try:
                                data = pd.read_parquet(BytesIO(response.read()), engine='pyarrow')
                                # Insert the DataFrame into the list of filtered files
                                filtered_files.append(data)
                            except Exception as e:
                                print(f"Error reading object {obj.object_name}: {str(e)}")
                    # Throw an error if the file format is not parquet.
                    except ValueError:
                        print(f"Skipping file with unexpected format: {obj.object_name}")
            return filtered_files
        else:
            print("Invalid minute range. Please enter a valid minute range.")
            return []
    else:
        print("Invalid hour range. Please enter a valid hour range.")
        return []

# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------

def main():
    key = "ground_truth"
    value = (-34.397, 150.644)
    user_id = "02:00:00:00:00:00"

    # * Obtain the list of dataframes for the specified hour and minute range
    data_list = retrieve_hour_range_data("wl-data", user_id, 18, 18, 0, 30)
    print("---------- RETRIEVED DATA FRAMES -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
    print(data_list)
    print("\n\n")

    # * Add a new column to the obtained list of dataframes
    print("---------- MODIFIED DATA FRAMES -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
    add_new_data(key, value, data_list)

if __name__ == "__main__":
    main()
