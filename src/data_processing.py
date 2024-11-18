import json
import urllib3
import pandas as pd
import paho.mqtt.client as mqtt
from io import BytesIO
from minio import Minio
from datetime import datetime
from key_specific_data_retrieval import handle_automated_query

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
"""
@params: key: The name of the new column
@params: value: The value of the new column
@params: data_list: The list of dataframes that we want to add the new column to

@return: A list of dataframes with the new column added
"""
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

# * Feature 2: Retrieve the name of parquet files from MinIO based on the given range of hour and minute.
"""
@params: bucket_name: The name of the bucket in MinIO
@params: user_id: The list of user_id that we want to retrieve the data from
@params: keys: The category of data that we want to retrieve from the parquet file
@params: start_hour: The starting hour of the range
@params: end_hour: The ending hour of the range
@params: start_minute: The starting minute of the range
@params: end_minute: The ending minute of the range

@return: A list where each element is a list of parquet files of each unique user ID that are within the specified hour and minute range. The last element of the list is the keys that we want to retrieve from the parquet file.
"""
def retrieve_hour_range_data(bucket_name, user_id, keys, start_hour, end_hour, start_minute, end_minute):
    # Check if the hour and minute are within the valid range
    if (start_hour >= 0 or start_hour <= 23) and (end_hour >= 0 or end_hour <= 23) and (start_hour <= end_hour):
        if (start_minute >= 0 or start_minute <= 59) and (end_minute >= 0 or end_minute <= 59) and (start_minute <= end_minute):
            # Create an empty list to store the filtered files
            file_names = []

            # Create an empty list to store the objects in the specified bucket and prefix
            objects = []
            
            # List objects in the specified bucket and prefix
            for user in user_id:
                objects.append(minio_client.list_objects(bucket_name, prefix=user + "/", recursive=True))

            
            # * The format of the file is: "YYYY-MM-DD HH:MM:SS.SSSSSS.parquet"
            # ! The index of the hour in the filename is from 29 to 31, and the minute is from 32 to 34.
            for user in objects:
                # For each unique user, we will create a list of files that are within the specified hour and minute range of that user
                user_files = []
                for obj in user:
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
                                user_files.append(obj.object_name[:-8])

                        # Throw an error if the file format is not parquet.
                        except ValueError:
                            print(f"Skipping file with unexpected format: {obj.object_name}")

                # Append the list of files for the user to the main list of files
                file_names.append(user_files)

                # We add the key to the last element of the list to request for certain column of data from the parquet file
                # ! We're testing in getting the GPS and rssi data from the parquet file for now.
                file_names = file_names + keys
                return file_names
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
    user_id = ["02:00:00:00:00:00"]
    keys = ["GPS", "rssi"]

    # * Obtain the list of dataframes for the specified hour and minute range
    file_names_list = retrieve_hour_range_data("wl-data", user_id, keys, 18, 18, 0, 30)
    # print(file_names_list)

    # * Retrieve the list of dataframes based on the provided file names
    data_list = handle_automated_query(file_names_list)

    add_new_data(key, value, data_list)

    print(data_list[0]["GPS"])

if __name__ == "__main__":
    main()
