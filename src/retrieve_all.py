import json
from io import BytesIO
from minio import Minio
from datetime import datetime
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",  # MinIO access key
    secret_key="password",  # MinIO secret key
    secure=False,  # Set to True if using HTTPS
)


def retrieve_data_from_minio(bucket_name="wl-data"):
    """Retrieve and print the data from MinIO using User ID and Timestamp."""
    try:
        # Retrieve the object from MinIO
        folder = input("Name of data folder?: ")
        folder_prefix = f"{folder}/"  # Ensure it ends with '/'

        # List all files inside the folder
        objects = minio_client.list_objects(
            bucket_name, prefix=folder_prefix, recursive=True
        )

        for obj in objects:
            if obj.object_name.endswith(".parquet"):
                response = minio_client.get_object(bucket_name, obj.object_name)
                data = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
                json_data = data.to_json(orient="records", indent=4).replace("\\", "")
                print(f"{json_data}\n")

        # # Read Parquet data into a DataFrame
        # data = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
        # json_data = data.to_json(orient="records", indent=4)

        # Display the retrieved data
        # print(f"Data for User ID: {user_id} at {timestamp}:")
        # print(json_data)
    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print(
            "Please check if the object exists and ensure the User ID and Timestamp are correct."
        )


if __name__ == "__main__":
    retrieve_data_from_minio()
