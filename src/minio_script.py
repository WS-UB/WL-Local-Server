import json
from io import BytesIO
from minio import Minio
from datetime import datetime

# MinIO client setup
minio_client = Minio(
    "127.0.0.1:9000",  # Replace with your MinIO server address
    access_key="DiGqSmuc6oSFP89UvkHG",  # MinIO access key
    secret_key="KIe5dxtKd7N1easAU2F4nNApuzUTXxUoN0I9zrIp",  # MinIO secret key
    secure=False,  # Set to True if using HTTPS
)


def store_received_data(received_data, bucket_name="wl-data"):
    """Parses the received data and stores it in MinIO with User ID and Timestamp."""
    print("Storing received data...")
    try:
        # Parse the incoming data (assuming it's a JSON string)
        data = json.loads(received_data)
        print("Data parsed successfully!")

        # Ensure the bucket exists, create it if it doesn't
        if not minio_client.bucket_exists(bucket_name):
            minio_client.make_bucket(bucket_name)
            print(f"Bucket '{bucket_name}' created.")

        # Extract the User ID and Timestamp from the received data
        user_id = data.get("user_id", "unknown_user")
        timestamp = data.get("timestamp", datetime.utcnow().isoformat())

        # Convert the rest of the data into a JSON structure to store
        imu_data = data.get("IMU", {})
        gps_data = data.get("GPS", {})
        wifi_data = data.get("WiFi", {})
        channel_data = data.get("Channel", {})

        # Structure the data to store in MinIO
        data_entry = {
            "user_id": user_id,
            "timestamp": timestamp,
            "IMU": imu_data,
            "GPS": gps_data,
            "WiFi": wifi_data,
            "Channel": channel_data,
        }

        # Convert data to JSON format for storage
        json_data = json.dumps(data_entry)

        # Define the object name using User ID and Timestamp in the format "UserId/Timestamp"
        object_name = f"{user_id}/{timestamp}.json"

        # Store the data in MinIO
        minio_client.put_object(
            bucket_name,
            object_name,
            BytesIO(json_data.encode("utf-8")),
            len(json_data),
            content_type="application/json",
        )
        print(f"Successfully stored data for {user_id} at {timestamp} in MinIO.")
    except Exception as e:
        print(f"Error storing data in MinIO: {str(e)}")


def list_objects_in_bucket(bucket_name="wl-data"):
    """List all objects in the specified bucket to verify the data has been stored."""
    try:
        print(f"Listing objects in bucket: {bucket_name}")
        objects = minio_client.list_objects(
            bucket_name, recursive=True
        )  # Recursive=True to list all
        object_found = False
        print(f"Objects in bucket '{bucket_name}':")
        for obj in objects:
            object_found = True
            # Display the full object name (including the timestamp)
            print(f" - {obj.object_name}")
        if not object_found:
            print("No objects found in the bucket.")
    except Exception as e:
        print(f"Error listing objects: {str(e)}")


def retrieve_data_from_minio(bucket_name="wl-data"):
    """Retrieve and print the data from MinIO using User ID and Timestamp."""

    # Ask the user for input
    user_id = input("Please enter the User ID: ")
    timestamp = input("Please enter the Timestamp (format: YYYY-MM-DDTHH:MM:SS): ")

    # Construct the object name using User ID and Timestamp
    object_name = f"{user_id}/{timestamp}.json"
    print(f"Attempting to retrieve object: {object_name} from bucket: {bucket_name}")

    try:
        # Retrieve the object from MinIO
        response = minio_client.get_object(bucket_name, object_name)
        print("Object retrieved successfully.")

        # Read and decode the object data
        data = response.read().decode("utf-8")
        json_data = json.loads(data)

        # Display all the details for the respective user_id and timestamp
        print(f"Data for User ID: {user_id} at {timestamp}:")
        print(json.dumps(json_data, indent=4))

    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print(
            "Please check if the object exists in the bucket and ensure the User ID and Timestamp are correct."
        )


# Test cases for the code


def test_store_valid_data():
    print("\nRunning Test Case: Store Valid Data")
    test_data = json.dumps(
        {
            "user_id": "user1",
            "timestamp": "2024-10-07T12:40:00",
            "IMU": {"gyro": [0.1, 0.2, 0.3], "accel": [4.8, 0.0, 0.0]},
            "GPS": {"latitude": 42.12545, "longitude": -78.12315},
            "WiFi": {"csi_imag": 0.5, "csi_real": 0.6, "rssi": -70, "ap_id": "AP123"},
            "Channel": {
                "chan": 1,
                "channel": 36,
                "bw": 20,
                "nss": 2,
                "ntx": 1,
                "nrx": 2,
                "mcs": 7,
            },
        }
    )
    store_received_data(test_data)


def test_retrieve_valid_data():
    print("\nRunning Test Case: Retrieve Valid Data")
    retrieve_data_from_minio("user123", "2024-10-07T12:30:00")


# Main function to handle both tasks and test cases
def main():
    while True:
        print("\nOptions:")
        print(
            "1: Verify if the table (bucket) has been created and list all stored objects."
        )
        print("2: Run Test: Store Valid Data")
        print("3: Retrieve data by entering User ID and Timestamp.")
        print("4: Exit.")
        choice = input("Enter your choice: ")

        if choice == "1":
            # List objects to verify if the bucket and data exist
            list_objects_in_bucket()

        elif choice == "2":
            # Run test case to store valid data
            test_store_valid_data()

        elif choice == "3":
            # Retrieve data by entering User ID and Timestamp
            retrieve_data_from_minio()

        elif choice == "4":
            # Exit the program
            print("Exiting the program.")
            break

        else:
            print("Invalid choice. Please select a valid option.")


if __name__ == "__main__":
    main()
