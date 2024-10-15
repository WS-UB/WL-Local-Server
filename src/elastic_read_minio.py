import json
import io
import urllib3
from minio import Minio
from elasticsearch import Elasticsearch

# Configure MinIO Client
minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",      # MinIO access key
    secret_key="password",   # MinIO secret key
    secure=False             # Set to True if using HTTPS
)

# Configure Elasticsearch Client
es = Elasticsearch(
    "https://128.205.218.189:9200", 
    basic_auth=("elastic", "mwLUsUm3IJd=ljk8Sq7P"), 
    verify_certs=False  # Disable certificate verification
)

# Supress all warnings
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# * -------------------------------------------------------------------------------------------- START BUILDING FEATURES HERE --------------------------------------------------------------------------------------------
# ! Feature 1: Retrieve data from MinIO and index in Elasticsearch 
# * This function will retrieve the data from MinIO using the requested User ID and Timestamp
def retrieve_data_from_minio(bucket_name="wl-data"):
    """Retrieve and print the data from MinIO using User ID and Timestamp."""
    
    # Ask the user for input
    print("-------------------------ENTER REQUESTED DATA HERE-------------------------")
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
        print("Please check if the object exists in the bucket and ensure the User ID and Timestamp are correct.")
    return user_id, timestamp, json_data

# * This function will index the JSON file from MinIO and index them in Elasticsearch
def retrieve_data_and_index_es(bucket_name="wl-data"):
    """Retrieve data from MinIO and index it in Elasticsearch."""
    
    user_id, timestamp, json_data = retrieve_data_from_minio()
    
    try:
        # Index the data into Elasticsearch
        index_name = "mqtt-data"  # Adjust index name if needed
        doc_id = f"{user_id}/{timestamp}.json"  # Unique document ID

        es.index(index=index_name, id=doc_id, body=json_data)
        print(f"Data indexed successfully with ID: {doc_id}")

    except Exception as e:
        print(f"Error retrieving or indexing data: {str(e)}")
    finally:
        response.close()
        response.release_conn()

# * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# ! Feature 2: Delete an indexed data from Elasticsearch
def delete_es_data(index_name="mqtt-data", doc_id=""):
    """Delete the requested data from Elasticsearch."""

    user_id = input("Please enter the User ID of the data that you want to delete: ")
    timestamp = input("Please enter the Timestamp of the data you want to delete (format: YYYY-MM-DDTHH:MM:SS): ")
    doc_id = f"{user_id}/{timestamp}.json"
    try:
        response = es.delete(index=index_name, id=doc_id)
        print(f"Document with ID '{doc_id}' deleted successfully.")
        print(response)
    except Exception as e:
        print(f"Error deleting document: {str(e)}")

# * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# ! Feature 3: Print available data from Elasticsearch
def get_es_data(index_name="mqtt-data", doc_id=""):
    """Get the requested data from Elasticsearch."""
    user_id = input("Please enter the User ID of the data that you want read: ")
    timestamp = input("Please enter the Timestamp of the data you want read (format: YYYY-MM-DDTHH:MM:SS): ")
    doc_id = f"{user_id}/{timestamp}.json"
    try:
        response = es.get(index=index_name, id=doc_id)
        # Print the document source
        print("Document retrieved successfully:")
        print(json.dumps(response['_source'], indent=4))
    except Exception as e:
        print(f"Error retrieving document: {str(e)}")


# * ---------------------------------------------------------------------------------------------- MAIN FUNCTION RUNS HERE -----------------------------------------------------------------------------------------------
# !! This is the main function that runs the program
def main():
    while True:
        print("\nOptions:")
        print("1: Retrieve data from MinIO and put the data (index) in Elasticsearch.")
        print("2: Delete an indexed data from Elasticsearch.")
        print("3: Print available data from Elasticsearch.")
        print("4: Exit.")
        choice = input("Enter your choice: ")

        if choice == "1":
            # List objects to verify if the bucket and data exist
            retrieve_data_and_index_es()

        elif choice == "2":
            # Run test case to store valid data
            delete_es_data()

        elif choice == "3":
            # Retrieve data by entering User ID and Timestamp
            get_es_data()

        elif choice == "4":
            # Exit the program
            print("Exiting the program.")
            break
        else:
            print("Invalid choice. Please select a valid option.")

if __name__ == "__main__":
    main()
