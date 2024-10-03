import json
import struct
from minio import Minio
from minio.error import S3Error
from elasticsearch import Elasticsearch, exceptions

minio_client = Minio(
    "100.100.100.100:9000",
    access_key="minioadmin",
    secret_key="minioadmin",
    secure=False
)

es = Elasticsearch(“100.100.100.100:9200")

def list_minio_objects(bucket_name="csi-data"):
    try:
        objects = minio_client.list_objects(bucket_name)
        return [obj.object_name for obj in objects]
    except S3Error as err:
        print(f"MinIO Error: {err}")
        return []

def download_minio_object(bucket_name, object_name, download_path="/tmp/csi_data.bin"):
    try:
        minio_client.fget_object(bucket_name, object_name, download_path)
        print(f"Downloaded {object_name} to {download_path}")
        return download_path
    except S3Error as err:
        print(f"Error downloading {object_name}: {err}")
        return None

def parse_csi_data(file_content):
    num_floats = len(file_content) // 4
    data = struct.unpack(f'{num_floats}f', file_content)
    return data

def index_csi_data_to_elasticsearch(index_name, data):
    for i, entry in enumerate(data):
        doc = {
            "csi_value": entry,
            "timestamp": "2024-09-28T15:45:00"
        }
        try:
            es.index(index=index_name, body=doc)
        except exceptions.ConnectionError as e:
            print(f"Failed to index document {i}: {e}")

def query_elasticsearch(index_name, query):
    try:
        response = es.search(index=index_name, body=query)
        return response['hits']['hits']
    except exceptions.ConnectionError as e:
        print(f"Failed to connect to Elasticsearch: {e}")
        return []

def main():
    bucket_name = "csi-data"
    object_name = "csi_data_12345.bin"
    objects = list_minio_objects(bucket_name)
    print(f"Objects in bucket '{bucket_name}': {objects}")

    download_path = download_minio_object(bucket_name, object_name)

    if download_path:
        with open(download_path, 'rb') as file:
            file_content = file.read()
            parsed_data = parse_csi_data(file_content)
            print(f"Parsed Data (first 10 entries): {parsed_data[:10]}")
            index_csi_data_to_elasticsearch("csi-index", parsed_data)

    query = {
        "query": {
            "match_all": {}
        }
    }

    es_results = query_elasticsearch("csi-index", query)

    if es_results:
        print("Elasticsearch query results:")
        for result in es_results:
            print(json.dumps(result, indent=2))
    else:
        print("No results found in Elasticsearch.")

if __name__ == "__main__":
    main()
