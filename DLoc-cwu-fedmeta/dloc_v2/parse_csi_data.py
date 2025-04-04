import json
from minio import Minio
from io import BytesIO
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
import numpy as np


AP_NAMES = ["WiFi-AP-1", "WiFi-AP-2", "WiFi-AP-3"]
DISTANCES = np.arange(0, 40, 0.1)
ANGLES = np.arange(-90, 90, 0.5)

minio_client = Minio(
    "128.205.218.189:9000",  # Replace with your MinIO server address
    access_key="admin",  # MinIO access key
    secret_key="password",  # MinIO secret key
    secure=False,  # Set to True if using HTTPS
)


def extract_csi_data(wifi_data, ap_names=AP_NAMES):
    csi_data = {}
    for ap_name in ap_names:
        if ap_name in wifi_data:
            csi_i = wifi_data[ap_name][1].split(": ")[1].strip("[]").split(", ")
            csi_r = wifi_data[ap_name][2].split(": ")[1].strip("[]").split(", ")
            csi_data[ap_name] = [csi_i, csi_r]
    return csi_data


def retrieve_csi(bucket_name="wl-data"):
    global AP_NAMES
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
                pd.set_option("display.max_colwidth", None)
                # json_data = data.to_json(991956470159_DC991956470159_DCorient="records", indent=4).replace("\\", "")
                # print(f"{json_data}\n")
                data["WiFi"] = data["WiFi"].apply(json.loads)

                for wifi_data in data["WiFi"]:
                    csi_data = extract_csi_data(wifi_data)
                    print("\n")
                    for ap_name in AP_NAMES:
                        csi_i_flattened = np.array(
                            [float(x) for x in csi_data[ap_name][0]]
                        ).flatten()
                        csi_r_flattened = np.array(
                            [float(x) for x in csi_data[ap_name][1]]
                        ).flatten()

                        print(csi_i_flattened)
                        print(csi_r_flattened)

    except Exception as e:
        print(f"Error retrieving data: {str(e)}")
        print(
            "Please check if the object exists and ensure the User ID and Timestamp are correct."
        )


def main():
    retrieve_csi()


if __name__ == "__main__":
    main()
