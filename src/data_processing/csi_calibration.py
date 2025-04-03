import numpy as np
import sys
import os
from os.path import join

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))
import json
import random
from minio import Minio
from io import BytesIO
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
import matplotlib.pyplot as plt
from src.data_processing.constants import subcarrier_indices
from src.data_processing.pipeline_utils import extract_csi

AP_NAMES = ["WiFi-AP-1", "WiFi-AP-2", "WiFi-AP-3"]
DISTANCES = np.arange(0, 40, 0.1)
ANGLES = np.arange(-90, 90, 0.5)
FREQ = 5.8e9  # WiFi at 5.8 GHz
C = 3e8  # Speed of light
WAVELENGTH = C / FREQ
d = WAVELENGTH / 2  # Antenna spacing
OUT = os.getcwd()

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

                    calibrate_csi(csi_i_flattened, csi_r_flattened)


# def omit_frequencies(csi_i, csi_r, bw):
#     # Check if bandwidth is valid
#     if bw not in subcarrier_indices:
#         print("Invalid bandwidth for OFDM")
#         return None, None

#     # Get the indices of subcarriers to keep
#     keep_indices = np.in1d(np.arange(256), subcarrier_indices[bw])

#     # Use the indices to select the data that should remain
#     csi_i_filtered = csi_i[keep_indices]
#     csi_r_filtered = csi_r[keep_indices]

#     # print(f"Filtered csi_i size: {len(csi_i_filtered)}")
#     # print(f"Filtered csi_r size: {len(csi_r_filtered)}")

#     return csi_i_filtered, csi_r_filtered


def calibrate_csi(
    csi_i: list[float], csi_r: list[float], csi_compensated: list[float] = None
):
    csi_complex = extract_csi(80, csi_i=csi_i, csi_r=csi_r, valid_tx=0, apply_nts=False)
    # Hcomp = csi_complex
    # np.save(join(OUT, f"comp-{random.randint(1, 10)}.npy"), Hcomp)
    print(f"Complex CSI: {csi_complex}")

    csi_padded = np.zeros((400, 4), dtype=np.complex128)
    csi_padded[:234, :] = csi_complex.reshape(
        234, 4
    )  # Copy existing CSI data, pad the rest with zeros

    csi_distance = np.fft.ifft(csi_padded, axis=0)
    csi_distance = np.abs(csi_distance)

    k = 2 * np.pi / WAVELENGTH  # Wavenumber

    # Create a steering matrix that explicitly maps to the 360 angles
    steering_matrix = np.exp(
        1j * k * d * np.outer(np.arange(4),s np.sin(np.radians(ANGLES)))
    )

    csi_angle = np.dot(csi_distance, steering_matrix)  # (400, 360)

    heatmap = np.abs(np.fft.fftshift(np.fft.fft2(csi_angle, s=(400, 360))))

    print(f"FFT: {heatmap}")

    plt.figure(figsize=(10, 6))

    plt.imshow(
        heatmap, aspect="auto", extent=[-90, 90, 0, 40], origin="lower", cmap="jet"
    )
    plt.colorbar(label="Amplitude")
    plt.xlabel("Angle (degrees)")
    plt.ylabel("Distance (m)")
    plt.title("CSI 2D Fourier Transform Heatmap")
    plt.show()


def main():
    retrieve_csi()


if __name__ == "__main__":
    main()
