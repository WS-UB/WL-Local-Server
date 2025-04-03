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

BW = 80e6
DISTANCES = np.arange(0, 40, 0.1)
ANGLES = np.arange(-90, 90, 0.5)
FREQ = 5.8e9  # WiFi at 5.8 GHz
C = 3e8  # Speed of light
N_Rx = 4
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

                    calibrate_csi(ap_name, csi_i_flattened, csi_r_flattened)


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
    ap_name, csi_i: list[float], csi_r: list[float], csi_compensated: list[float] = None
):
    csi_complex = extract_csi(80, csi_i=csi_i, csi_r=csi_r, apply_nts=True)[:, :, 0]

    # Hcomp = csi_complex
    # np.save(join(OUT, f"comp-{random.randint(1, 10)}.npy"), Hcomp)
    print(f"Complex CSI: {csi_complex.shape}")

    fs = 8e6  # ADC sampling frequency in Hz
    N_samples = 234  # Number of samples per chirp
    k = BW / 0.8e-6  # Slope (Hz/s)
    print(k)

    # Time and frequency axes
    Ts = 1 / fs  # Sampling period
    t = np.arange(0, N_samples) * Ts  # Time axis
    delta_freqs = np.arange(0, fs, fs / N_samples)  # Frequency axis
    delta_est = delta_freqs / k  # Slope-based estimation
    distance_range = delta_est * C  # Convert to distance

    rangeFFT = np.fft.fft(csi_complex, 234, 0)

    plt.figure(1)
    plt.plot(distance_range, np.abs(rangeFFT))
    plt.xlabel("Distance (m)")
    plt.ylabel("Amplitude")
    plt.title(f"Range FFT for {ap_name}")
    plt.grid(True)
    plt.show()

    exponent_AoA = np.exp(
        (1j * 2 * np.pi * FREQ * d / C)
        * np.arange(1, N_Rx + 1)[:, None]
        * np.sin(np.radians(ANGLES))
    )

    AoARangeFFT = rangeFFT @ exponent_AoA

    plt.figure(2)
    plt.imshow(
        np.abs(AoARangeFFT).T,
        aspect="auto",
        extent=[
            distance_range.min(),
            distance_range.max(),
            ANGLES.min(),
            ANGLES.max(),
        ],
        origin="lower",
    )

    # Labels and title
    plt.xlabel("Range (m)")
    plt.ylabel("AoA (°)")
    plt.title("AoA vs Range Heatmap")

    # Colorbar for scale
    plt.colorbar(label="Magnitude")

    plt.show()


def main():
    retrieve_csi()


if __name__ == "__main__":
    main()
