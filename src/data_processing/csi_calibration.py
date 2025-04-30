import numpy as np
import scipy.io
from scipy.signal import find_peaks
import sys
import os
import json
from minio import Minio
from io import BytesIO
import pandas as pd
import matplotlib.pyplot as plt
import shutil


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from src.data_processing.constants import (
    subcarrier_width,
    get_channel_frequencies,
)
from src.minio_script import store_received_data
from src.data_processing.pipeline_utils import extract_csi

os.chdir(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

AP_NAMES = ["WiFi-AP-1", "WiFi-AP-2", "WiFi-AP-3"]
COMPENSATION_FILES = {
    "192.168.48.1": "src/data_processing/compensated_csi/48.1.mat",
    "192.168.48.2": "src/data_processing/compensated_csi/48.2.mat",
    "192.168.48.3": "src/data_processing/compensated_csi/48.3.mat",
}

SUBCARRIER_SPACING = subcarrier_width  # Subcarrier spacing (312.5 kHz)
BW = 80e6
DISTANCES = np.arange(-10, 40, 0.125)
ANGLES = np.arange(-90, 90, 0.5)
FREQ = 5e9  # WiFi at 5.8 GHz
C = 3e8  # Speed of light
N_Rx = 4
WAVELENGTH = C / FREQ
d = WAVELENGTH / 2  # Antenna spacing
OUT = os.getcwd()
HISTOGRAM = []


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

    save_dir = os.path.join(
        "/Users/harrisonmoore/Developer/WL-Local-Server/heatmap_data", folder
    )
    if os.path.exists(save_dir):
        shutil.rmtree(save_dir)  # Deletes the whole directory and contents
    os.makedirs(save_dir)

    for obj in objects:
        if obj.object_name.endswith(".parquet"):
            response = minio_client.get_object(bucket_name, obj.object_name)
            data = pd.read_parquet(BytesIO(response.read()), engine="pyarrow")
            pd.set_option("display.max_colwidth", None)
            user_id = data["user_id"][0]
            timestamp = data["timestamp"][0]
            imu = data["IMU"].apply(json.loads)
            gps = data["GPS"].apply(json.loads)
            data["WiFi"] = data["WiFi"].apply(json.loads)

            gyro_xyz = imu[0]["gyro"]
            accel_xyz = imu[0]["accel"]
            GPS_lat = gps[0]["latitude"]
            GPS_long = gps[0]["longitude"]

            for wifi_data in data["WiFi"]:
                heatmaps = []
                csi_data = extract_csi_data(wifi_data)
                for ap_name in AP_NAMES:
                    rx_ip = wifi_data[ap_name][11].split(": ")[1]
                    apLoc = json.loads(wifi_data[ap_name][12].split(": ")[1])
                    apL1 = json.loads(wifi_data[ap_name][13].split(": ")[1])
                    apL2 = json.loads(wifi_data[ap_name][14].split(": ")[1])
                    csi_compensatedRaw = scipy.io.loadmat(
                        COMPENSATION_FILES[rx_ip]
                    )  # Load CSI compensation
                    csi_compensated = np.array(csi_compensatedRaw["csi"])[
                        0, :, :, :
                    ]  # Reshape to 234x4x4

                    csi_i_flattened = np.array(
                        [float(x) for x in csi_data[ap_name][0]]
                    ).flatten()
                    csi_r_flattened = np.array(
                        [float(x) for x in csi_data[ap_name][1]]
                    ).flatten()

                    aoaGT, theta, phi, tof = generate_AoA_GT(
                        [GPS_lat, GPS_long],
                        apLoc=apLoc,
                        apL1=apL1,
                        apL2=apL2,
                        apName=ap_name,
                    )

                    heatmap = calibrate_csi(
                        ap_name,
                        csi_i_flattened,
                        csi_r_flattened,
                        aoaGT=aoaGT,
                        folderName=folder,
                        timestamp=timestamp,
                        csi_compensated=csi_compensated,
                    )

                    heatmaps.append(heatmap)
                    if len(heatmaps) == 3:
                        send_heatmaps(
                            user_id=user_id,
                            timestamp=timestamp,
                            gyro_xyz=gyro_xyz,
                            accel_xyz=accel_xyz,
                            GPS_lat=GPS_lat,
                            GPS_long=GPS_long,
                            heatmaps=heatmaps,
                            apLoc=apLoc,
                            apL1=apL1,
                            apL2=apL2,
                            theta=theta,
                            phi=phi,
                            tof=tof,
                            aoaGT=aoaGT,
                        )
    return


def send_heatmaps(
    user_id,
    timestamp,
    gyro_xyz,
    accel_xyz,
    GPS_lat,
    GPS_long,
    heatmaps,
    apLoc,
    apL1,
    apL2,
    theta,
    phi,
    tof,
    aoaGT,
):
    user_data = json.dumps(
        [
            {
                "user_id": f"{user_id}_HEATMAPS",
                "timestamp": timestamp,
                "IMU": {"gyro": gyro_xyz, "accel": accel_xyz},
                "GPS": {"latitude": GPS_lat, "longitude": GPS_long},
                "WiFi": {
                    "WiFi-AP-1_HEATMAP": heatmaps[0],
                    "WiFi-AP-2_HEATMAP": heatmaps[1],
                    "WiFi-AP-3_HEATMAP": heatmaps[2],
                    "AP Location": apLoc,
                    "AP L1": apL1,
                    "AP L2": apL2,
                    "Theta": theta,
                    "Phi": phi,
                    "ToF (m)": tof,
                    "AoA Ground Truth": aoaGT,
                },
            }
        ],
    )
    store_received_data(user_data)


def peakFind(heatmap):
    magnitude = np.abs(heatmap)
    peak_index = np.unravel_index(np.argmax(magnitude), magnitude.shape)
    angle_peak = ANGLES[peak_index[1]]
    return angle_peak


def haversine(latitude1: float, longitude1: float, latitude2: float, longitude2: float):
    earthR = 6.371e3
    latitude1_rads = np.deg2rad(latitude1)
    longitude1_rads = np.deg2rad(longitude1)
    latitude2_rads = np.deg2rad(latitude2)
    longitude2_rads = np.deg2rad(longitude2)
    sin1 = np.sin((latitude2_rads - latitude1_rads) / 2) ** 2
    cos1 = np.cos(latitude1_rads)
    cos2 = np.cos(latitude2_rads)
    sin2 = np.sin((longitude2_rads - longitude1_rads) / 2) ** 2
    computation = 2 * earthR * np.arcsin(np.sqrt(sin1 + cos1 * cos2 * sin2))
    return computation * 1e3


def generate_AoA_GT(
    user_GPS: list[float],
    apLoc: list[float],
    apL1: list[float],
    apL2: list[float],
    apName: str,
):

    apLoc_lat, apLoc_long = (apLoc[0], apLoc[1])
    apL1_lat, apL1_long = (apL1[0], apL1[1])
    apL2_lat, apL2_long = (apL2[0], apL2[1])
    user_lat, user_long = (user_GPS[0], user_GPS[1])
    # print(f"{user_lat}, {user_long}")

    theta = np.arctan2((apL2_lat - apL1_lat), (apL2_long - apL1_long)) * (180 / np.pi)

    # if apName == "WiFi-AP-2":
    #     theta = 0

    calc = haversine(user_lat, user_long, apLoc_lat, apLoc_long)
    # if apName == "WiFi-AP-1":
    # theta = -theta
    print(f"User location: {user_lat}, {user_long}")
    print(f"{apName} Theta: {theta}")
    phi = np.arctan2((user_lat - apLoc_lat), (user_long - apLoc_long)) * (180 / np.pi)
    aoaGt = phi - (90 + theta)
    print(f"{apName} Phi: {phi}")
    print(f"AoA Ground Truth for {apName}: {aoaGt}\n")
    return aoaGt, theta, phi, calc


def calibrate_csi(
    ap_name,
    csi_i: list[float],
    csi_r: list[float],
    aoaGT: float,
    folderName: str,
    timestamp: str,
    csi_compensated: list[float] = None,
):

    csi_complex = extract_csi(
        80, csi_i=csi_i, csi_r=csi_r, apply_nts=False, comp=csi_compensated
    )[:, :, 0]

    # csi_complex = np.squeeze(csi_complex)

    # Hcomp = csi_complex
    # np.save(join(OUT, f"comp-{random.randint(1, 10)}.npy"), Hcomp)

    fs = 8e6  # ADC sampling frequency in Hz
    N_subfrequencies = len(
        get_channel_frequencies(155, 80e6)
    )  # Number of samples per chirp
    fc, freqs_subcarriers = get_channel_frequencies(155, 80e6)
    k = 2 * np.pi * np.mean(N_subfrequencies) / (C)  # Slope (Hz/s)

    # Time and frequency axes
    Ts = 1 / fs  # Sampling period
    t = np.arange(0, N_subfrequencies) * Ts  # Time axis
    delta_freqs = np.arange(0, fs, fs / N_subfrequencies)  # Frequency axis
    delta_est = delta_freqs / k  # Slope-based estimation
    # distance_range = delta_est * C  # Convert to distance

    exponent_range = np.exp(
        (
            1j
            * 2
            * np.pi
            * DISTANCES.reshape(400, 1)
            @ freqs_subcarriers.reshape(234, 1).T
            / C
        )
    )

    rangeFFT = exponent_range @ csi_complex
    exponent_AoA = np.exp(
        (1j * 2 * np.pi * fc * d / C)
        * np.arange(1, N_Rx + 1)[:, None]
        @ np.sin(np.radians(ANGLES.reshape(360, 1))).T
    )

    AoARangeFFT = rangeFFT @ exponent_AoA  # 400x360 matrix
    stringComplex = [
        str(x) for x in AoARangeFFT.flatten().tolist()
    ]  # Converts all complex numbers to strings, needs to be converted back when parsing
    # plot_csiGraph(rangeFFT, ap_name)
    plot_heatmaps(AoARangeFFT, aoaGT, ap_name, folderName, timestamp)

    rawAoA = peakFind(AoARangeFFT)
    print(f"rawAoA: {rawAoA}")
    aoaDiff = aoaGT - rawAoA
    HISTOGRAM.append(aoaDiff)

    return stringComplex


def plot_csiGraph(csiFFT, ap_name):
    plt.figure(1)
    plt.plot(DISTANCES, np.abs(csiFFT))
    plt.xlabel("Distance (m)")
    plt.ylabel("Amplitude")
    plt.title(f"Range FFT for {ap_name}")
    plt.grid(True)
    plt.show()
    return


def plot_heatmaps(heatmap, aoaGT, apName, folderName, timestamp):
    rounded_AoA = round(aoaGT, 1)
    # Step 2: Find the index of the maximum value
    angle_peak = peakFind(heatmap)

    save_dir = os.path.join(
        "/Users/harrisonmoore/Developer/WL-Local-Server/heatmap_data", folderName
    )

    filename = f"{folderName}_{timestamp}_{apName}.jpg"
    filepath = os.path.join(save_dir, filename)

    plt.figure(2)
    plt.imshow(
        np.abs(heatmap).T,
        aspect="auto",
        extent=[
            DISTANCES.min(),
            DISTANCES.max(),
            ANGLES.min(),
            ANGLES.max(),
        ],
        origin="lower",
    )

    # Labels and title
    plt.xlabel("Range (m)", fontsize=20)
    plt.ylabel("AoA (°)", fontsize=20)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.title(f"AoA vs Range Heatmap ({apName})", fontsize=20)
    plt.axhline(
        y=rounded_AoA,
        color="red",
        linestyle="--",
        linewidth=2,
        label=f"Ground Truth AoA = {rounded_AoA}°",
    )
    plt.axhline(
        y=angle_peak,
        color="green",
        linestyle="--",
        linewidth=2,
        label=f"Raw AoA = {angle_peak}°",
    )
    plt.legend(loc="upper right")

    # Colorbar for scale
    plt.colorbar(label="Magnitude")

    plt.savefig(filepath, dpi=300, bbox_inches="tight")
    plt.close()
    return


def plot_histogram(data, folderName):
    plt.hist(data, bins=30, edgecolor="black")

    # Add labels and title
    plt.xlabel("(AoAGT - rawAoA)", fontsize=15)
    plt.ylabel("Frequency", fontsize=15)

    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    plt.title("Histogram", fontsize=15)

    # Show the plot
    plt.show()


def main():
    retrieve_csi()
    plot_histogram(HISTOGRAM, "histogram")
    print("\nHeatmaps have been successfully generated!")


if __name__ == "__main__":
    main()
