import sys, os, time, json
from io import BytesIO
import numpy as np
import pandas as pd
import scipy.io
from minio import Minio

import os
import importlib.util

# 1) Compute the absolute path to pred_loc.py
#    Adjust this relative path if your layout is different
PROJECT_ROOT = os.path.abspath(os.path.join(__file__, os.pardir, os.pardir, os.pardir))
pred_loc_path = os.path.join(PROJECT_ROOT, "DLoc-cwu-fedmeta", "dloc_v2", "pred_loc.py")

# 2) Dynamically load the module
spec = importlib.util.spec_from_file_location("pred_loc", pred_loc_path)
pred_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pred_mod)

# 3) Grab the function
predict_gps = pred_mod.predict_gps


# ensure project root on path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))
from src.data_processing.constants import subcarrier_indices, subcarrier_width, get_channel_frequencies
from src.data_processing.pipeline_utils import extract_csi

# === Configuration ===
# Toggle automatic processing on new Parquet arrivals (True) or manual mode (False)
AUTO_RUN = True
# Local base directory for storing heatmap Parquets
BASE_OUT = os.path.join(os.getcwd(), "heatmap_data")
# MinIO connection via environment variables
minio_client = Minio(
    os.getenv("MINIO_ENDPOINT", "127.0.0.1:9000"),
    access_key=os.getenv("MINIO_ACCESS_KEY", "admin"),
    secret_key=os.getenv("MINIO_SECRET_KEY", "password"),
    secure=os.getenv("MINIO_SECURE", "false").lower() == "true",
)
BUCKET = os.getenv("MINIO_BUCKET", "wl-data")

# === Constants ===
AP_NAMES = ["WiFi-AP-1", "WiFi-AP-2", "WiFi-AP-3"]
COMPENSATION_FILES = {
    "192.168.48.1": "src/data_processing/compensated_csi/48.1.mat",
    "192.168.48.2": "src/data_processing/compensated_csi/48.2.mat",
    "192.168.48.3": "src/data_processing/compensated_csi/48.3.mat",
}
SUBCARRIER_SPACING = subcarrier_width
BW = 80e6
DISTANCES = np.arange(-10, 40, 0.125)
ANGLES    = np.arange(-90, 90, 0.5)
FREQ      = 5e9
C         = 3e8
N_Rx      = 4
WAVELENGTH= C / FREQ
d         = WAVELENGTH / 2

# === Helper Functions ===
def extract_csi_data(wifi_data: dict) -> dict:
    csi_data = {}
    for ap in AP_NAMES:
        if ap in wifi_data:
            i_vals = wifi_data[ap][1].split(": ")[1].strip("[]").split(", ")
            r_vals = wifi_data[ap][2].split(": ")[1].strip("[]").split(", ")
            csi_data[ap] = [i_vals, r_vals]
    return csi_data


def haversine(lat1, lon1, lat2, lon2):
    R = 6.371e3
    dlat = np.deg2rad(lat2 - lat1)
    dlon = np.deg2rad(lon2 - lon1)
    a = np.sin(dlat/2)**2 + np.cos(np.deg2rad(lat1))*np.cos(np.deg2rad(lat2))*np.sin(dlon/2)**2
    return 2*R*np.arcsin(np.sqrt(a))*1e3


def peakFind(mat: np.ndarray) -> float:
    idx = np.unravel_index(np.argmax(np.abs(mat)), mat.shape)
    return ANGLES[idx[1]]


def generate_AoA_GT(user_GPS: list, apLoc: list, apL1: list, apL2: list, apName: str):
    uLat,uLon = user_GPS
    apLat,apLon = apLoc
    l1Lat,l1Lon = apL1
    l2Lat,l2Lon = apL2
    theta = np.arctan2(l2Lat-l1Lat, l2Lon-l1Lon)*(180/np.pi)
    phi   = np.arctan2(uLat-apLat, uLon-apLon)*(180/np.pi)
    dist  = haversine(uLat,uLon,apLat,apLon)
    aoaGT = phi - (90 + theta)
    return aoaGT, theta, phi, dist


def calibrate_csi(ap_name: str, csi_i: list, csi_r: list, aoaGT: float, csi_comp: np.ndarray=None) -> np.ndarray:
    # full CSI calibration + FFT pipeline
    csi_complex = extract_csi(80, csi_i=csi_i, csi_r=csi_r, apply_nts=False, comp=csi_comp)[:,:,0]
    fs = 8e6
    freqs = np.arange(0, fs, fs/len(get_channel_frequencies(155, BW)[1]))
    # Range FFT
    exponent_range = np.exp(1j*2*np.pi*DISTANCES.reshape(-1,1) @ freqs.reshape(1,-1)/C)
    rangeFFT = exponent_range @ csi_complex
    # AoA FFT
    exponent_AoA = np.exp(1j*2*np.pi*FREQ*d/C *
                          np.arange(1,N_Rx+1).reshape(-1,1) @ 
                          np.sin(np.radians(ANGLES)).reshape(1,-1))
    AoARangeFFT = rangeFFT @ exponent_AoA  # 400x360 matrix
    return AoARangeFFT


def store_heatmaps_locally(user_id: str, timestamp: str, gyro: list, accel: list,
                            gps: dict, gpsr: dict, heatmaps: list, apLoc: list,
                            apL1: list, apL2: list, theta: float, phi: float,
                            tof: float, aoaGT: float):
    """
    Store a single Parquet file of heatmap matrices & metadata in
    heatmap_data/<user_id>_HEATMAPS/<timestamp>.parquet
    """
    folder = f"{user_id}_HEATMAPS"
    out_dir = os.path.join(BASE_OUT, folder)
    os.makedirs(out_dir, exist_ok=True)
    df = pd.DataFrame({
        'user_id':[user_id],
        'timestamp':[timestamp],
        'gyro':[gyro],
        'accel':[accel],
        'GPS':[json.dumps(gps)],
        'GPS_RAW':[json.dumps(gpsr)],
        'AP1_heatmap':[heatmaps[0].tolist()],
        'AP2_heatmap':[heatmaps[1].tolist()],
        'AP3_heatmap':[heatmaps[2].tolist()],
        'AP_Loc':[json.dumps(apLoc)],
        'AP_L1':[json.dumps(apL1)],
        'AP_L2':[json.dumps(apL2)],
        'Theta':[theta],
        'Phi':[phi],
        'ToF':[tof],
        'AoA_GT':[aoaGT]
    })
    path = os.path.join(out_dir, f"{timestamp}.parquet")
    df.to_parquet(path, engine='pyarrow', index=False)
    print(f"Heatmap Parquet written: {path}")
    try:
        print("→ Running GPS prediction on", path)
        predict_gps(path)
    except Exception as e:
        print("Error while running predict_gps:", e)
# === Raw-data Parquet discovery ===

def get_user_ids():
    objs = minio_client.list_objects(BUCKET, "", recursive=True)
    return set(o.object_name.split("/",1)[0] for o in objs)


def most_recent_parquet(uid: str) -> str:
    prefix = f"{uid}/"
    files = [o for o in minio_client.list_objects(BUCKET, prefix=prefix, recursive=True)
             if o.object_name.endswith('.parquet')]
    if not files: return None
    return max(files, key=lambda o: o.last_modified).object_name


def process_parquet(key: str, uid: str):
    obj = minio_client.get_object(BUCKET, key)
    df  = pd.read_parquet(BytesIO(obj.read()), engine='pyarrow')
    timestamp = df['timestamp'][0]
    imu       = json.loads(df['IMU'][0])
    gps       = json.loads(df['GPS'][0])
    gpsr      = json.loads(df['GPS_RAW'][0])
    wifi_list = json.loads(df['WiFi'][0]) if isinstance(df['WiFi'][0], str) else df['WiFi'].apply(json.loads)[0]

    gyro  = imu['gyro']; accel = imu['accel']
    lat, lon = gps['latitude'], gps['longitude']
    latr, lonr = gpsr['latitude'], gpsr['longitude']

    heatmaps=[]
    for ap in AP_NAMES:
        raw = wifi_list[ap]
        csi_i = [float(x) for x in raw[1].split(', ')]
        csi_r = [float(x) for x in raw[2].split(', ')]
        ip = wifi_list[ap][11].split(': ')[1]
        apLoc = json.loads(wifi_list[ap][12].split(': ')[1])
        apL1  = json.loads(wifi_list[ap][13].split(': ')[1])
        apL2  = json.loads(wifi_list[ap][14].split(': ')[1])
        comp = np.array(scipy.io.loadmat(COMPENSATION_FILES[ip])['csi'])[0]
        aoaGT, theta, phi, tof = generate_AoA_GT([lat,lon], apLoc, apL1, apL2, ap)
        hm = calibrate_csi(ap, csi_i, csi_r, aoaGT, comp)
        heatmaps.append(hm)

    if len(heatmaps)==len(AP_NAMES):
        store_heatmaps_locally(uid, timestamp, gyro, accel,
                              {'latitude':lat,'longitude':lon},
                              {'latitude':latr,'longitude':lonr},
                              heatmaps, apLoc, apL1, apL2, theta, phi, tof, aoaGT)

# === Manual mode ===

def manual_process():
    folder = input("Enter folder name (user_id): ")
    prefix = f"{folder}/"
    objs = minio_client.list_objects(BUCKET, prefix=prefix, recursive=True)
    for o in objs:
        if o.object_name.endswith('.parquet'):
            process_parquet(o.object_name, folder)

# === Main loop ===

def main():
    if AUTO_RUN:
        seen = set()
        while True:
            for uid in get_user_ids():
                if uid in seen: continue
                key = most_recent_parquet(uid)
                if key:
                    process_parquet(key, uid)
                    seen.add(uid)
            time.sleep(5)
    else:
        manual_process()

if __name__ == '__main__':
    main()
