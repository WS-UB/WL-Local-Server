import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Optional, Callable
import pandas as pd
import json
import re
import os
from minio import Minio
from io import BytesIO
from dotenv import load_dotenv



class DLocDatasetV2(Dataset):
    def __init__(self, parquet_file_path: str, transform: Optional[Callable] = None):
        self.df = pd.read_parquet(parquet_file_path)
        self.transform = transform
        # Define the reshape dimensions (n_subcarriers, n_rx, n_tx)
        self.reshape_dims = (256, 4, 4)

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx: int):
        row = self.df.iloc[idx]

        wifi_data = json.loads(row['WiFi'])
        first_ap_data = next(iter(wifi_data.values()))

        csi_i_string = first_ap_data[1]
        csi_r_string = first_ap_data[2]

        csi_i = np.array(re.findall(r"[-+]?\d*\.\d+|\d+", csi_i_string), dtype=np.float32)
        csi_r = np.array(re.findall(r"[-+]?\d*\.\d+|\d+", csi_r_string), dtype=np.float32)

        # Combine real and imaginary parts
        csi_complex = csi_r + 1j * csi_i
        
        # Reshape the CSI data
        csi_reshaped = csi_complex.reshape(self.reshape_dims, order='F')
        
        # Get magnitude and phase (optional, can be removed if not needed)
        csi_magnitude = np.abs(csi_reshaped)
        csi_phase = np.angle(csi_reshaped)

        gps_data = json.loads(row['GPS'])
        gps_coords = np.array([gps_data['latitude'], gps_data['longitude']], dtype=np.float32)

        # Convert to torch tensors
        csi_tensor = torch.from_numpy(csi_reshaped)  # or use csi_magnitude/csi_phase if preferred
        gps_tensor = torch.from_numpy(gps_coords)

        if self.transform:
            csi_tensor = self.transform(csi_tensor)

        return csi_tensor, gps_tensor

    @staticmethod
    def process_parquet_file(parquet_file_path: str, return_reshaped=True):
        df = pd.read_parquet(parquet_file_path)
        all_csi = []
        all_gps = []
        reshape_dims = (256, 4, 4)

        for _, row in df.iterrows():
            wifi_data = json.loads(row['WiFi'])
            first_ap_data = next(iter(wifi_data.values()))

            csi_i_string = first_ap_data[1]
            csi_r_string = first_ap_data[2]

            csi_i = np.array(re.findall(r"[-+]?\d*\.\d+|\d+", csi_i_string), dtype=np.float32)
            csi_r = np.array(re.findall(r"[-+]?\d*\.\d+|\d+", csi_r_string), dtype=np.float32)

            csi_complex = csi_r + 1j * csi_i
            
            if return_reshaped:
                # Reshape the CSI data
                csi_complex = csi_complex.reshape(reshape_dims, order='F')
            
            all_csi.append(csi_complex)

            gps_data = json.loads(row['GPS'])
            gps_coords = np.array([gps_data['latitude'], gps_data['longitude']], dtype=np.float32)
            all_gps.append(gps_coords)

        return all_csi, all_gps
    

def fetch_selected_parquet_from_minio(bucket_name="wl-data"):
    load_dotenv()
    minio_client = Minio(
    os.getenv("MINIO_ENDPOINT"),
    access_key=os.getenv("MINIO_ACCESS_KEY"),
    secret_key=os.getenv("MINIO_SECRET_KEY"),
    secure=os.getenv("MINIO_SECURE").lower() == 'true',
)

    folder = input("Enter MinIO folder name: ").strip()
    prefix = f"{folder}/"
    os.makedirs("data", exist_ok=True)

    try:
        objects = list(minio_client.list_objects(bucket_name, prefix=prefix, recursive=True))
        parquet_files = [obj for obj in objects if obj.object_name.endswith(".parquet")]

        if not parquet_files:
            print("⚠️ No .parquet files found in that folder.")
            return None

        print("\n📄 Available .parquet files:")
        for i, obj in enumerate(parquet_files):
            print(f"[{i}] {obj.object_name}")

        idx = int(input("\nSelect file index to download: ").strip())
        selected_obj = parquet_files[idx]

        print(f"\n📥 Downloading {selected_obj.object_name}")
        response = minio_client.get_object(bucket_name, selected_obj.object_name)

        local_filename = os.path.join("data", os.path.basename(selected_obj.object_name))
        with open(local_filename, "wb") as f:
            for chunk in response.stream(32 * 1024):
                f.write(chunk)

        print(f"✅ Saved to {local_filename}")
        return local_filename

    except Exception as e:
        print(f"❌ Error: {e}")
        return None

# --- MAIN ENTRY ---
if __name__ == "__main__":
    path = fetch_selected_parquet_from_minio()
    if path:
        dataset = DLocDatasetV2(parquet_file_path=path)
        print(f"✅ Loaded {len(dataset)} samples from {path}")