import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Optional, Callable, List, Dict, Union
import pandas as pd
import json
from dotenv import load_dotenv
from gps_cali import normalize_gps
from fetchdata import (
    fetch_selected_parquet_from_minio,
    fetch_and_split_parquet_from_minio,
)
from minio import Minio
import os
from io import BytesIO

load_dotenv()
BUCKET = os.getenv("MINIO_BUCKET")


def get_minio_client():
    load_dotenv()
    return Minio(
        os.getenv("MINIO_ENDPOINT"),
        access_key=os.getenv("MINIO_ACCESS_KEY"),
        secret_key=os.getenv("MINIO_SECRET_KEY"),
        secure=os.getenv("MINIO_SECURE", "false").lower() == "true",
    )


class DLocDatasetV2(Dataset):
    """Loads one sample per parquet from MinIO, based on a CSV index."""

    def __init__(self, index_csv: str = None, parquet_path: str = None, transform=None):
        """
        Args:
            index_csv: CSV with file paths (for batch processing)
            parquet_path: Direct path to a single parquet file (for inference)
            transform: Normalization transform
        """

        self.bucket = BUCKET
        self._client = None
    
        # self.index_df = pd.read_csv(index_csv)
        # if 'file_path' not in self.index_df:
        #     raise ValueError(f"'file_path' column missing in {index_csv}")

        # Full list of MinIO object names, e.g. "my-folder/subfolder/my.parquet"
        # self.file_paths = self.index_df['file_path'].tolist()
        #self.client = get_minio_client()
        self.transform = transform

        self.heatmap_dims = (400, 360)
        self.ap_names = [
            "WiFi-AP-1_HEATMAP",
            "WiFi-AP-2_HEATMAP",
            "WiFi-AP-3_HEATMAP",
        ]

        if parquet_path:
            # For single-file inference
            self.file_paths = [parquet_path]
        elif index_csv:
            # For batch processing (training/validation)
            self.index_df = pd.read_csv(index_csv)
            self.file_paths = self.index_df['file_path'].tolist()
        else:
            raise ValueError("Must provide either index_csv or parquet_path")
    @property
    def client(self):
        if self._client is None:
            self._client = get_minio_client()
        return self._client

    def __len__(self):
        return len(self.file_paths)

    def __getitem__(self, idx: int):
        # 1) Fetch the parquet bytes from MinIO
        obj_name = self.file_paths[idx]
        obj = self.client.get_object(self.bucket, obj_name)
        buf = BytesIO(obj.read())

        # 2) Load the single-row parquet
        df = pd.read_parquet(buf)
        row = df.iloc[0]

        # 3) Parse WiFi JSON → heatmaps
        wifi = json.loads(row["WiFi"])
        heatmaps = []
        for ap in self.ap_names:
            if ap in wifi:
                arr = np.array([complex(v) for v in wifi[ap]], dtype=np.complex64)
                arr = arr.reshape(self.heatmap_dims)
                heatmaps.append(np.abs(arr))
            else:
                heatmaps.append(np.zeros(self.heatmap_dims, dtype=np.float32))

        combined = np.stack(heatmaps, axis=0).astype(np.float32)
        heatmap_tensor = torch.from_numpy(combined)
        if self.transform:
            heatmap_tensor = self.transform(heatmap_tensor)

        # 4) AoA
        aoa = float(wifi.get("AoA Ground Truth", 0.0))
        aoa_tensor = torch.full((len(self.ap_names),), aoa, dtype=torch.float32)

        # 5) GPS → normalize → tensor
        gps_raw = json.loads(row["GPS"])
        gps_norm = normalize_gps(gps_raw)
        gps_tensor = torch.tensor(gps_norm, dtype=torch.float32)

        return heatmap_tensor, aoa_tensor, gps_tensor
        return heatmap_tensor, aoa_tensor, gps_tensor

    @staticmethod
    def process_dataframe(df: pd.DataFrame):
        """
        Process a pandas DataFrame containing WiFi and GPS data

        Args:
            df: Pandas DataFrame with WiFi and GPS columns

        Returns:
            Tuple of (heatmaps, aoa values, gps coordinates)
        """
        all_heatmaps = []
        all_gps = []
        all_aoa = []
        heatmap_dims = (400, 360)
        ap_names = ["WiFi-AP-1_HEATMAP", "WiFi-AP-2_HEATMAP", "WiFi-AP-3_HEATMAP"]
        all_aoa = []
        heatmap_dims = (400, 360)
        ap_names = ["WiFi-AP-1_HEATMAP", "WiFi-AP-2_HEATMAP", "WiFi-AP-3_HEATMAP"]

        for _, row in df.iterrows():
            wifi_data = json.loads(row["WiFi"])

            # Process each AP's heatmap
            ap_heatmaps = []
            for ap_name in ap_names:
                if ap_name in wifi_data:
                    heatmap_str = wifi_data[ap_name]
                    heatmap_complex = np.array(
                        [complex(val) for val in heatmap_str], dtype=np.complex64
                    )
                    heatmap_reshaped = heatmap_complex.reshape(heatmap_dims)
                    ap_heatmaps.append(np.abs(heatmap_reshaped))
                else:
                    ap_heatmaps.append(np.zeros(heatmap_dims, dtype=np.float32))

            combined_heatmaps = np.stack(ap_heatmaps, axis=0)
            all_heatmaps.append(combined_heatmaps)

            all_aoa.append(float(wifi_data.get("AoA Ground Truth", 0.0)))

            gps_data = json.loads(row["GPS"])
            gps_coords = np.array(
                [gps_data["latitude"], gps_data["longitude"]], dtype=np.float32
            )
            all_gps.append(gps_coords)

        return all_heatmaps, all_aoa, all_gps


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # 1) Prompt only for CSV index path
    csv_path = input("CSV index path (e.g. data/parquet_index.csv): ").strip()

    # 2) Hardcode the bucket name

    # 3) Instantiate
    try:
        ds = DLocDatasetV2(index_csv=csv_path, transform=None)
    except Exception as e:
        print(f"❌ Failed to load dataset: {e}")
        exit(1)

    print(f"✅ Loaded dataset from bucket '{BUCKET}' with {len(ds)} samples.")

    # 4) Prompt for which sample to inspect (1-based)
    n = len(ds)
    idx_input = input(f"Enter sample index to inspect [1–{n}]: ").strip()
    try:
        idx = int(idx_input)
        assert 1 <= idx <= n
    except:
        print("❌ Invalid index. Must be between 1 and", n)
        exit(1)

    # 5) Fetch and display
    heatmap, aoa, gps = ds[idx - 1]
    print("\nSample data:")
    print(f" • heatmap shape: {heatmap.shape}")
    print(f" • AoA ground truth: {aoa.item():.2f}")
    print(f" • Normalized GPS: lat={gps[0].item():.6f}, lon={gps[1].item():.6f}")

    # 6) Optional visualize first AP
    if input("\nVisualize all 3 AP heatmaps? (y/n): ").strip().lower() == "y":
        ap_names = ["WiFi-AP-1", "WiFi-AP-2", "WiFi-AP-3"]
        fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=True)
        for i, ax in enumerate(axes):
            ax.imshow(
                heatmap[i].numpy(),
                origin="lower",
                aspect="auto",
                extent=[0, 5, -90, 90],
            )
            ax.set_title(f"Sample {idx_input} {ap_names[i]} Heatmap")
            ax.set_xlabel("Range (m)")
            if i == 0:
                ax.set_ylabel("AoA (°)")
        plt.tight_layout()
        plt.show()
