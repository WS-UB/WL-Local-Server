import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Optional, Callable
import pandas as pd
import json
import os
from minio import Minio
from io import BytesIO
from dotenv import load_dotenv
from gps_cali import normalize_gps

class DLocDatasetV2(Dataset):
    def __init__(self, parquet_file_path: str, transform: Optional[Callable] = None):
        self.df = pd.read_parquet(parquet_file_path)
        self.transform = transform
        # Define the reshape dimensions for heatmaps (range, angle)
        self.heatmap_dims = (400, 360)
        self.ap_names = ["WiFi-AP-1_HEATMAP", "WiFi-AP-2_HEATMAP", "WiFi-AP-3_HEATMAP"]

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx: int):
        row = self.df.iloc[idx]

        wifi_data = json.loads(row['WiFi'])
        
        # Initialize a list to store heatmaps from all APs
        all_heatmaps = []
        
        # Extract heatmap data from each AP
        for ap_name in self.ap_names:
            if ap_name in wifi_data:
                # Extract complex heatmap data
                heatmap_str = wifi_data[ap_name]
                
                # Convert string representation of complex numbers back to complex values
                # Parse each complex number string and reconstruct the array
                heatmap_complex = np.array([complex(val) for val in heatmap_str], dtype=np.complex64)
                
                # Reshape to the heatmap dimensions
                heatmap_reshaped = heatmap_complex.reshape(self.heatmap_dims)
                
                # Extract magnitude (amplitude) of the heatmap
                heatmap_magnitude = np.abs(heatmap_reshaped)
                
                # Append to list of heatmaps
                all_heatmaps.append(heatmap_magnitude)
            else:
                # If this AP's data is missing, use zeros
                all_heatmaps.append(np.zeros(self.heatmap_dims, dtype=np.float32))
        
        # Stack all heatmaps along a new dimension
        combined_heatmaps = np.stack(all_heatmaps, axis=0)
        
        # Get GPS data
        gps_data = json.loads(row['GPS'])
        # Normalize GPS data
        norm_gps = normalize_gps(gps_data)

        # Convert to torch tensors
        heatmap_tensor = torch.from_numpy(combined_heatmaps.astype(np.float32))

        if self.transform:
            heatmap_tensor = self.transform(heatmap_tensor)

        return heatmap_tensor, norm_gps

    @staticmethod
    def process_parquet_file(parquet_file_path: str):
        df = pd.read_parquet(parquet_file_path)
        all_heatmaps = []
        all_gps = []
        heatmap_dims = (400, 360)
        ap_names = ["WiFi-AP-1_HEATMAP", "WiFi-AP-2_HEATMAP", "WiFi-AP-3_HEATMAP"]

        for _, row in df.iterrows():
            wifi_data = json.loads(row['WiFi'])
            
            # Process each AP's heatmap
            ap_heatmaps = []
            for ap_name in ap_names:
                if ap_name in wifi_data:
                    heatmap_str = wifi_data[ap_name]
                    heatmap_complex = np.array([complex(val) for val in heatmap_str], dtype=np.complex64)
                    heatmap_reshaped = heatmap_complex.reshape(heatmap_dims)
                    ap_heatmaps.append(np.abs(heatmap_reshaped))
                else:
                    ap_heatmaps.append(np.zeros(heatmap_dims, dtype=np.float32))
            
            combined_heatmaps = np.stack(ap_heatmaps, axis=0)
            all_heatmaps.append(combined_heatmaps)

            gps_data = json.loads(row['GPS'])
            gps_coords = np.array([gps_data['latitude'], gps_data['longitude']], dtype=np.float32)
            all_gps.append(gps_coords)

        return all_heatmaps, all_gps

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

# Example usage
if __name__ == "__main__":
    datapath = './data/new.parquet'  # Update with your actual file path
    dataset = DLocDatasetV2(parquet_file_path=datapath)
    print(f"Dataset length: {len(dataset)}")
    
    # Get the first sample
    heatmaps, norm_gps = dataset[0]
    print(f"Heatmaps shape: {heatmaps.shape}")  # Should be [3, 400, 360]
    print(f"Normalized GPS: {norm_gps}")

    # Visualize a heatmap (optional)
    import matplotlib.pyplot as plt
    
    plt.figure(figsize=(10, 8))
    plt.imshow(
        heatmaps[0].numpy(),  # Display the first AP's heatmap
        aspect='auto',
        extent=[-30, 30, -90, 90],  # Based on DISTANCES and ANGLES from original code
        origin='lower'
    )
    plt.colorbar(label='Magnitude')
    plt.xlabel('Range (m)')
    plt.ylabel('AoA (°)')
    plt.title('Heatmap from WiFi-AP-1')
    plt.show()