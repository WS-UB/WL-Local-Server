import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Optional, Callable
import pandas as pd
import json
from dotenv import load_dotenv
from gps_cali import normalize_gps
from fetchdata import fetch_selected_parquet_from_minio, fetch_and_split_parquet_from_minio


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
        
        # Pull out AoA ground truth
        aoa_val = float(wifi_data.get("AoA Ground Truth", 0.0))
        aoa_tensor = torch.tensor(aoa_val, dtype=torch.float32)

        # Get GPS data
        gps_data = json.loads(row['GPS'])
        # Normalize GPS data
        norm_gps = normalize_gps(gps_data)

        # Convert to torch tensors
        heatmap_tensor = torch.from_numpy(combined_heatmaps.astype(np.float32))

        if self.transform:
            heatmap_tensor = self.transform(heatmap_tensor)

        return heatmap_tensor, aoa_tensor, norm_gps

    @staticmethod
    def process_parquet_file(parquet_file_path: str):
        df = pd.read_parquet(parquet_file_path)
        all_heatmaps = []
        all_gps = []
        all_aoa = []
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

            all_aoa.append(float(wifi_data.get("AoA Ground Truth", 0.0)))

            gps_data = json.loads(row['GPS'])
            gps_coords = np.array([gps_data['latitude'], gps_data['longitude']], dtype=np.float32)
            all_gps.append(gps_coords)

        return all_heatmaps, all_aoa, all_gps



if __name__ == "__main__":
    mode = input("Do you want to 'load' data or 'process' data? (type 'load' or 'process'): ").strip().lower()
    if mode == "process":
        splits = fetch_and_split_parquet_from_minio(max_workers=8)
        if splits:
            print("\n✅ Files have been split and downloaded as follows:")
            print(f"Train files: {len(splits['train'])}")
            print(f"Validation files: {len(splits['val'])}")
            print(f"Test files: {len(splits['test'])}")
        else:
            print("❌ Processing failed; no files were split.")
    else:
        # Default is 'load' mode, which preserves the current functionality
        path = fetch_selected_parquet_from_minio()
        if path:
            dataset = DLocDatasetV2(parquet_file_path=path)
            print(f"✅ Loaded {len(dataset)} samples from {path}")
        else:
            print("❌ Failed to download a file from MinIO.")
            exit(1)

        print(f"Dataset length: {len(dataset)}")
        
        # Get the first sample
        heatmaps, aoa_tensor, norm_gps = dataset[0]
        print(f"Heatmaps shape: {heatmaps.shape}")  # Should be [3, 400, 360]
        print(f"Aoa Ground Truth: {aoa_tensor}")
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