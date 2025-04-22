import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Optional, Callable, List, Dict, Union
import pandas as pd
import json
from dotenv import load_dotenv
from gps_cali import normalize_gps
from fetchdata import fetch_selected_parquet_from_minio, fetch_and_split_parquet_from_minio


class DLocDatasetV2(Dataset):
    def __init__(self, data_source: Union[str, pd.DataFrame], transform: Optional[Callable] = None):
        """
        Initialize dataset with either a parquet file path or a pandas DataFrame
        
        Args:
            data_source: Either a file path to a parquet file or a pandas DataFrame with the data
            transform: Optional transform to apply to the heatmap data
        """
        if isinstance(data_source, str):
            # Load from file path
            self.df = pd.read_parquet(data_source)
        else:
            # Use provided DataFrame directly
            self.df = data_source
            
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
        gps_tensor = torch.tensor(norm_gps, dtype=torch.float32)

        # Convert to torch tensors
        heatmap_tensor = torch.from_numpy(combined_heatmaps.astype(np.float32))

        if self.transform:
            heatmap_tensor = self.transform(heatmap_tensor)

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
    
    @staticmethod
    def process_parquet_file(parquet_file_path: str):
        """Maintained for backward compatibility"""
        df = pd.read_parquet(parquet_file_path)
        return DLocDatasetV2.process_dataframe(df)


def create_datasets_from_dataframes(dataframes_dict):
    """
    Create datasets from dictionary of dataframes
    
    Args:
        dataframes_dict: Dictionary with keys 'train', 'val', 'test' containing lists of DataFrames
        
    Returns:
        Dictionary with keys 'train', 'val', 'test' containing Dataset objects
    """
    datasets = {}
    
    for split_name, dfs in dataframes_dict.items():
        if not dfs:
            datasets[split_name] = None
            continue
            
        # Concatenate all dataframes in this split
        if len(dfs) > 1:
            combined_df = pd.concat(dfs, ignore_index=True)
        else:
            combined_df = dfs[0]
            
        # Create dataset from the combined dataframe
        datasets[split_name] = DLocDatasetV2(combined_df)
        
    return datasets


if __name__ == "__main__":
    mode = input("Choose mode:\n1. Load single dataset\n2. Process split datasets\nEnter choice (1-2): ").strip()
    
    if mode == "2":
        print("Fetching and splitting datasets...")
        splits = fetch_and_split_parquet_from_minio(max_workers=8)
        
        if splits:
            # Create datasets from the fetched dataframes
            datasets = create_datasets_from_dataframes(splits)
            
            print("\n✅ Created datasets with the following sizes:")
            for split_name, dataset in datasets.items():
                if dataset:
                    print(f"{split_name.capitalize()} dataset: {len(dataset)} samples")
                else:
                    print(f"{split_name.capitalize()} dataset: No data")
            
            # Visualize a sample from training set if available
            if datasets['train'] and len(datasets['train']) > 0:
                # Get the first sample from training set
                heatmaps, aoa_tensor, norm_gps = datasets['train'][0]
                print(f"\nSample from training set:")
                print(f"Heatmaps shape: {heatmaps.shape}")
                print(f"AoA Ground Truth: {aoa_tensor}")
                print(f"Normalized GPS: {norm_gps}")
                
                # Visualize a heatmap (optional)
                visualize = input("\nVisualize heatmap? (y/n): ").strip().lower()
                if visualize == 'y':
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
        else:
            print("❌ Fetching and splitting failed; no datasets were created.")
    else:
        # Single dataset mode
        print("Fetching a single dataset...")
        data = fetch_selected_parquet_from_minio()
        
        if data is not None:
            # The fetchdata.py now returns a DataFrame directly
            dataset = DLocDatasetV2(data)
            print(f"✅ Created dataset with {len(dataset)} samples")
            
            # Get the first sample
            if len(dataset) > 0:
                heatmaps, aoa_tensor, norm_gps = dataset[0]
                print(f"Heatmaps shape: {heatmaps.shape}")  # Should be [3, 400, 360]
                print(f"AoA Ground Truth: {aoa_tensor}")
                print(f"Normalized GPS: {norm_gps}")
                
                # Visualize a heatmap (optional)
                visualize = input("Visualize heatmap? (y/n): ").strip().lower()
                if visualize == 'y':
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
        else:
            print("❌ Failed to fetch data from MinIO.")