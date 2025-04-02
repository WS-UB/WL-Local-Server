import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Optional, Callable
import pandas as pd
import json
import re

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