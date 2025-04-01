"""Dataset class for Parquet files containing CSI and GPS data."""

import torch
from torch.utils.data import Dataset
import numpy as np
from typing import Optional, Callable
import os
import pandas as pd
import json

class DLocDatasetV2(Dataset):
    """Dataset class for Parquet files containing CSI and GPS data."""
    
    def __init__(self, parquet_file_path: str, transform: Optional[Callable] = None):
        """Constructor for CSIGPSDataset.

        Args:
            parquet_file_path: Path to the parquet file containing the data.
            transform: Optional transform to be applied to the CSI data. Defaults to None.
        """
        self.df = pd.read_parquet(parquet_file_path)
        self.transform = transform
        
    def __len__(self):
        return len(self.df)
    
    def __getitem__(self, idx: int):
        """Return CSI and GPS data given an index.

        Args:
            idx: Index to get data.

        Returns:
            A tuple of (csi_data, gps_data).
            csi_data: Complex CSI data. Shape is (n_subcarriers,).
            gps_data: GPS coordinates. Shape is (2,) representing (latitude, longitude).
        """
        row = self.df.iloc[idx]
        
        # Parse WiFi data which contains CSI
        wifi_data = json.loads(row['WiFi'])
        csi_real = np.array(wifi_data['csi_real'], dtype=np.float32)
        csi_imag = np.array(wifi_data['csi_imag'], dtype=np.float32)
        csi_complex = csi_real + 1j * csi_imag
        
        # Parse GPS data
        gps_data = json.loads(row['GPS'])
        gps_coords = np.array([gps_data['latitude'], gps_data['longitude']], dtype=np.float32)
        
        # Convert to torch tensors
        csi_tensor = torch.from_numpy(csi_complex)
        gps_tensor = torch.from_numpy(gps_coords)
        
        if self.transform:
            csi_tensor = self.transform(csi_tensor)
            
        return csi_tensor, gps_tensor

    @staticmethod
    def process_parquet_file(parquet_file_path: str):
        """Process a parquet file and return all CSI and GPS data.
        
        Args:
            parquet_file_path: Path to the parquet file.
            
        Returns:
            A tuple of (all_csi, all_gps).
            all_csi: List of all CSI data arrays.
            all_gps: List of all GPS coordinate arrays.
        """
        df = pd.read_parquet(parquet_file_path)
        all_csi = []
        all_gps = []
        
        for _, row in df.iterrows():
            # Parse WiFi data which contains CSI
            wifi_data = json.loads(row['WiFi'])
            csi_real = np.array(wifi_data['csi_real'], dtype=np.float32)
            csi_imag = np.array(wifi_data['csi_imag'], dtype=np.float32)
            csi_complex = csi_real + 1j * csi_imag
            all_csi.append(csi_complex)
            
            # Parse GPS data
            gps_data = json.loads(row['GPS'])
            gps_coords = np.array([gps_data['latitude'], gps_data['longitude']], dtype=np.float32)
            all_gps.append(gps_coords)
            
        return all_csi, all_gps
