"""Dataset class for DLocV2 dataset. Load aoa-tof plots and groundtruth labels."""

import torch
from torch.utils.data import Dataset
from utils.data_utils import list_and_sort_files
import h5py
import numpy as np
from typing import Union, List, Optional, Callable
from utils.schema import DatasetKeys
import os
import pandas as pd

class DLocDatasetV2(Dataset):
    """Dataset class for DLocV2 dataset."""
    def __init__(self, data_file_csv: str, transform: Optional[Callable] = None):
        """Constructor for DLocDatasetV2.

        Args:
            data_file_csv: Path to the csv file containing the data files.
            transform: Torch transform to transform aoa-tof plot. Defaults to None.
        """
        self.data_files_list = self._get_all_data_from_csv(data_file_csv)
        self.transform = transform

    def __len__(self):
        return len(self.data_files_list)

    def __getitem__(self, idx: int):
        """Return data and ground truth given an index.

        Args:
            idx: Index to get data and ground truth.

        Returns:
            A tuple of (features_2d, aoa_label, location_label).
            features_2d: 2D fft plot of CSI data. Shape is (n_ap, H, W).
            aoa_label: Angle of arrival ground truth of the signal. Shape is (n_ap,).
            location_label: Location ground truth of the signal. Shape is (2,) representing (x, y).
        """
        data_path = self.data_files_list[idx]
        df = pd.read_parquet(data_path)
        # Assuming that each parquet file contains a single sample,
        # we take the first row.
        row = df.iloc[0]
        features_2d_np = np.array(row[DatasetKeys.FEATURES_2D.value], dtype=np.float32)
        # If needed, perform the same transpose and squeeze operations as before:
        features_2d_np = np.transpose(features_2d_np).squeeze()
        aoa_label_np = np.array(row[DatasetKeys.AOA_GT_LABEL.value], dtype=np.float32).squeeze()
        location_label_np = np.array(row[DatasetKeys.LOCATION_GT_LABEL.value], dtype=np.float32).squeeze()

        # Convert to torch tensors
        features_2d = torch.tensor(features_2d_np)
        aoa_label = torch.tensor(aoa_label_np)
        location_label = torch.tensor(location_label_np)

        if self.transform:
            features_2d = self.transform(features_2d)

        return features_2d, aoa_label, location_label

    def _get_all_data_from_directory(self, data_paths: Union[List[str], str]) -> List[str]:
        """Get all data files from the given data paths.

        Args:
            data_paths: List of data directories that contain individual data files. Can also be a single data directory.

        Returns:
            A list containing absolute paths to all data files sorted by their filenames.
        """
        if isinstance(data_paths, str):
            return list_and_sort_files(data_paths)

        data_files_list = []
        for data_path in set(data_paths):
            data_files_list += list_and_sort_files(data_path)

        return data_files_list

    def _get_all_data_from_csv(self, data_file_csv: str) -> List[str]:
        """Get all data files from the given csv file.

        Args:
            data_file_csv: Path to the csv file containing the data files.

        Returns:
            A list containing absolute paths to all data files sorted by their filenames.
        """
        assert os.path.exists(data_file_csv), f"Data file csv {data_file_csv} does not exist."

        with open(data_file_csv, "r") as f:
            data_files_list = [line.strip() for line in f.readlines()]

        if len(data_files_list) == 0:
            raise ValueError(f"No data files found in {data_file_csv}.")

        return data_files_list
