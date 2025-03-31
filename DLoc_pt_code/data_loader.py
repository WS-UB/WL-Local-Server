"""Dataset class for DLocV2 dataset. Load aoa-tof plots and groundtruth labels."""

import torch
from torch.utils.data import Dataset
from utils.data_utils import list_and_sort_files
import h5py
import numpy as np
from typing import Union, List, Optional, Callable
from utils.schema import DatasetKeys
import os

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

        with h5py.File(data_path, "r") as f:
            # features_2d.shape = (4, 315, 401)
            features_2d_np = np.transpose(np.array(f.get(DatasetKeys.FEATURES_2D.value), dtype=np.float32)).squeeze()

            # aoa_gnd.shape = (4,)
            aoa_label_np = np.array(f.get(DatasetKeys.AOA_GT_LABEL.value), dtype=np.float32).squeeze()

            # xy_label_np.shape = (2,)
            location_label_np = np.array(f.get(DatasetKeys.LOCATION_GT_LABEL.value), dtype=np.float32).squeeze()

        # Convert to torch Tensors

        # 2D fft plot of CSI data. Shape is (n_ap, H, W)
        features_2d = torch.tensor(features_2d_np)

        # Angle of arrival ground truth of the signal. Shape is (n_ap,)
        aoa_label = torch.tensor(aoa_label_np)

        # Location ground truth of the signal. Shape is (2,) representing (x, y)
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