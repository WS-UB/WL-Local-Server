"""
Data module for DLoc dataset. Data module is used to load the dataset and create dataloaders for training, validation and testing.
"""
from pytorch_lightning import LightningDataModule
from torch.utils.data import DataLoader
from dataset import DLocDatasetV2
from typing import Optional, Union, List, Callable

class DLocDataModule(LightningDataModule):
    def __init__(self,
                 train_data_paths: Union[List[str], str] = None,
                 val_data_paths: Union[List[str], str] = None,
                 test_data_paths: Optional[Union[List[str], str]] = None,
                 transform: Optional[Callable] = None,
                 batch_size: int = 32,
                 num_workers: int = 8,
                 prefetch_factor: int = 2):
        super().__init__()
        self.train_data_paths = train_data_paths
        self.val_data_paths = val_data_paths
        self.test_data_paths = test_data_paths
        self.transform = transform
        self.batch_size = batch_size
        self.num_workers = num_workers
        self.prefetch_factor = prefetch_factor

    def setup(self, stage: Optional[str] = None):

        def load_dataset(data_paths):
            return DLocDatasetV2(data_paths, transform=self.transform) if data_paths else None

        self.train_dataset = load_dataset(self.train_data_paths)
        self.val_dataset = load_dataset(self.val_data_paths)
        self.test_dataset = load_dataset(self.test_data_paths)

    def train_dataloader(self):
        if self.train_data_paths is None:
            raise ValueError("Training dataset path is not provided.")
        return DataLoader(self.train_dataset, batch_size=self.batch_size, shuffle=True, num_workers=self.num_workers, prefetch_factor=self.prefetch_factor)

    def val_dataloader(self):
        if self.val_data_paths is None:
            raise ValueError("Validation dataset path is not provided.")
        return DataLoader(self.val_dataset, batch_size=self.batch_size, shuffle=False, num_workers=self.num_workers, prefetch_factor=self.prefetch_factor)

    def test_dataloader(self):
        if self.test_data_paths is None:
            raise ValueError("Test dataset path is not provided.")
        return DataLoader(self.test_dataset, batch_size=self.batch_size, shuffle=False, num_workers=self.num_workers, prefetch_factor=self.prefetch_factor)


if __name__ == "__main__":
    train_data_path = ["/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_train_2"]
    val_data_path = ["/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_train_3"]
    test_data_path = ["/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_test_2"]

    data_module = DLocDataModule(train_data_paths=train_data_path, val_data_paths=val_data_path, test_data_paths=test_data_path)
    data_module.setup()

    for batch in data_module.train_dataloader():
        features_2d, aoa_label, location_label = batch
        print(f"train dataloader size: {len(data_module.train_dataloader())}")
        print(f'Features shape: {features_2d.shape}, AOA Label shape: {aoa_label.shape}, Location Label shape: {location_label.shape}')
        break

    for batch in data_module.val_dataloader():
        features_2d, aoa_label, location_label = batch
        print(f"val dataloader size: {len(data_module.val_dataloader())}")
        print(f'Features shape: {features_2d.shape}, AOA Label shape: {aoa_label.shape}, Location Label shape: {location_label.shape}')
        break

    for batch in data_module.test_dataloader():
        features_2d, aoa_label, location_label = batch
        print(f"test dataloader size: {len(data_module.test_dataloader())}")
        print(f'Features shape: {features_2d.shape}, AOA Label shape: {aoa_label.shape}, Location Label shape: {location_label.shape}')
        break
