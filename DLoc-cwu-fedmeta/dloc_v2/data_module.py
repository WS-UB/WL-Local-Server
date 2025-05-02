"""
Data module for DLoc dataset. Data module is used to load the dataset and create dataloaders for training, validation and testing.
"""

from pytorch_lightning import LightningDataModule
from torch.utils.data import DataLoader, Dataset
from dataset import DLocDatasetV2
from typing import Optional, Union, List, Callable


class DLocDataModule(LightningDataModule):
    def __init__(
        self,
        *,
        train_data_paths: Union[List[str], str],
        val_data_paths: Union[List[str], str],
        test_data_paths: Optional[Union[List[str], str]] = None,
        transform: Optional[Callable] = None,
        batch_size: int = 32,
        num_workers: int = 0,
        prefetch_factor: int = 2,
    ):
        super().__init__()
        self.train_data_paths = train_data_paths
        self.val_data_paths = val_data_paths
        self.test_data_paths = test_data_paths
        self.transform = transform
        self.batch_size = batch_size
        self.num_workers = num_workers
        self.prefetch_factor = prefetch_factor

    def setup(self, stage: Optional[str] = None):
        """
        Instantiate datasets for each split, unwrapping list if necessary.
        """

        def load_dataset(data_paths):
            if not data_paths:
                return None
            if isinstance(data_paths, list):
                data_paths = data_paths[0] if data_paths else None
                if not data_paths:
                    return None
            return DLocDatasetV2(index_csv=data_paths, transform=self.transform)

        self.train_dataset = load_dataset(self.train_data_paths)
        self.val_dataset = load_dataset(self.val_data_paths)
        self.test_dataset = load_dataset(self.test_data_paths)

    def _create_dataloader(self, dataset: Dataset, shuffle: bool) -> DataLoader:
        """
        Helper method to create a DataLoader.
        """
        if dataset is None:
            raise ValueError("Dataset is not provided.")

        loader_args = {
            "dataset": dataset,
            "batch_size": self.batch_size,
            "shuffle": shuffle,
            "num_workers": self.num_workers,
            "persistent_workers": False,
        }
        # Only set prefetch_factor when using multiprocessing
        if self.num_workers > 0:
            loader_args["prefetch_factor"] = self.prefetch_factor

        return DataLoader(**loader_args)

    def train_dataloader(self):
        return self._create_dataloader(self.train_dataset, shuffle=True)

    def val_dataloader(self):
        return self._create_dataloader(self.val_dataset, shuffle=False)

    def test_dataloader(self):
        return self._create_dataloader(self.test_dataset, shuffle=False)


if __name__ == "__main__":
    train_data_path = "/Users/yanghu/302 Project/WL-Local-Server/DLoc-cwu-fedmeta/dloc_v2/data/train_index.csv"
    val_data_path = "/Users/yanghu/302 Project/WL-Local-Server/DLoc-cwu-fedmeta/dloc_v2/data/validation_index.csv"
    test_data_path = "/Users/yanghu/302 Project/WL-Local-Server/DLoc-cwu-fedmeta/dloc_v2/data/test_index.csv"

    data_module = DLocDataModule(
        train_data_paths=train_data_path,
        val_data_paths=val_data_path,
        test_data_paths=test_data_path,
    )
    data_module.setup()

    # Test train loader
    for batch in data_module.train_dataloader():
        features_2d, aoa_label, location_label = batch
        print(f"train batches: {len(data_module.train_dataloader())}")
        print(
            f"Features shape: {features_2d.shape}, AOA Label shape: {aoa_label.shape}, Location Label shape: {location_label.shape}"
        )
        break

    # Test val loader
    for batch in data_module.val_dataloader():
        features_2d, aoa_label, location_label = batch
        print(f"val batches: {len(data_module.val_dataloader())}")
        print(
            f"Features shape: {features_2d.shape}, AOA Label shape: {aoa_label.shape}, Location Label shape: {location_label.shape}"
        )
        break

    # Test test loader
    for batch in data_module.test_dataloader():
        features_2d, aoa_label, location_label = batch
        print(f"test batches: {len(data_module.test_dataloader())}")
        print(
            f"Features shape: {features_2d.shape}, AOA Label shape: {aoa_label.shape}, Location Label shape: {location_label.shape}"
        )
        break
