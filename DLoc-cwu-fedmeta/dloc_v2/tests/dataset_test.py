from dataset import DLocDatasetV2
from torch.utils.data import DataLoader
from utils.ray_intersection_solver import solve_ray_intersection_batch
from utils.geometry_utils import cos_angle_sum, sin_angle_sum
from utils.schema import APMetadata
import torch

def test_dloc_dataset_v2():
    """Check dataset class and dataloader output number and dimensions.
    """
    data_path1 = '/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_train_2'
    data_path2 = '/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_train_3'
    data_paths = [data_path1, data_path2]

    # test single data path
    dataset_single_path = DLocDatasetV2(data_path1)
    assert dataset_single_path[0] is not None
    assert len(dataset_single_path[0]) == 3

    # test multiple data paths
    dataset_multiple_paths = DLocDatasetV2(data_paths)
    assert dataset_multiple_paths[0] is not None
    assert len(dataset_multiple_paths[0]) == 3

    # test shape of dataset
    features_2d, aoa_label, location_label = dataset_single_path[0]
    n_ap = features_2d.shape[0]
    n_ap, height, width = features_2d.shape

    # AoA label should have shape (n_ap,)
    assert aoa_label.shape == (n_ap,)

    # Location label should have shape (2,)
    assert location_label.shape == (2,)

    # Test dataloader
    batch_size = 5
    data_loader = DataLoader(dataset_single_path, batch_size=batch_size, shuffle=False)
    first_batch = next(iter(data_loader))
    features_2d_batch, aoa_label_batch, location_label_batch = first_batch
    assert features_2d_batch.shape == (batch_size, n_ap, height, width)
    assert aoa_label_batch.shape == (batch_size, n_ap)
    assert location_label_batch.shape == (batch_size, 2)


def test_dataset_accuracy():
    """Iterate through the dataset and check if using aoa_label can reproduce the location_label.
    """
    data_path = '/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_test_2'
    dataset = DLocDatasetV2(data_path)
    data_loader = DataLoader(dataset, batch_size=32, shuffle=False, num_workers=8)
    ap_metadata = APMetadata()
    max_batch = 10

    for batch_id, batch in enumerate(data_loader):
        _, aoa_label, location_label = batch
        cos_aoa = torch.cos(aoa_label)
        sin_aoa = torch.sin(aoa_label)

        cos_aoa_map_frame = cos_angle_sum(cos_aoa,
                                          sin_aoa,
                                          ap_metadata.cos_ap_orientations,
                                          ap_metadata.sin_ap_orientations)
        sin_aoa_map_frame = sin_angle_sum(cos_aoa,
                                          sin_aoa,
                                          ap_metadata.cos_ap_orientations,
                                          ap_metadata.sin_ap_orientations)
        location_pred = solve_ray_intersection_batch(ap_metadata.ap_locations,
                                                     cos_aoa_map_frame,
                                                     sin_aoa_map_frame,
                                                     torch.ones_like(cos_aoa))
        assert torch.allclose(location_label, location_pred, atol=1e-4)

        print(f'Batch {batch_id} passed')

        if batch_id == max_batch:
            break
