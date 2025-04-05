"""Compute mean and standard deviation of dataset used for normalization."""
from torch.utils.data import DataLoader
from dataset import DLocDatasetV2
import torch
from tqdm import tqdm
from torchvision import transforms

def get_mean_std(loader: DataLoader) -> tuple:
    """Compute mean and standard deviation across channel dimension of all image samples.

    Note this function assumes the first item in a batch is the image tensor.

    Args:
        loader: pytorch dataloader.

    Returns:
        (mean, std): tuple of mean and standard deviation. Each value is a tensor of shape (num_channels,).
    """
    num_pixels = 0
    sum_ = 0
    sum_of_squares = 0

    for batch_data in tqdm(loader, desc="Computing mean and std"):
        images = batch_data[0]
        batch_size, num_channels, height, width = images.shape
        num_pixels += batch_size * height * width
        sum_ += images.sum(axis=[0, 2, 3])
        sum_of_squares += (images ** 2).sum(axis=[0, 2, 3])

    mean = sum_ / num_pixels
    std = torch.sqrt(sum_of_squares / num_pixels - mean ** 2)

    return mean, std

if __name__ == "__main__":
    data_csv_path = "/media/ehdd_2t/chenfengw/DLoc/dloc_v2/train_files.csv"
    # device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # transform_reshape = transforms.Compose([transforms.Resize(224)])
    transform_reshape = None

    dataset = DLocDatasetV2(data_csv_path, transform=transform_reshape)
    data_loader = DataLoader(dataset, batch_size=32, shuffle=False, num_workers=8)
    mean, std = get_mean_std(data_loader)
    print(f'Mean: {mean}, Std: {std}')

    # test with transform
    # transform = transforms.Compose([transforms.Resize(224),
    #                                 transforms.Normalize(mean=mean,
    #                                                      std=std)])
    # dataset_with_transform = DLocDatasetV2(data_paths, transform=transform)
    # data_loader_with_transform = DataLoader(dataset_with_transform, batch_size=4, shuffle=False)
    # mean_with_transform, std_with_transform = get_mean_std(data_loader_with_transform)

    # # the mean should be close to 0 and std should be close to 1
    # print(f'Mean with transform: {mean_with_transform}, Std with transform: {std_with_transform}')
