"""Implement various metrics calculators for evaluating the performance of the model.
"""
from torchmetrics import Metric
import torch
from utils.schema import ModelOutput, GTlabel
from utils.geometry_utils import wrap_to_pi
import pdb
import numpy as np
from gps_cali import reverse_normalization
import matplotlib.pyplot as plt



class MetricNames:
    """Class to store all metrics that metrics calculator generates."""
    AOA_ERROR_MEAN = "aoa_error_mean"
    AOA_ERROR_STD = "aoa_error_std"
    AOA_ERROR_MSE = "aoa_error_mse"
    AOA_PREDS = "aoa_preds"
    AOA_TARGETS = "aoa_targets"
    AOA_ERRORS_ALL = "aoa_errors_all"
    LOCATION_ERROR_MEAN = "location_error_mean"
    LOCATION_ERROR_MEDIAN = "location_error_median"
    LOCATION_ERROR_STD = "location_error_std"
    LOCATION_ERROR_90_PERCENTILE = "location_error_90_percentile"
    LOCATION_ERROR_99_PERCENTILE = "location_error_99_percentile"
    LOCATION_PREDS = "location_preds"
    LOCATION_TARGETS = "location_targets"
    LOCATION_ERROR_CDF = "location_error_cdf_figure"
   

class AoAAccuracy(Metric):
    """This class computes mean and standard deviation of AoA error.
    The error is defined as difference between predicted AoA and the ground truth AoA.

    Reference: https://lightning.ai/docs/torchmetrics/stable/pages/implement.html
    """
    def __init__(self, n_aps: int, **kwargs):
        """Initialize the class.

        Args:
            n_aps: number of APs.
        """
        super().__init__(**kwargs)
        self.add_state("sum_aoa_error", default=torch.zeros(n_aps), dist_reduce_fx="sum")
        self.add_state("sum_aoa_error_sq", default=torch.zeros(n_aps), dist_reduce_fx="sum")
        self.add_state("total", default=torch.tensor(0), dist_reduce_fx="sum")
        self.add_state("preds", default=[])
        self.add_state("targets", default=[])
        self.add_state("aoa_error_all", default=[])

    def update(self, pred: ModelOutput, target: GTlabel) -> None:
        # Calculate the difference between prediction and ground truth
        # pred and target aoa shape is (batch_size, n_aps)
        assert pred.aoa.shape == target.aoa.shape
        aoa_diff = wrap_to_pi(pred.aoa - target.aoa)
        n_sample = pred.aoa.shape[0]

        # Update the states
        self.sum_aoa_error += aoa_diff.sum(dim=0)
        self.sum_aoa_error_sq += (aoa_diff ** 2).sum(dim=0)
        self.total += n_sample

        # accumulate prediction and target aoa
        self.preds.append(pred.aoa)
        self.targets.append(target.aoa)
        self.aoa_error_all.append(aoa_diff)

    def compute(self) -> torch.Tensor:
        """Compute mean and standard deviation of AoA error.

        Returns:
            A dictionary containing mean and standard deviation of AoA error.
            the shape of mean is (n_aps,) and the shape of std is (n_aps,)
        """
        # Compute the mean AoA error
        aoa_error_mean = self.sum_aoa_error / self.total
        aoa_error_std = torch.sqrt(self.sum_aoa_error_sq / self.total - aoa_error_mean ** 2)
        aoa_error_mse = self.sum_aoa_error_sq / self.total

        # convert aoa prediction and target to tensor
        preds_tensor = torch.cat(self.preds, dim=0)
        targets_tensor = torch.cat(self.targets, dim=0)
        aoa_error_tensor = torch.cat(self.aoa_error_all, dim=0)

        return {MetricNames.AOA_ERROR_MEAN: aoa_error_mean,
                MetricNames.AOA_ERROR_STD: aoa_error_std,
                MetricNames.AOA_ERROR_MSE: aoa_error_mse,
                MetricNames.AOA_PREDS: preds_tensor,
                MetricNames.AOA_TARGETS: targets_tensor,
                MetricNames.AOA_ERRORS_ALL: aoa_error_tensor}

class LocationAccuracy(Metric):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_state("preds", default=[])
        self.add_state("targets", default=[])

    def update(self, pred: ModelOutput, target: GTlabel) -> None:
        # accumulate prediction and target location
        self.preds.append(pred.location)
        self.targets.append(target.location)

    def compute(self) -> dict:
        """Compute mean, median, standard deviation, 90th percentile, and 99th percentile of location error.

        Returns:
            A dictionary containing the computed metrics. Also return the prediction and target tensors for
            plotting purpose.
        """

        preds_tensor = torch.cat(self.preds, dim=0) # shape: (n_samples, 2)
        targets_tensor = torch.cat(self.targets, dim=0) # shape: (n_samples, 2)

        reversed_preds_tensor = reverse_normalization(preds_tensor[:, 0], preds_tensor[:, 1])
        reversed_targets_tensor = reverse_normalization(targets_tensor[:, 0], targets_tensor[:, 1])

        # errors = torch.norm(preds_tensor - targets_tensor, dim=1) #uses the distance formula to get the error in lat/lon
        errors_m = measure(reversed_preds_tensor[:, 0], 
                    reversed_preds_tensor[:, 1], 
                    reversed_targets_tensor[:, 0], 
                    reversed_targets_tensor[:, 1])
        
        mean_error = errors_m.mean()
        median_error = errors_m.median()
        std_error = errors_m.std()
        percentile_90_error = torch.quantile(errors_m, 0.90)
        percentile_99_error = torch.quantile(errors_m, 0.99)

        cdf_fig = plot_location_error_cdf(errors_m)

        return {
            MetricNames.LOCATION_ERROR_MEAN: mean_error,
            MetricNames.LOCATION_ERROR_MEDIAN: median_error,
            MetricNames.LOCATION_ERROR_STD: std_error,
            MetricNames.LOCATION_ERROR_90_PERCENTILE: percentile_90_error,
            MetricNames.LOCATION_ERROR_99_PERCENTILE: percentile_99_error,
            MetricNames.LOCATION_PREDS: preds_tensor,
            MetricNames.LOCATION_TARGETS: targets_tensor,
            MetricNames.LOCATION_ERROR_CDF: cdf_fig
        }

# def measure(lat1, lon1, lat2, lon2):
#     """Calculate the distance between two GPS coordinates using the Haversine formula.

#     Args: 
#         lat1: Latitude of the first point in degrees.
#         lon1: Longitude of the first point in degrees.
#         lat2: Latitude of the second point in degrees.
#         lon2: Longitude of the second point in degrees.

#     Returns:
#         Distance in meters.
#     """
#     r = 6378.137  # Radius of Earth in meters
#     dLat = np.radians(lat2 - lat1)
#     dLon = np.radians(lon2 - lon1)
#     a = (np.sin(dLat / 2) ** 2 +
#          np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) *
#          (np.sin(dLon / 2) ** 2))
#     c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
#     d = r * c  # Distance in meters
#     return d * 1000  # Convert to meters

def measure(lat1, lon1, lat2, lon2):
    """Calculate the distance between two GPS coordinates using the Haversine formula.
    
    Args: 
        lat1: Latitude of the first point in degrees (tensor)
        lon1: Longitude of the first point in degrees (tensor)
        lat2: Latitude of the second point in degrees (tensor)
        lon2: Longitude of the second point in degrees (tensor)
    
    Returns:
        Distance in meters as a tensor.
    """
    # Convert to radians
    lat1_rad = torch.deg2rad(lat1)
    lon1_rad = torch.deg2rad(lon1)
    lat2_rad = torch.deg2rad(lat2)
    lon2_rad = torch.deg2rad(lon2)
    
    # Haversine formula using PyTorch operations
    dLat = lat2_rad - lat1_rad
    dLon = lon2_rad - lon1_rad
    
    a = (torch.sin(dLat / 2) ** 2 + 
         torch.cos(lat1_rad) * torch.cos(lat2_rad) * 
         torch.sin(dLon / 2) ** 2)
    
    c = 2 * torch.atan2(torch.sqrt(a), torch.sqrt(1 - a))
    r = 6378.137  # Radius of Earth in km
    return (r * c) * 1000  # Distance in meters


def plot_location_error_cdf(location_errors: torch.Tensor, max_error: float = 10.0) -> plt.Figure:
    """
    Plot the CDF of location errors.
    
    Args:
        location_errors: Tensor containing all location errors in meters
        max_error: Maximum error value to display on x-axis (default: 10 meters)
        
    Returns:
        matplotlib Figure object
    """
    # Convert to numpy array if it's a tensor
    if isinstance(location_errors, torch.Tensor):
        errors = location_errors.cpu().numpy()
    else:
        errors = np.array(location_errors)
    
    # Sort the errors
    sorted_errors = np.sort(errors)
    
    # Calculate cumulative probabilities
    n = len(sorted_errors)
    y = np.arange(1, n + 1) / n
    
    # Create the plot
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Plot the CDF
    ax.plot(sorted_errors, y, marker='.', linestyle='none', markersize=5)
    
    # Add a line for better visibility
    ax.plot(sorted_errors, y, 'b-', alpha=0.5)
    
    # Set axis limits and labels
    ax.set_xlim(0, max_error)
    ax.set_ylim(0, 1)
    ax.set_xlabel('Location Error (meters)')
    ax.set_ylabel('Cumulative Probability')
    ax.set_title('CDF of Location Errors')
    
    # Add grid
    ax.grid(True, which='both', linestyle='--', alpha=0.5)
    
    # Add important percentiles
    for percentile in [50, 75, 90, 95]:
        value = np.percentile(errors, percentile)
        ax.axvline(x=value, color='r', linestyle='--', alpha=0.5)
        ax.text(value, 0.1, f'{percentile}%: {value:.2f}m', 
                rotation=90, verticalalignment='bottom')
    
    plt.tight_layout()
    return fig


if __name__ == "__main__":
    from torchmetrics import MetricCollection
    # Test the AoAAccuracy class
    batch_size = 5
    n_batch = 6
    n_aps = 4
    metrics_collection = MetricCollection([AoAAccuracy(n_aps), LocationAccuracy()])

    # prepare fake data
    pred_list = [ModelOutput(aoa=torch.randn(batch_size, n_aps),
                             cos_aoa=None,
                             sin_aoa=None,
                             location=torch.rand(batch_size, 2),
                             confidence=None) for _ in range(n_batch)]
    target_list = [GTlabel(aoa=torch.rand(batch_size, n_aps),
                           cos_aoa=None,
                           sin_aoa=None,
                           location=torch.rand(batch_size, 2)) for _ in range(n_batch)]

    # extract values into tensor
    pred_value_tensor = torch.cat([pred.location for pred in pred_list], dim=0)
    target_value_tensor = torch.cat([target.location for target in target_list], dim=0)

    # compute mean and std of the difference
    diff_tensor = torch.norm(pred_value_tensor - target_value_tensor, dim=1)
    diff_mean = diff_tensor.mean(dim=0)
    diff_std = diff_tensor.std()
    diff_median = diff_tensor.median()
    print(f"diff_mean: {diff_mean}, diff_std: {diff_std}, diff_median: {diff_median}")

    # compute mean using the metric
    for idx,(pred, target) in enumerate(zip(pred_list, target_list)):
        metrics_collection.update(pred, target)
        # print(f"batch {idx} , total count: {metric.total}")

    result = metrics_collection.compute()
    print(f"metrics calculation results: {result.keys()}")
    print(f"aoa metrics shape: {result[MetricNames.AOA_ERROR_MEAN].shape}")
    # Output: {'mean': tensor([-0.1000, -0.1000, -0.1000, -0.1000]), 'std': tensor([0., 0., 0., 0.])}
    # The mean AoA error is -0.1 for all APs and the standard deviation is 0 for all APs.

    # test metrics collection


