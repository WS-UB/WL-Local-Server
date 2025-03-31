from metrics_calculator import AoAAccuracy, MetricNames
from model import ModelOutput, GTlabel
import torch
from utils.geometry_utils import wrap_to_pi

def test_AoAAccuracy():
    # Test the AoAAccuracy class
    n_aps = 4
    batch_size = 5
    n_batch = 6
    metric = AoAAccuracy(n_aps=n_aps)

    # prepare fake data
    pred_list = [ModelOutput(aoa=torch.randn(batch_size, n_aps),
                             cos_aoa=None,
                             sin_aoa=None,
                             location=None,
                             confidence=None) for _ in range(n_batch)]
    target_list = [GTlabel(aoa=torch.randn(batch_size, n_aps),
                           cos_aoa=None,
                           sin_aoa=None,
                           location=None) for _ in range(n_batch)]

    # extract values into tensor
    pred_value_tensor = torch.cat([pred.aoa for pred in pred_list], dim=0)
    target_value_tensor = torch.cat([target.aoa for target in target_list], dim=0)

    # compute mean and std of the difference
    diff_tensor = wrap_to_pi(pred_value_tensor - target_value_tensor)
    diff_mean = diff_tensor.mean(dim=0)
    diff_std = diff_tensor.std(dim=0, correction=0)

    # compute mean using the metric
    for idx,(pred, target) in enumerate(zip(pred_list, target_list)):
        metric.update(pred, target)

    result = metric.compute()

    # compare results
    assert torch.allclose(result[MetricNames.AOA_ERROR_MEAN], diff_mean, atol=10e-5)
    assert torch.allclose(result[MetricNames.AOA_ERROR_STD], diff_std, atol=10e-5)
