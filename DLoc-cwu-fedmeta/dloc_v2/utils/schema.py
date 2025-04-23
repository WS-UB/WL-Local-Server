"""Defines the schema for the dataset and other dataclasses used in the project."""
from dataclasses import dataclass, field
from enum import Enum, unique
import torch
import numpy as np
import os
from datetime import datetime
from typing import Optional

@unique
class DatasetKeys(Enum):
    """Keys for each individual dataset h5 file.
    """
    # 2D fft plot of CSI data. Shape is (H, W, n_ap, 1)
    FEATURES_2D: str = 'features_2d'

    # Angle of arrival ground truth of the signal. Shape is (n_ap, 1)
    AOA_GT_LABEL: str = 'aoa_gnd'

    # Location ground truth of the signal. Shape is (2, 1) representing (x, y)
    LOCATION_GT_LABEL: str = 'labels'

    # Time of flight ground truth of the signal. Shape is (n_ap, 1)
    TOF_GT_LABEL: str = 'tof_gnd'


class APMetadata:
    """Metadata for the WiFi access points in the dataset.
    data read form ap.h5 file. The ap_orientation is not correct in the file. Use hard coded value
    in this class instead.
    """
    def __init__(self) -> None:
        # AP locations in the dataset. Shape is (n_aps, 2) representing (x, y)
        self._ap_locations = torch.tensor([[ 0. ,  2. ],
                                          [16.8,  7.6],
                                          [ 6.4,  7.6],
                                          [12.4,  0. ]])

        # Orientation of normal of the AP in the dataset w.r.t to map frame. Shape is (n_aps,)
        self._ap_orientations = torch.tensor([0, -np.pi, -np.pi/2, np.pi/2])
        assert self._ap_locations.shape[0] == self._ap_orientations.shape[0], "Shape mismatch."

        # cosine of ap orientation
        self._cos_ap_orientations = torch.cos(self.ap_orientations)

        # sine of ap orientation
        self._sin_ap_orientations = torch.sin(self.ap_orientations)

        # number of APs
        self._n_aps = self._ap_locations.shape[0]

    @property
    def ap_locations(self):
        return self._ap_locations

    @property
    def ap_orientations(self):
        return self._ap_orientations

    @property
    def cos_ap_orientations(self):
        return self._cos_ap_orientations

    @property
    def sin_ap_orientations(self):
        return self._sin_ap_orientations

    @property
    def n_aps(self):
        return self._n_aps

@dataclass
class LoggerParameters:
    """Dataclass to hold parameters for the logger.

    Ref: https://lightning.ai/docs/pytorch/1.9.1/api/pytorch_lightning.loggers.comet.html?highlight=comet
    """
    # experiment name
    experiment_name: str

    # api key. Do not set this value in LoggerParameters constructor. Instead, set the COMET_API_KEY environment variable.
    api_key: str = field(init=False)

    # workspace. Do not set this value in LoggerParameters constructor. Instead, set the COMET_WORKSPACE environment variable.
    workspace: str = field(init=False)

    # project name
    project_name: str = "ml-model-training"

    # directory to save log
    save_dir: str = "./logs"


    def __post_init__(self):
        if os.getenv('COMET_API_KEY'):
            self.api_key = os.getenv('COMET_API_KEY')
        else:
            raise ValueError("Please set the COMET_API_KEY environment variable by running `export COMET_API_KEY=<your_api_key>`.")

        if os.getenv('COMET_WORKSPACE'):
            self.workspace = os.getenv('COMET_WORKSPACE')
        else:
            raise ValueError("Please set the COMET_WORKSPACE environment variable by running `export COMET_WORKSPACE=<your_workspace_name>`.")

        # add datetime to experiment name
        self.experiment_name = f"{datetime.now().strftime('%Y-%m-%d_%H%M%S')}:{self.experiment_name}"

@dataclass
class ModelOutput:
    # cos of aoa in AP frame, shape (batch_size, n_aps)
    cos_aoa: torch.Tensor

    # sin of aoa in AP frame, shape (batch_size, n_aps)
    sin_aoa: torch.Tensor

    # AoA prediction in AP frame, shape (batch_size, n_aps)
    aoa: torch.Tensor

    # location prediction, shape (batch_size, 2)
    location: torch.Tensor

    # confidence for aoa prediction, shape (batch_size, n_aps)
    confidence: Optional[torch.Tensor] = None


@dataclass
class GTlabel:
    # cos of aoa in AP frame, shape (batch_size, n_aps)
    cos_aoa: torch.Tensor

    # sin of aoa in AP frame, shape (batch_size, n_aps)
    sin_aoa: torch.Tensor

    # aoa ground truth value in AP frame, shape (batch_size, n_aps)
    aoa: torch.Tensor

    # location prediction, shape (batch_size, 2)
    location: torch.Tensor


@dataclass
class LossTerms:
    # total loss
    total_loss: torch.Tensor

    # loss for cos of AoA
    cos_loss: torch.Tensor

    # loss for sin of AoA
    sin_loss: torch.Tensor

    # loss for location prediction
    location_loss: torch.Tensor


@dataclass
class AoAVisualizationSample:
    # AoA tof plots, shape (n_aps, H, W)
    aoa_tof_plots: torch.Tensor

    # AoA ground truth, shape (n_aps,)
    aoa_gt: torch.Tensor

    # AoA prediction, shape (n_aps,)
    aoa_pred: torch.Tensor

    # confidence prediction, shape (n_aps,). This is optional value.
    confidence: torch.Tensor = None

    def __post_init__(self):
        # Check that the first dimension of all tensors match
        n_aps = self.aoa_tof_plots.shape[0]
        if not (self.aoa_gt.shape[0] == n_aps and
                self.aoa_pred.shape[0] == n_aps and
                (self.confidence is None or self.confidence.shape[0] == n_aps)):
            error_message = (
                f"The first dimension of aoa_gt, aoa_pred, and confidence (if provided) must match the first dimension of aoa_tof_plots.\n"
                f"aoa_tof_plots shape: {self.aoa_tof_plots.shape}\n"
                f"aoa_gt shape: {self.aoa_gt.shape}\n"
                f"aoa_pred shape: {self.aoa_pred.shape}\n"
            )
            if self.confidence is not None:
                error_message += f"confidence shape: {self.confidence.shape}\n"

            raise ValueError(error_message)
