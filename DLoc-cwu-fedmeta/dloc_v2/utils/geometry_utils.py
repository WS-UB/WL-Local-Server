"""Utility functions related to geometry."""
import torch
from typing import Union
import numpy as np

def angle_to_so2(angle: Union[float, torch.Tensor]) -> torch.Tensor:
    """Converts an angle to an SO(2) rotation matrix.

    Args:
        angle: The angle in radians. Single scalar value.

    Returns:
        The corresponding SO(2) rotation matrix.
    """
    cos_angle = torch.cos(angle)
    sin_angle = torch.sin(angle)
    rotation_matrix = torch.Tensor([
        [cos_angle, -sin_angle],
        [sin_angle, cos_angle]
    ])
    return rotation_matrix

def so2_to_angle(rotation_matrix: torch.Tensor) -> float:
    """
    Converts an SO(2) rotation matrix back to an angle.

    Args:
        rotation_matrix: The SO(2) rotation matrix.

    Returns:
        The corresponding angle in radians.
    """
    assert rotation_matrix.shape == (2, 2)
    angle = torch.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return angle


def cos_angle_sum(cos_angle1: Union[float, torch.Tensor],
                  sin_angle1: Union[float, torch.Tensor],
                  cos_angle2: Union[float, torch.Tensor],
                  sin_angle2: Union[float, torch.Tensor],) -> torch.Tensor:
    """Compute the cosine of the sum of two angles given cos and sine of the angles.

    cos(a+b) = cos(a)cos(b) - sin(a)sin(b)

    Args:
        cos_angle1: The cosine value of first angle, cos(a).
        sin_angle1: The sine value of first angle, sin(a).
        cos_angle2: The cosine value of second angle, cos(b).
        sin_angle2: The sine value of second angle, sin(b).

    Returns:
        The cosine of the sum of the two angles, cos(a+b).
    """
    return cos_angle1 * cos_angle2 - sin_angle1 * sin_angle2


def sin_angle_sum(cos_angle1: Union[float, torch.Tensor],
                  sin_angle1: Union[float, torch.Tensor],
                  cos_angle2: Union[float, torch.Tensor],
                  sin_angle2: Union[float, torch.Tensor],) -> torch.Tensor:
    """Compute the sine of the sum of two angles given cos and sine of the angles.

    sin(a+b) = sin(a)cos(b) + cos(a)sin(b)

    Args:
        cos_angle1: The cosine value of first angle, cos(a).
        sin_angle1: The sine value of first angle, sin(a).
        cos_angle2: The cosine value of second angle, cos(b).
        sin_angle2: The sine value of second angle, sin(b).

    Returns:
        The sine of the sum of the two angles, sin(a+b).
    """
    return sin_angle1 * cos_angle2 + cos_angle1 * sin_angle2


def wrap_to_pi(angle: Union[float, torch.Tensor]) -> torch.Tensor:
    """Wrap the angle to the range of [-pi, pi].

    Args:
        angle: The angle in radians. Single scalar value.

    Returns:
        The angle wrapped to the range of [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi
