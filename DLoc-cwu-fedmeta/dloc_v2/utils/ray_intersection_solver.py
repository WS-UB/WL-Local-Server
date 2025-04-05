"""Line intersection solver"""

import torch

def solve_ray_intersection(
    ap_xy: torch.Tensor, theta: torch.Tensor, confidence_score: torch.Tensor = None
) -> torch.Tensor:
    """
    Solves the intersection point of n rays in a 2D plane. The rays start from the locations of the Access Points (APs),
    and their angles (in map frame, w.r.t to horizontal axis) are given by theta, which represents the Angle of Arrival (AoA)
    in the map frame. The confidence score indicates the importance of each AoA measurement in the least squares problem.
    Reference: https://www.notion.so/DLoc-Design-37e43b97634d4c848b237ae06fe08ac4?pvs=4

    Args:
        ap_xy: APs location in map frame. A tensor of shape (n_ap, 2).
        theta: Angle of rays with respect to the horizontal axis, which is AoA of each AP in map frame. A tensor of shape (n_ap,).
        confidence_score: AoA Confidence score that determines the imporance of AoA measurment in least square porblem.
            A tensor of shape (n_ap,).

    Returns:
        The intersection point of the rays. A tensor of shape (2,).
    """
    assert (
        ap_xy.shape[0] == theta.shape[0]
    ), "Shape mismatch."

    if confidence_score is None:
        confidence_score = torch.ones_like(theta)
    else:
        assert (
            ap_xy.shape[0] == confidence_score.shape[0]
        ), "Shape mismatch."
        # Assert that confidence_score is not all zeros
        assert torch.any(confidence_score), "Confidence score tensor must not be all zeros."

    # Compute the elements of the matrix
    cos_theta = torch.cos(theta)
    sin_theta = torch.sin(theta)

    # Matrix elements
    A11 = torch.sum(confidence_score * (1 - cos_theta**2))
    A12 = -torch.sum(confidence_score * cos_theta * sin_theta)
    A21 = A12
    A22 = torch.sum(confidence_score * (1 - sin_theta**2))

    # Construct the matrix
    A = torch.tensor([[A11, A12], [A21, A22]])

    # Compute the elements of the right-hand side vector
    x_i = ap_xy[:, 0]
    y_i = ap_xy[:, 1]

    b1 = torch.sum(confidence_score * ((1 - cos_theta**2) * x_i - cos_theta * sin_theta * y_i))
    b2 = torch.sum(confidence_score * ((1 - sin_theta**2) * y_i - cos_theta * sin_theta * x_i))

    # Construct the right-hand side vector
    b = torch.tensor([b1, b2])

    # Solve the linear system Ax = b
    solution = torch.linalg.solve(A, b)

    return solution


def solve_ray_intersection_batch(
    ap_xy: torch.Tensor,
    cos_theta: torch.Tensor,
    sin_theta: torch.Tensor,
    confidence_score: torch.Tensor
) -> torch.Tensor:
    """
    See docstring from above function for more details. This function performs the same calculation in batch mode.
    Reference: https://www.notion.so/DLoc-Design-37e43b97634d4c848b237ae06fe08ac4?pvs=4

    Args:
        ap_xy: AP locations in map frame. A tensor of shape (n_ap, 2).
        cos_theta: Cos of AoA measurement in map frame. A tensor of shape (batch_size, n_ap).
        sin_theta: Sin of AoA measurement in map frame. A tensor of shape (batch_size, n_ap).
        confidence_score: AoA Confidence score that determines the importance of AoA measurement in least square problem.
            A tensor of shape (batch_size, n_ap).

    Returns:
        The intersection point of the rays formed by AoA from each AP. A tensor of shape (batch_size, 2).
    """
    assert ap_xy.shape[1] == 2, "This function solves the intersection point of rays in 2D plane."
    batch_size = cos_theta.shape[0]
    n_ap = ap_xy.shape[0]
    assert cos_theta.shape == (batch_size, n_ap), f"Shape mismatch. Expected {(batch_size, n_ap)}, but got {cos_theta.shape}."
    assert sin_theta.shape == cos_theta.shape, "Shape mismatch."
    assert confidence_score.shape == cos_theta.shape, "Shape mismatch."
    assert ap_xy.shape == (n_ap, 2), "Shape mismatch."

    # Matrix elements, each element has shape (batch_size, 1)
    A11 = torch.sum(confidence_score * (1 - cos_theta**2), dim=1).unsqueeze(1)
    A12 = -torch.sum(confidence_score * cos_theta * sin_theta, dim=1).unsqueeze(1)
    A21 = A12
    A22 = torch.sum(confidence_score * (1 - sin_theta**2), dim=1).unsqueeze(1)

    # Construct the matrix A, shape is (batch_size, 2, 2)
    A = torch.cat((A11, A12, A21, A22), dim=1).view(batch_size, 2, 2)

    # Compute the elements of the right-hand side vector, each element has shape (batch_size, 1)
    x_i = ap_xy[:, 0] # shape is (n_ap,)
    y_i = ap_xy[:, 1] # shape is (n_ap,)

    b1 = torch.sum(confidence_score * ((1 - cos_theta**2) * x_i - cos_theta * sin_theta * y_i), dim=1).unsqueeze(1)
    b2 = torch.sum(confidence_score * ((1 - sin_theta**2) * y_i - cos_theta * sin_theta * x_i), dim=1).unsqueeze(1)

    # Construct the right-hand side matrix, shape is (batch_size, 2)
    b = torch.cat((b1, b2), dim=1)

    # Solve the linear system Ax = b
    # A.shape = (batch_size, 2, 2), b.shape = (batch_size, 2)
    # solution.shape = (batch_size, 2)
    solution = torch.linalg.solve(A, b)

    return solution
