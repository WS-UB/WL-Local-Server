import pandas as pd
import pyarrow.parquet as pq
import torch


def normalize_gps(gps_data):
    """
    Read GPS data from parquet file and normalize coordinates relative to davis_box.

    Args:
        parquet_file_path: Path to the parquet file containing GPS data

    Returns:
        Tuple of (lat_new, long_new) normalized coordinates
    """
    return compute_normalize_gps(gps_data["latitude"], gps_data["longitude"])


def compute_normalize_gps(latitude, longitude):
    davis_box = [
        (43.00297463348004, -78.78674402431034),
        (43.00242950065132, -78.7868579571325),
        (43.00240355874723, -78.78777435710242),
        (43.0030251198603, -78.78781856606011),
    ]

    # Calculate min/max bounds from davis_box
    lats = [point[0] for point in davis_box]
    longs = [point[1] for point in davis_box]
    lat_min, lat_max = min(lats), max(lats)
    long_min, long_max = min(longs), max(longs)

    try:

        # Compute normalized coordinates
        lat_new = (latitude - lat_min) / (lat_max - lat_min)
        long_new = (longitude - long_min) / (long_max - long_min)

        return lat_new, long_new

    except Exception as e:
        print(f"Error processing file: {str(e)}")
        return None, None


# def reverse_normalization(latitude, longitude):
#     """
#     Reverse the normalization of GPS coordinates.

#     Args:
#         latitude: Normalized latitude
#         longitude: Normalized longitude

#     Returns:
#         Tuple of (latitude, longitude) in original coordinates
#     """
#     davis_box = [
#         (43.00297463348004, -78.78674402431034),
#         (43.00242950065132, -78.7868579571325),
#         (43.00240355874723, -78.78777435710242),
#         (43.0030251198603, -78.78781856606011)
#     ]

#     # Calculate min/max bounds from davis_box
#     lats = [point[0] for point in davis_box]
#     longs = [point[1] for point in davis_box]
#     lat_min, lat_max = min(lats), max(lats)
#     long_min, long_max = min(longs), max(longs)

#     # Compute original coordinates
#     lat_orig = latitude * (lat_max - lat_min) + lat_min
#     long_orig = longitude * (long_max - long_min) + long_min

#     return lat_orig, long_orig


def reverse_normalization(latitude, longitude):
    """
    Reverse the normalization of GPS coordinates.

    Args:
        latitude: Normalized latitude (tensor)
        longitude: Normalized longitude (tensor)

    Returns:
        Stacked tensor of (latitude, longitude) in original coordinates with shape (N, 2)
    """
    davis_box = [
        (43.00297463348004, -78.78674402431034),
        (43.00242950065132, -78.7868579571325),
        (43.00240355874723, -78.78777435710242),
        (43.0030251198603, -78.78781856606011),
    ]

    # Calculate min/max bounds from davis_box
    lats = [point[0] for point in davis_box]
    longs = [point[1] for point in davis_box]
    lat_min, lat_max = min(lats), max(lats)
    long_min, long_max = min(longs), max(longs)

    # Compute original coordinates
    lat_orig = latitude * (lat_max - lat_min) + lat_min
    long_orig = longitude * (long_max - long_min) + long_min

    # Stack into (N, 2) tensor
    return torch.stack([lat_orig, long_orig], dim=-1)
