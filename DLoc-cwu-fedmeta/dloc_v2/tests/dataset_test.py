import sys
import os
sys.path.append(os.path.abspath("../"))
from dataset import DLocDatasetV2

if __name__ == "__main__":
    # Example usage
    dataset = DLocDatasetV2(parquet_file_path="../data/sample.parquet")
    print(f"Number of samples: {len(dataset)}")
    csi_data, gps_data = dataset[0]
    print(f"CSI data shape: {csi_data.shape}, GPS data shape: {gps_data.shape}")
    
    # Alternative usage to get all data at once
    all_csi, all_gps = DLocDatasetV2.process_parquet_file("../data/sample.parquet")
    print(f"Total CSI samples: {len(all_csi)}, Total GPS samples: {len(all_gps)}")
    print(f"First CSI sample shape: {all_csi[0].shape}, First GPS sample shape: {all_gps[0].shape}")
    print(f"First CSI sample: {all_csi[0]}")
    print(f"First GPS sample: {all_gps[0]}")