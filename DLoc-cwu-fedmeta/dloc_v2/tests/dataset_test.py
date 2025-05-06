# import sys
# import os
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))  
# # sys.path.append(os.path.abspath("../"))

# from dataset import DLocDatasetV2

# #change the output length
# import torch
# torch.set_printoptions(threshold=10000)
# import numpy as np
# np.set_printoptions(threshold=10000)

# if __name__ == "__main__":
#     dataset = DLocDatasetV2(parquet_file_path="../data/2025-04-01 17_57_32.397.parquet")
    

#     print(f"Number of samples: {len(dataset)}")
#     csi_data, gps_data = dataset[0]
#     print(f"CSI data shape: {csi_data.shape}, GPS data shape: {gps_data.shape}")
#     print(f"First CSI tensor: {csi_data}")
#     print(f"First GPS tensor: {gps_data}")

#     all_csi, all_gps = DLocDatasetV2.process_parquet_file("../data/2025-04-01 17_57_32.397.parquet")
#     print(f"Total CSI samples: {len(all_csi)}, Total GPS samples: {len(all_gps)}")
#     print(f"First CSI sample shape: {all_csi[0].shape}, First GPS sample shape: {all_gps[0].shape}")
#     print(f"First CSI sample: {all_csi[0]}")
#     print(f"First GPS sample: {all_gps[0]}")



import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))  

from dataset import DLocDatasetV2
from dataset import fetch_selected_parquet_from_minio

#change the output length
import torch
torch.set_printoptions(threshold=10000)
import numpy as np
np.set_printoptions(threshold=10000)

if __name__ == "__main__":
    parquet_path = fetch_selected_parquet_from_minio()

    if parquet_path:
        dataset = DLocDatasetV2(parquet_file_path=parquet_path)

        print(f"Number of samples: {len(dataset)}")
        csi_data, gps_data = dataset[0]
        print(f"CSI data shape: {csi_data.shape}, GPS data shape: {gps_data.shape}")
        print(f"First CSI tensor: {csi_data}")
        print(f"First GPS tensor: {gps_data}")

        all_csi, all_gps = DLocDatasetV2.process_parquet_file(parquet_path)
        print(f"Total CSI samples: {len(all_csi)}, Total GPS samples: {len(all_gps)}")
        print(f"First CSI sample shape: {all_csi[0].shape}, First GPS sample shape: {all_gps[0].shape}")
        print(f"First CSI sample: {all_csi[0]}")
        print(f"First GPS sample: {all_gps[0]}")
    else:
        print("❌ No file selected or download failed.")