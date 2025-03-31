#!/usr/bin/python
"""
Modified data loader for DLoc network that adapts to the new dataset structure
while maintaining compatibility with train_and_test.py

Input: HDF5 file with different field names
Output: features_wo_offset, features_w_offset, labels_gaussian_2d tensors
"""
import torch
import h5py
import numpy as np

def load_data(filename):
    """
    Load data from HDF5 file and convert to DLoc-compatible format
    
    Args:
        filename (str): Path to HDF5 file
        
    Returns:
        tuple: (features_wo_offset, features_w_offset, labels_gaussian_2d) as torch.Tensor
    """
    print(f'Loading {filename}')
    
    try:
        with h5py.File(filename, 'r') as f:
            # Extract and process features
            features_2d = np.array(f['features_2d'], dtype=np.float32)
            features_2d_spotfi = np.array(f['features_2d_spotfi'], dtype=np.float32)
            labels = np.array(f['labels'], dtype=np.float32)
            
            # Reshape features to expected format
            # Original: (N, C) where N is samples, C is channels
            # Current: (401, 315, 4, 1) - assuming 401 samples, 315 timesteps?
            # We'll take mean across timesteps (axis=1) and remove last dimension
            features_2d = np.squeeze(np.mean(features_2d, axis=1))
            features_2d_spotfi = np.squeeze(np.mean(features_2d_spotfi, axis=1))
            
            # Convert labels to 2D Gaussian format
            # Original expects (N, 2) shape
            labels_gaussian = np.tile(labels.T, (features_2d.shape[0], 1))
            
            # Create torch tensors
            features_wo_offset = torch.tensor(features_2d, dtype=torch.float32)
            features_w_offset = torch.tensor(features_2d_spotfi, dtype=torch.float32)
            labels_gaussian_2d = torch.tensor(labels_gaussian, dtype=torch.float32)
            
            print(f"Loaded shapes - wo_offset: {features_wo_offset.shape}, "
                  f"w_offset: {features_w_offset.shape}, "
                  f"labels: {labels_gaussian_2d.shape}")
            
            return features_wo_offset, features_w_offset, labels_gaussian_2d
            
    except Exception as e:
        print(f"Error loading {filename}: {str(e)}")
        raise

# For testing the loader directly
if __name__ == "__main__":
    try:
        wo, w, lbl = load_data('./data/1.h5')
        print("\nSample features_wo_offset:", wo[0])
        print("Sample features_w_offset:", w[0])
        print("Sample labels:", lbl[0])
    except Exception as e:
        print(f"Test failed: {str(e)}")