import h5py
import numpy as np
import torch

def load_h5_file(filepath):
    """
    Load and inspect an HDF5 file for DLoc network compatibility
    
    Args:
        filepath (str): Path to the .h5 file (e.g., './data/1.h5')
    
    Returns:
        dict: Dictionary containing all datasets from the file
    """
    try:
        with h5py.File(filepath, 'r') as f:
            print(f"Successfully opened {filepath}")
            print(f"File contents: {list(f.keys())}")
            
            data = {}
            for key in f.keys():
                # Convert to numpy array first for proper handling
                arr = np.array(f[key])  
                print(f"\nDataset: {key}")
                print(f"Shape: {arr.shape}")
                print(f"Data type: {arr.dtype}")
                print(f"Sample values:\n{arr[:2] if len(arr.shape) > 0 else arr}")  # Show first 2 samples
                
                # Store as torch tensor (matching data_loader.py behavior)
                data[key] = torch.tensor(arr, dtype=torch.float32)
            
            return data
            
    except Exception as e:
        print(f"\nError loading {filepath}:")
        print(f"Type: {type(e).__name__}")
        print(f"Details: {str(e)}")
        return None

# Example usage
if __name__ == "__main__":
    h5_data = load_h5_file('./data/1.h5')
    
    if h5_data:
        print("\nVerifying DLoc-required datasets:")
        required = ['features_wo_offset', 'features_w_offset', 'labels_gaussian_2d']
        
        for req in required:
            if req in h5_data:
                print(f"✓ Found {req} with shape {h5_data[req].shape}")
            else:
                print(f"✗ Missing required dataset: {req}")