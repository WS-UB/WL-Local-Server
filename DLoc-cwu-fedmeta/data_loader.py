#!/usr/bin/python
'''
Data Loading Code for the DLoc dataset
Has a simpler dataloading pipeline for loading
Legacy mat file chunks, and
Custom DataLoader class DLocDataset that loads instances
from the folder as required into the memory.
'''
import os
import torch
import h5py
import scipy.io
import numpy as np
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils

#######################Legacy DataLoader########################
# A simple data loader that imports the train and test mat files
# from the `filename` and converts them to torch.tensors()
# to be loaded for training and testing DLoc network
# `features_wo_offset`: targets for the consistency decoder
# `features_w_offset` : inputs for the network/encoder
# `labels_gaussian_2d`: targets for the location decoder
# Demands multiple of 10s of Gigabytes of system memory
def load_data(filename):
    print('Loading '+filename)
    arrays = {}
    f = h5py.File(filename,'r')
    features_wo_offset = torch.tensor(np.transpose(np.array(f.get('features_wo_offset'), dtype=np.float32)), dtype=torch.float32)
    features_w_offset = torch.tensor(np.transpose(np.array(f.get('features_w_offset'), dtype=np.float32)), dtype=torch.float32)
    labels_gaussian_2d = torch.tensor(np.transpose(np.array(f.get('labels_gaussian_2d'), dtype=np.float32)), dtype=torch.float32)
        
    return features_wo_offset,features_w_offset, labels_gaussian_2d
    
#######################Custom DataLoader########################
# Demands only a few Gigabytes of memory
# `features_wo_offset`: targets for the consistency decoder
# `features_w_offset` : inputs for the network/encoder
# `labels_gaussian_2d`: targets for the location decoder
class DLocDataset(Dataset):
    """DLoc dataset."""

    def __init__(self, root_dir, transform=None):
        """
        Args:
            root_dirs (list of strings): List of Directories with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.root_dir = root_dir
        # print(self.root_dir)
        self.transform = transform
        self.n_files = []
        self.file_names = []
        for n,i in enumerate(self.root_dir):
            print(n, i)
            onlyfiles = next(os.walk(i))[2] #dir is your directory path as string
            self.file_names.append([onlyfiles])
            self.n_files.append(len(onlyfiles))

    def __len__(self):
        return np.sum(self.n_files)

    def __getitem__(self, idx):

        if torch.is_tensor(idx):
            idx_list = idx.tolist()
        elif isinstance(idx, int):
        	idx_list = []
        	idx_list.append(idx) 
        else:
	       	idx_list = idx


        

        file_num = 0
        count = 0
        for dir_id, n_file in enumerate(self.n_files):
            if idx_list[0] > (count+n_file):
                count += n_file
                continue
            elif idx_list[-1] <= (count+n_file-1):
                for i in idx_list:
                    fname = self.file_names[dir_id][0][i-count]
                    filename = os.path.join(self.root_dir[dir_id], fname)
                    # print(filename)
                    f = h5py.File(filename,'r')
                    f_wo_offset = np.transpose(np.array(f.get('features_wo_offset'), dtype=np.float32))
                    f_w_offset = np.transpose(np.array(f.get('features_w_offset'), dtype=np.float32))
                    labels_2d = np.transpose(np.array(f.get('labels_gaussian_2d'), dtype=np.float32))
                    
                    if ( file_num == 0) :
                        features_wo_offset = f_wo_offset
                        features_w_offset = f_w_offset
                        labels_gaussian_2d = labels_2d
                        file_num += 1
                    else:
                        features_wo_offset = np.concatenate((features_wo_offset, f_wo_offset),axis=0)
                        features_w_offset = np.concatenate((features_w_offset, f_w_offset),axis=0)
                        labels_gaussian_2d = np.concatenate((labels_gaussian_2d, labels_2d),axis=0)
                #     print(np.shape(labels_gaussian_2d))
                #     print(np.shape(features_wo_offset))
                #     print(np.shape(features_w_offset))
                # count += n_file
                break
            elif idx_list[-1] > (count+n_file-1):
                for i in range(idx_list[0],count+n_file-1):
#                     fname = str(i-count+1)+'.h5'
                    fname = self.file_names[dir_id][0][i-count]
                    filename = os.path.join(self.root_dir[dir_id], fname)
                    # print(filename)
                    f = h5py.File(filename,'r')
                    f_wo_offset = np.transpose(np.array(f.get('features_wo_offset'), dtype=np.float32))
                    f_w_offset = np.transpose(np.array(f.get('features_w_offset'), dtype=np.float32))
                    labels_2d = np.transpose(np.array(f.get('labels_gaussian_2d'), dtype=np.float32))
                    
                    if ( file_num == 0) :
                        features_wo_offset = f_wo_offset
                        features_w_offset = f_w_offset
                        labels_gaussian_2d = labels_2d
                        file_num += 1
                    else:
                        features_wo_offset = np.concatenate((features_wo_offset, f_wo_offset),axis=0)
                        features_w_offset = np.concatenate((features_w_offset, f_w_offset),axis=0)
                        labels_gaussian_2d = np.concatenate((labels_gaussian_2d, labels_2d),axis=0)
                count += n_file
                idx_list = np.array(idx_list)
                idx_list = idx_list[np.array(idx_list)>=count].tolist()
        features_wo_offset = np.squeeze(features_wo_offset)
        features_w_offset = np.squeeze(features_w_offset)
        if(len(np.shape(labels_gaussian_2d))==3):
            if(np.shape(labels_gaussian_2d)[0]!=1):
                labels_gaussian_2d = np.expand_dims(labels_gaussian_2d, axis=1)
        sample = {'features_wo_offset': features_wo_offset, 'features_w_offset': features_w_offset, 'labels_gaussian_2d': labels_gaussian_2d}

        if self.transform:
            sample = self.transform(sample)

        return sample

class ToTensor(object):
    """Convert ndarrays in sample to Tensors."""

    def __call__(self, sample):
        features_wo_offset, features_w_offset, labels_gaussian_2d = sample['features_wo_offset'], sample['features_w_offset'], sample['labels_gaussian_2d']

        # swap color axis because
        # numpy image: H x W x C
        # torch image: C x H x W
        return {'labels_gaussian_2d': torch.from_numpy(labels_gaussian_2d),
                'features_wo_offset': torch.from_numpy(features_wo_offset),
                'features_w_offset': torch.from_numpy(features_w_offset)}