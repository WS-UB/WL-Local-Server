#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May  4 12:18:28 2022

@author: wcsng-uloc
"""
import importlib
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from os.path import join
sys.path.append("..")
import experiment
import io_utils
from channels import *
from exp_utils import *
from transforms import *
sys.path.append(join(io_utils.utilities_dir, "python"))
from revloc_utils import *

##### FILL OUT INFO BEFORE RUNNING #####
data_dir = "/home/wcsng-uloc/datasets/atkin4-3-4-22" 
profs = "svdfft"
save_guassian_labels = True
save_dir = join(data_dir, "dloc_s2")
sample_size = 3
########################################

#%% GETS ALL NECCESARY DATA POINTS
labels, yaws = load_bot_poses(data_dir)

bag_folder = join(data_dir, "cartographer_data", "bag")
img = np.flipud(plt.imread(join(bag_folder, "map.pgm")))
with open(join(bag_folder, "map.yaml"), 'rb') as f:
   data = yaml.load(f, Loader=yaml.CLoader)
   res = data['resolution']
   origin = data['origin']
x_vals = np.arange(0,img.shape[1], step=sample_size)* res + (res * round(origin[0] / res))
y_vals = np.arange(0,img.shape[0], step=sample_size)* res + (res * round(origin[1] / res))

#%% GENERATES GUASSIAN LABELS
# labels_guassian = np.zeros((labels.shape[0], x_vals.shape[0], y_vals.shape[0]))
labels_guassian = get_gaussian_labels(labels, x_vals, y_vals)
#%% SAVES GUASSIAN LABELS TO SAVE_DIR
if save_guassian_labels:
    if not os.path.isdir(save_dir):
       os.mkdir(save_dir)
    print(f"Saving File at: {save_dir}, File Size: {labels_guassian.shape}")
    np.savez_compressed(join(save_dir, "guassian_labels"), labels_guassian = labels_guassian)