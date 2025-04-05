#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 27 16:46:32 2022

@author: wcsng-uloc
"""
import matplotlib.animation as ani
import importlib
import matplotlib.pyplot as plt
import scipy.signal as signal
from detecta import detect_peaks
import numpy as np
import sys
import tqdm
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

pl = importlib.import_module("plotting_utils")

data_dir = "/home/wcsng-uloc/datasets/atkin4-3-4-22"
cli = "204-pixel"
algo = "svdfft-10"
profs = "svdfft"
choice = "max-peak"
save_channels_wo_offset = True
ap = 204

#%%
d_vals = devices.d_table['qtn']
labels, yaws = load_bot_poses(data_dir)
bag_folder = join(data_dir, "cartographer_data", "bag")
img = np.flipud(plt.imread(join(bag_folder, "map.pgm")))
with open(join(bag_folder, "map.yaml"), 'rb') as f:
   data = yaml.load(f, Loader=yaml.CLoader)
   res = data['resolution']
   origin = data['origin']
x_vals = np.arange(0,img.shape[1], step = 2)* res + (res * round(origin[0] / res))
y_vals = np.arange(0,img.shape[0], step = 2)* res + (res * round(origin[1] / res))

n_points = (labels.shape[0], 4)
channels = np.load(join(data_dir, "channels", cli + ".npz"))
h = channels['H']
FREQ = valid_subfreq(channels['chan'][0], 20e6)
lambda_val = 3e8 / FREQ

channels_wo_offset_all = np.zeros(h.shape, dtype = 'complex_')
features_wo_offset_all = np.zeros((h.shape[0], x_vals.shape[0], y_vals.shape[0]))
gt_tof = np.zeros(n_points)
#%%
pbar = tqdm.trange(n_points[0] * n_points[1])

for i in range(n_points[0]):
    for j in range(4):
        pbar.update(1)
        P_tof = compute_distance_profile_music_fast(np.squeeze(h[i,:, j, :]), lambda_val, 2, d_vals, .1)
        thresh = 0.5 * np.max(np.abs(P_tof))
        peaks_tof = detect_peaks(np.abs(P_tof), threshold = .5)
        if peaks_tof.shape[0] == 0:
            peaks_tof = np.append(peaks_tof, np.where(P_tof == np.max(np.abs(P_tof))))

        channels_wo_offset_all[i, :, j, :] = np.squeeze(h[i,:,j,:]).T @ np.exp(1j * 2 * np.pi * FREQ.T * (d_vals[peaks_tof[0]] - gt_tof[i,j])/3e8) 
        # features_wo_offset_all[i,:, :] = comp_svdfft(h_new)

#%%
save_dir = join(data_dir, "atkin4-3-4-22_dloc", str(ap))
if save_channels_wo_offset:
    if not os.path.isdir(save_dir):
       os.mkdir(save_dir)
    print(f"Saving File at: {save_dir}, File Size: {channels_wo_offset_all.shape}")
    np.save(join(save_dir, "channels_wo_offset"), channels_wo_offset_all)
    