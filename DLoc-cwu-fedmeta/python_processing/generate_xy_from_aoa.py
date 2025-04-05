#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 13 17:48:53 2022

@author: wcsng-uloc
"""
import importlib
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import tqdm
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
ap = 204
algo = 'svdfft'
save_dir = join(data_dir, "dloc_s2", str(ap))
save_file_name = "features_w_offset"
profile_dir = join(data_dir, "profiles", algo)
cli = str(ap) + "-pixel"
save = True
sample_size = 3
########################################

#%%
theta_vals = devices.theta_table['qtn']
d_vals = devices.d_table['qtn']
labels, yaws = load_bot_poses(data_dir)

TX_AP = get_property(data_dir, 'tx')

ap_file, ap_dict, _ = load_ap_info(data_dir)
comps = cli.split('-')
cli_ap = comps[0] if comps[0] != TX_AP else comps[1]
cli_pos = ap_file[cli_ap]

bag_folder = join(data_dir, "cartographer_data", "bag")
img = np.flipud(plt.imread(join(bag_folder, "map.pgm")))
with open(join(bag_folder, "map.yaml"), 'rb') as f:
   data = yaml.load(f, Loader=yaml.CLoader)
   res = data['resolution']
   origin = data['origin']
x_vals = np.arange(0,img.shape[1], step=sample_size)* res + (res * round(origin[0] / res))
y_vals = np.arange(0,img.shape[0], step=sample_size)* res + (res * round(origin[1] / res))

#%%
n_pkts = labels.shape[0]
xy_prof = np.zeros((n_pkts, x_vals.shape[0], y_vals.shape[0]))

for i in tqdm.trange(n_pkts):
    prof = np.load(join(profile_dir, f"{cli}-{i}.npz"))['prof']
    xy_prof[i, :, :] = profile_polar2cartesian(prof[0], theta_vals , d_vals, x_vals, y_vals, cli_pos[:, :2])  

#%%
if save:
    if not os.path.isdir(save_dir):
       os.mkdir(save_dir)
    np.savez_compressed(join(save_dir, save_file_name), xy_prof)
