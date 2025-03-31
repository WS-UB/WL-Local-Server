#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May  6 16:47:11 2022

@author: wcsng-uloc
"""

import importlib
import time
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import matplotlib
import numpy as np
import sys
import os
from os.path import join
sys.path.append("..")
import experiment
import io_utils
import tqdm
from channels import *
from exp_utils import *
from transforms import *
sys.path.append(join(io_utils.utilities_dir, "python"))
from revloc_utils import *

pl = importlib.import_module("plotting_utils")

data_dir = "/home/wcsng-uloc/datasets/atkin4-3-4-22"
features_dir = '/home/wcsng-uloc/datasets/atkin4-3-4-22/atkin4-3-4-22_dloc'
cli = ["201-pixel", "203-pixel", "204-pixel"]
algo = "svdfft-10"
profs = "svdfft"
choice = "max-peak"
ap = [201, 203, 204] 

#%%

TX_AP = get_property(data_dir, 'tx')

profile_dir = join(data_dir, "profiles", profs)

theta_vals = devices.theta_table['qtn']
d_vals = devices.d_table['qtn']

labels, yaws = load_bot_poses(data_dir)
times = load_times(data_dir)


bag_folder = join(data_dir, "cartographer_data", "bag")
img = np.flipud(plt.imread(join(bag_folder, "map.pgm")))
with open(join(bag_folder, "map.yaml"), 'rb') as f:
   data = yaml.load(f, Loader=yaml.CLoader)
   res = data['resolution']
   origin = data['origin']
x_vals = np.arange(np.round(-.5 * img.shape[1]),np.round(1.5*img.shape[1]))* res + (2*res * round(origin[0] / (2*res)))
y_vals = np.arange(np.round(-.5 * img.shape[0]),np.round(1.5*img.shape[0]))* res + (2*res * round(origin[1] / (2*res)))

ap_file, ap_dict, _ = load_ap_info(data_dir)

comps = cli[0].split('-')
cli_ap = comps[0] if comps[0] != TX_AP else comps[1]
cli_pos_201 = ap_file[cli_ap]

comps = cli[1].split('-')
cli_ap = comps[0] if comps[0] != TX_AP else comps[1]
cli_pos_203 = ap_file[cli_ap]

comps = cli[2].split('-')
cli_ap = comps[0] if comps[0] != TX_AP else comps[1]
cli_pos_204 = ap_file[cli_ap]


#%%
fig, ((ax11, ax12, ax13), (ax21, ax22, ax23), (ax31, ax32, ax33)) = plt.subplots(3, 3, constrained_layout = True)

gnd_aoa = np.load(join(data_dir, "results", "svdfft-10-max-peak", "aoa_gnd_rad_fixed.npy"))


def animate(i):
    
    ### Clearing Scatter Plots
    """
    sc11 = ax11.scatter(0, 0)
    sc21 = ax21.scatter(0, 0)
    sc31 = ax31.scatter(0, 0)
    sc12 = ax12.scatter(0, 0)
    sc22 = ax22.scatter(0, 0)
    sc32 = ax32.scatter(0, 0)
    sc13 = ax13.scatter(0, 0)
    sc23 = ax23.scatter(0, 0)
    sc33 = ax33.scatter(0, 0)
     
    sc11.remove()
    sc21.remove()
    sc31.remove()
    sc12.remove()
    sc22.remove()
    sc32.remove()
    sc13.remove()
    sc23.remove()
    sc33.remove()
    """
    #### 
    
    
    fig.suptitle(f"DLoc Input Graphs: {i}")
    
    #######################################################################################
    # AP 1 Graphs
    prof = np.load(join(profile_dir, cli[0] + "-" + str(i) + ".npz"))['prof']
    print(f"Loading Profile {i}")
    xy_prof = profile_polar2cartesian(prof[0], theta_vals , d_vals, x_vals, y_vals, cli_pos_201[:, :2])
    
    xy_gt = labels[i, :] 
    aoa_gt, tof_gt = gnd_aoa[i, 0, :]*180/np.pi, np.linalg.norm(labels[i] - cli_pos_201[0])
    
    ax11.imshow(np.abs(prof[0]), origin='lower', zorder = -2, extent=(d_vals[0], d_vals[-1], theta_vals[0]*180/np.pi, theta_vals[-1]*180/np.pi))
    ax11.set_aspect((d_vals[-1]-d_vals[0])/((theta_vals[-1]-theta_vals[0])*180/np.pi))
    ax11.plot(tof_gt, aoa_gt, color = 'r', marker='+')
    ax11.title.set_text(f"AoA-ToF Plot of: AP 201")
    ax11.set_xlabel("ToF")
    ax11.set_ylabel('AoA')
    
    
    xy_w_offset = profile_polar2cartesian(prof[0], theta_vals , d_vals, x_vals, y_vals, cli_pos_201[:, :2])
    
    ax21.imshow(xy_w_offset.T, cmap='hot', interpolation='none',origin='lower', zorder = -2, extent=[x_vals[0], x_vals[-1], y_vals[0], y_vals[-1]])
    ax21.plot(xy_gt[0], xy_gt[1], color = 'w', marker='+')
    ax21.title.set_text("XY Heatmap with Regular Label")
    ax21.set_xlabel(f"x_vals({int(np.min(x_vals))}, {int(np.max(x_vals))})")
    ax21.set_ylabel(f"y_vals({int(np.min(y_vals))}, {int(np.max(y_vals))})")
    
    
    ax31.imshow(xy_prof.T, cmap='hot', interpolation='none',origin='lower', zorder = -2, extent=[x_vals[0], x_vals[-1], y_vals[0], y_vals[-1]])
    ax31.plot(xy_gt[0], xy_gt[1],  color = 'w', marker='+')
    ax31.title.set_text("XY Heatmap without Offsetl")
    ax31.set_xlabel(f"x_vals({int(np.min(x_vals))}, {int(np.max(x_vals))})")
    ax31.set_ylabel(f"y_vals({int(np.min(y_vals))}, {int(np.max(y_vals))})")
    
    #######################################################################################
    # AP 2 Graphs
    prof = np.load(join(profile_dir, cli[1] + "-" + str(i) + ".npz"))['prof']
    print(f"Profile Loaded: {cli}-{0}.npz")
    xy_prof = profile_polar2cartesian(prof[0], theta_vals , d_vals, x_vals, y_vals, cli_pos_203[:, :2])
    
    xy_gt = labels[i, :] 
    aoa_gt, tof_gt = gnd_aoa[i, 1, :]*180/np.pi, np.linalg.norm(labels[i] - cli_pos_203[0])
    
    ax12.imshow(np.abs(prof[0]), origin='lower', zorder = -2, extent=(d_vals[0], d_vals[-1], theta_vals[0]*180/np.pi, theta_vals[-1]*180/np.pi))
    ax12.set_aspect((d_vals[-1]-d_vals[0])/((theta_vals[-1]-theta_vals[0])*180/np.pi))
    ax12.plot(tof_gt, aoa_gt, color = 'r', marker='+')
    ax12.title.set_text(f"AoA-ToF Plot of: AP 203")
    ax12.set_xlabel("ToF")
    ax12.set_ylabel('AoA')
    
    
    xy_w_offset = profile_polar2cartesian(prof[0], theta_vals , d_vals, x_vals, y_vals, cli_pos_203[:, :2])
    
    ax22.imshow(xy_w_offset.T, cmap='hot', interpolation='none',origin='lower', zorder = -2, extent=[x_vals[0], x_vals[-1], y_vals[0], y_vals[-1]])
    ax22.plot(xy_gt[0], xy_gt[1],  color = 'w', marker='+')
    ax22.title.set_text("XY Heatmap with Regular Label")
    ax22.set_xlabel(f"x_vals({int(np.min(x_vals))}, {int(np.max(x_vals))})")
    ax22.set_ylabel(f"y_vals({int(np.min(y_vals))}, {int(np.max(y_vals))})")
    
    
    ax32.imshow(xy_prof.T, cmap='hot', interpolation='none',origin='lower', zorder = -2, extent=[x_vals[0], x_vals[-1], y_vals[0], y_vals[-1]])
    ax32.plot(xy_gt[0], xy_gt[1],  color = 'w', marker='+')
    ax32.title.set_text("XY Heatmap without Offset")
    ax32.set_xlabel(f"x_vals({int(np.min(x_vals))}, {int(np.max(x_vals))})")
    ax32.set_ylabel(f"y_vals({int(np.min(y_vals))}, {int(np.max(y_vals))})")
    
    #######################################################################################
    # AP 3 Graphs
    prof = np.load(join(profile_dir, cli[2] + "-" + str(i) + ".npz"))['prof']
    print(f"Profile Loaded: {cli}-{0}.npz")
    xy_prof = profile_polar2cartesian(prof[0], theta_vals , d_vals, x_vals, y_vals, cli_pos_204[:, :2])
    
    xy_gt = labels[i, :] 
    aoa_gt, tof_gt = gnd_aoa[i, 2, :]*180/np.pi, np.linalg.norm(labels[i] - cli_pos_204[0])
    
    ax13.imshow(np.abs(prof[0]), origin='lower', zorder = -2, extent=(d_vals[0], d_vals[-1], theta_vals[0]*180/np.pi, theta_vals[-1]*180/np.pi))
    ax13.set_aspect((d_vals[-1]-d_vals[0])/((theta_vals[-1]-theta_vals[0])*180/np.pi))
    ax13.plot(tof_gt, aoa_gt, color = 'r', marker='+')
    ax13.title.set_text(f"AoA-ToF Plot of: AP 204")
    ax13.set_xlabel("ToF")
    ax13.set_ylabel('AoA')
    
    
    xy_w_offset = profile_polar2cartesian(prof[0], theta_vals , d_vals, x_vals, y_vals, cli_pos_204[:, :2])
    
    ax23.imshow(xy_w_offset.T, cmap='hot', interpolation='none',origin='lower', zorder = -2, extent=[x_vals[0], x_vals[-1], y_vals[0], y_vals[-1]])
    ax23.plot(xy_gt[0], xy_gt[1],  color = 'w', marker='+')
    ax23.title.set_text("XY Heatmap with Regular Label")
    ax23.set_xlabel(f"x_vals({int(np.min(x_vals))}, {int(np.max(x_vals))})")
    ax23.set_ylabel(f"y_vals({int(np.min(y_vals))}, {int(np.max(y_vals))})")
    
    ax33.imshow(xy_prof.T, cmap='hot', interpolation='none', origin='lower', zorder = -2, extent=[x_vals[0], x_vals[-1], y_vals[0], y_vals[-1]])
    ax33.plot(xy_gt[0], xy_gt[1],  color = 'w', marker='+')
    ax33.title.set_text("XY Heatmap without Offset")
    ax33.set_xlabel(f"x_vals({int(np.min(x_vals))}, {int(np.max(x_vals))})")
    ax33.set_ylabel(f"y_vals({int(np.min(y_vals))}, {int(np.max(y_vals))})")
    
        
    plt.show()

    
animate(1)
#%%
# anim = ani.FuncAnimation(fig, animate, frames=range(0, 10000, 100), interval=1000, repeat = False)

#TODO 
# 1. Plot Ground Truth of AoA-ToF plot, 2. Verify all Axes, XY orientation, and Plots, 3. Fix Axes of AoA-ToF Plot