"""Partition the data into train and validation sets based on the grid cells."""
# %%
from utils.data_utils import list_and_sort_files
import numpy as np
import math
from utils.plot_utils import plot_location_pred_vs_gt
import matplotlib.pyplot as plt
from utils.data_utils import get_xy_label_from_file, partition_points_into_n_by_n_grid_cells, randomly_choose_numbers
# %%
# get all files
train_data_path = "/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_train_2"
val_data_path = "/media/datadisk_2/loc_data/wifi/DLoc_data_split/dataset_jacobs_July28/features_aoa/ind_test_2"

data_path_all = list_and_sort_files(train_data_path) + list_and_sort_files(val_data_path)
data_path_all_np = np.array(data_path_all)

# %% get all xy labels
xy_labels_all = [get_xy_label_from_file(file_path) for file_path in data_path_all]
xy_labels_all_np = np.array(xy_labels_all)
# %%
# partition points into n by n grid cells
n = 8
grid_cell_masks = partition_points_into_n_by_n_grid_cells(xy_labels_all_np, n)


# %%
val_min_percentage = 0.2
val_max_percentage = 0.23
assert val_min_percentage < val_max_percentage

total_grid_cells_count = n * n
val_num_cells = math.ceil(val_min_percentage * total_grid_cells_count)

max_iteration = 200
counter = 0
while True:
    counter += 1
    val_set_index, train_set_index = randomly_choose_numbers(total_grid_cells_count, val_num_cells)

    # get val and train points
    val_points = np.concatenate([xy_labels_all_np[grid_cell_masks[i]] for i in val_set_index], axis=0)
    train_points = np.concatenate([xy_labels_all_np[grid_cell_masks[i]] for i in train_set_index], axis=0)

    # get val and train files
    val_files = np.concatenate([data_path_all_np[grid_cell_masks[i]] for i in val_set_index])
    train_files = np.concatenate([data_path_all_np[grid_cell_masks[i]] for i in train_set_index])

    # compute the percentage of val points
    val_pct = len(val_points) / len(xy_labels_all_np)

    # check if the percentage is within the range
    if (val_pct > val_min_percentage and val_pct < val_max_percentage) or (counter >= max_iteration):
        print(f"val point percentage is {len(val_points) / len(xy_labels_all_np)}, counter is: {counter}")
        break

# %%
plot_location_pred_vs_gt(val_points, val_points)
# plt.tile("validation sets")

plot_location_pred_vs_gt(train_points, train_points)
# plt.tile("train sets")

# %% save the train and val files to csv
np.savetxt("train_files.csv", train_files, fmt="%s")
np.savetxt("val_files.csv", val_files, fmt="%s")
# %%
