#!/bin/bash

# Input parameters examples:
#
#   Kitti:
#   dir_dense = "~/.ros/kitti04/dmap/"
#   dir_gt = "~/datasets/kitti04GT/dmap/"
#   sequence = "kitti04_"
#
#   Tsukuba:
#   dir_dense = "~/.ros/tsukuba/dmap/"
#   dir_gt = "~/datasets/tsukuba_dataset/depth_maps/left/dmap/"
#   sequence = "tsukuba"

dir_dense="$1"
dir_gt="$2"
sequence="$3"

echo "Erasing previous computations..."
rm depth_info/*.npy

echo "Computing statistics..."
python depth_map.py $dir_dense $dir_gt --show_time 1 --min_dist 5.0

echo "Plotting..."
python plot_dmap_error.py $sequence

echo "End"
