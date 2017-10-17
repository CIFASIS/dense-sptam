#!/bin/bash

# Input parameters examples:
#
#  Kitti:
#  dir_dense_log="~/.ros/kitti04/dense_node.log"
#  dir_dense_pcd="~/.ros/kitti04/pcd/"
#  dir_dense_dmap="~/.ros/kitti04/dmap/"
#  dir_gt_dmap="~/datasets/kitti04GT/dmap/"
#  sequence="kitti04_"
#
#  Tsukuba:
#  dir_dense_log="~/.ros/tsukuba/dense_node.log"
#  dir_dense_pcd="~/.ros/tsukuba/pcd/"
#  dir_dense_dmap = "~/.ros/tsukuba/dmap/"
#  dir_gt_dmap = "~/datasets/tsukuba_dataset/depth_maps/left/dmap/"
#  sequence = "tsukuba"

dir_dense_log="$1"
dir_dense_pcd="$2"
dir_dense_dmap="$3"
dir_gt_dmap="$4"
sequence="$5"

usage () {
	echo "usage: $0 <dense-log-file> <dense-pcd-dir> <dense-dmap-dir> <gt-dmap-dir> <sequence-name>" >&2
	exit 1
}

if [ $# -lt 5 ]; then
	usage
fi

echo "Erasing previous computations..."
rm -f depth_info/*.npy

echo "Computing statistics..."
python depth_map.py $dir_dense_dmap $dir_gt_dmap --show_time 1 --min_dist 5.0

echo "Plotting..."
python plot_dmap_error.py $sequence

echo "Profiling..."

# Binaries/lib build path is assumed to be there
profiling=$(../../../devel/lib/dense/point_cloud_profiling $dir_dense_pcd | tail -n3)
total=$(echo $profiling | cut -d' ' -f3)
hypothesis=$(echo $profiling | cut -d' ' -f6)
validated=$(echo $profiling | cut -d' ' -f9)

python profiling.py $dir_dense_log $sequence --hypothesis $hypothesis --validated $validated

echo "End"
