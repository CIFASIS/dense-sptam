import os

# change these with your paths and sequence
dir_dense = "~/.ros/kitti04/dmap/"
dir_gt = "~/ariel/datasets/kitti04GT/dmap/"
sequence = "kitti04_"

#dir_dense = "~/.ros/tsukuba/dmap/"
#dir_gt = "~/ariel/datasets/tsukuba_dataset/depth_maps/left/dmap/"
#sequence = "tsukuba"

print "Erasing previous computations..."
print ""
os.system("rm depth_info/*.npy")

print "Computing statistics..."
print ""
os.system("python depth_map.py " + dir_dense + ' ' + dir_gt + " --show_time 1 --min_dist 5.0")

print ""
print "Plotting..."
print ""
os.system("python plot_dmap_error.py " + sequence)

print ""
print "End"
