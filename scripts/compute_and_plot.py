import os

# change these with your paths and sequence
dir_dense = "~/.ros/kitti04dmap/"
dir_gt = "~/ariel/datasets/kitti04GT"
sequence = "kitti"

print "Erasing previous computations..."
os.system("rm depth_info/*.npy")

print "Computing statistics..."
os.system("python depth_map.py " + dir_dense + ' ' + dir_gt + " --show_time 1")

print "Plotting..."
os.system("python plot_dmap_error.py " + sequence)
