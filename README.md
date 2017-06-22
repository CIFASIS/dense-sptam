# Dense ROS package

## Quick start

Build the sptam and dense package running:

```
$ catkin_make --pkg sptam --cmake-args -DSHOW_TRACKED_FRAMES=OFF -DSHOW_PROFILING=OFF -DCMAKE_BUILD_TYPE=Release
$ catkin_make --pkg dense
```

## Then just launch it:

```
$ source devel/setup.bash
$ roslaunch src/dense/launch/kitti.launch
$ rosbag play --clock path/to/kitti_dataset/kitti_00.bag
```

# Dense node

## Parameters

TODO

## Published topics

### `/dense/dense_cloud`

Dense reconstruction point cloud. It only publishes the last `local_area_size`
frames. Note that publishing the entire global point cloud would hog your
system.

### `/dense/dense_cloud_bad`

Dense reconstruction point cloud containing those points with a `viewing_counter`
below the `pub_area_filter_min` parameter.

## Output

* `~/.ros/clouds/`

Dense clouds in `.pcd` format and their poses in `.txt` format are stored here.

* `~/.ros/dense_node.log`

Dense node execution logging information is stored here.

# Tools

## kitti\_ground\_truth: Generate pointclouds from KITTI dataset velodyne binaries

```
$ ./devel/lib/dense/kitti_ground_truth
  usage: ./devel/lib/dense/kitti_ground_truth [in-calib] [in-poses] [in-velo] [out-cloud] [min-distance]

$ ./devel/lib/dense/kitti_ground_truth \
    ../../datasets/kitti/sequences/${sequence}/velodyne/calib.txt \
    ../../datasets/kitti/sequences/${sequence}/velodyne/poses.txt \
    ../../datasets/kitti/sequences/${sequence}/velodyne/raw/ \
    ../../datasets/kitti/sequences/${sequence}/velodyne/pcd/ \
    0.0
```

Note there's a float parameter named `min_distance` at the end, which allows
to filter (omit) those points that are beyond this distance threshold.
This is useful as the velodyne laser may contain noise in the first meters.

## kitti\_dmap\_generator: Generate depth maps (`.dmap`) from DENSE node output for KITTI dataset


```
$ ./devel/lib/dense/kitti_dmap_generator
  usage: ./devel/lib/dense/kitti_ground_truth [in-cam-calib] [in-dense-configuration] [in-poses] [out-dmap]

$ ./devel/lib/dense/kitti_dmap_generator \
    dense/configurationFiles/kitti_cam.yaml \
    dense/configurationFiles/configuration.yaml \
    path/to/poses.txt \
    ../../datasets/kitti/sequences/${sequence}/dmap/
```

## plot\_depth\_map: Plot depth maps

Show coloured depth map:

```
$ python dense/scripts/plot_depth_map.py ${original_depth_map}.dmap
```

Show coloured absolute differences between both maps. Pixels not containing
valid values on both maps are omitted and assigned a negative value.

```
$ python dense/scripts/plot_depth_map.py ${depth_map}.dmap --compare ${another_depth_map}.dmap
```

Optional arguments `--clim_low` and `--clim_high` to specify the plotter clim bounds.

```
$ python dense/scripts/plot_depth_map.py ${depth_map}.dmap --clim_low -3 --clim_high 50
```

## Process and compare depth maps directories

This script accounts for all computation and plotting at the same time:

```
$ python dense/scripts/compute_and_plot.py
```

You should edit it with your paths and sequence to make it work. This script basically calls the following ones..

The following scripts are used to process two sets of depth maps, generated
from DENSE node output and ground truth. Arguments are the paths to directories
containing the `.dmap` files:

```
$ python dense/scripts/depth_map.py path/to/dense/dmaps/ path/to/velodyne/dmaps/
```

(Check other parameters)

Generated output consists in several npy files, used by the following script to plot the results:

```
$ python dense/scripts/plot_dmap_error.py ${sequence_name}
```
## Process and compare two depth maps

```
$ ./devel/lib/dense/depth_maps_comparison ${ground_truth_depth_map}.dmap ${dense_depth_map}.dmap mae
```

```
$ ./devel/lib/dense/depth_maps_comparison ${ground_truth_depth_map}.dmap ${dense_depth_map}.dmap rmse
```

```
$ ./devel/lib/dense/depth_maps_comparison ${ground_truth_depth_map}.dmap ${dense_depth_map}.dmap silmse
```

```
$ ./devel/lib/dense/depth_maps_comparison ${ground_truth_depth_map}.dmap ${dense_depth_map}.dmap intersect
```

```
$ ./devel/lib/dense/depth_maps_comparison ${ground_truth_depth_map}.dmap ${dense_depth_map}.dmap error_color
```

```
$ ./devel/lib/dense/depth_maps_comparison ${ground_truth_depth_map}.dmap ${dense_depth_map}.dmap error_graph size
```

# Examples of use : compile, run, and plot of kitti04 and tsukuba

## KITTI 04

```
$ cd ~/catkin_ws
```

- COMPILE DENSE AND SPTAM
```
$ catkin clean
$ catkin build
```

- LAUNCH DENSE
```
$ roscore &
$ roslaunch dense kitti.launch
$ rosbag play --clock ~/ariel/datasets/bags/kitti_04.bag -r0.1
```

- PCD -> DMAP
```
$ ./devel/lib/dense/kitti_dmap_generator ~/catkin_ws/src/dense/configurationFiles/kitti_cam_04_to_12.yaml ~/catkin_ws/src/dense/configurationFiles/kitti.yaml ~/.ros/clouds/poses.txt ~/.ros/clouds/
```

- PLOT
```
$ mkdir ~/.ros/clouds/dmap/
```

- move the .dmap from ~/ros/clouds/ to ~/.ros/clouds/dmap
- edit scripts/compute_and_plot.py with the new directory

```
$ dir_dense = "~/.ros/clouds/dmap/"
$ dir_gt = "~/ariel/datasets/kitti04GT/dmap/"
$ sequence = "kitti"
```

```
$ cd scripts
$ python compute_and_plot.py
```
This will generate 5 png files in the scripts directory with the names kitti{1-5}.png


## TSUKUBA

```
cd ~/catkin_ws
```

- COMPILE DENSE AND SPTAM
```
catkin clean
catkin build
```

- LAUNCH DENSE
```
roscore &
roslaunch dense tsukuba.launch
rosbag play --clock ~/ariel/datasets/bags/tsukuba_daylight.bag -r0.05
```

- PCD -> DMAP
```
./devel/lib/dense/kitti_dmap_generator  ~/Downloads/tsukuba_cam.yaml ~/catkin_ws/src/dense/configurationFiles/tsukuba.yaml  ~/.ros/clouds/poses.txt ~/.ros/clouds/
```

- PLOT
```
cd ~/catkin_ws/src/dense/scripts
```

- Edit scripts/compute_and_plot.py with the new directory and the gt directory

```
dir_dense = "~/.ros/clouds/dmap/"
dir_gt = "~/ariel/datasets/tsukuba_dataset/depth_maps/left/dmap/"
sequence = "tsukuba"
```

```
cd scripts
python compute_and_plot.py
```
This will generate 5 png files in the scripts directory with the names tsukuba{1-5}.png
