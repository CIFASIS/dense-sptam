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
  usage: ./devel/lib/dense/kitti_ground_truth [in-calib] [in-poses] [in-velo] [out-cloud]

$ ./devel/lib/dense/kitti_ground_truth \
    ../../datasets/kitti/sequences/${sequence}/velodyne/calib.txt \
    ../../datasets/kitti/sequences/${sequence}/velodyne/poses.txt \
    ../../datasets/kitti/sequences/${sequence}/velodyne/raw/ \
    ../../datasets/kitti/sequences/${sequence}/velodyne/pcd/
```

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
