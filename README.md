# dense-sptam ROS package

# Quick start

The `dense-sptam` package provides a Dockerfile you could use to configure
and build the project. Note that the same instructions inside the Dockerfile
could be followed to setup an Ubuntu distro as well.

## Docker

Create a dir for your catkin workspace and clone `dense-sptam` package:

```
$ mkdir -p catkin/src
$ cd catkin/src
$ git clone git@github.com:adalessandro/dense-sptam.git
$ cd dense-sptam
```

Inside the `dense-sptam` package you'll find the `Dockerfile` and a `docker.sh`
script to ease the docker setup. Just build and run the container.
Note: You need to have a proper SSH agent `with SSH_AUTH_SOCK` environment
variable set.

```
$ ./docker.sh build
$ ./docker.sh run
```

The `docker.sh run` command will mount the current `$PWD/../../` dir to
`/usr/src/dense_sptam` as your working dir inside the container.

Now you're inside the docker container, initialize a catkin workspace, clone
the rest of the packages and build!

```
$ catkin init
$ cd src/
$ git clone git@github.com:lrse/ros-utils.git
$ git clone git@github.com:adalessandro/sptam.git
$ cd ../
$ catkin build --cmake-args \
	-DSHOW_TRACKED_FRAMES=OFF -DSHOW_PROFILING=OFF -DCMAKE_BUILD_TYPE=Release -DUSE_LOOPCLOSURE=OFF
```

## Run it!

Once everything is built, launch the nodes using one of the launchfiles
provided, then play a ROS bag:

```
$ source devel/setup.bash
$ roslaunch src/dense-sptam/launch/kitti.launch &
$ rosbag play --clock path/to/your/kitti.bag
```

## Other docker magic

Using `docker.sh run` the container is run with X11 socket shared, so you can
GUI tools like `rviz` from within the container itself.

# dense node

## Configuration parameters

The dense node allows several configuration parameters. These can be set in the launch
file directly, as done in `launch/kitti.launch` for example:

```
<param name="camera_frame" value="left_camera" />
```

Or you could set it in a YAML configuration file, e.g. `configurationFiles/kitti.yaml`:

```
frustum_near_plane_dist: 0.0001
```

and load it from the launch file, e.g. `launch/kitti.launch`:

```
<rosparam command="load" file="$(find sptam)/configurationFiles/kitti.yaml" />
```

* `base_frame`: (string, default: *"base_link"*) Reference frame for the robot.

* `camera_frame`: (string, default: *"camera"*) Reference frame for the left camera,
used to get left camera pose from tf.

* `map_frame`: (string, default: *"map"*) Name for the published map frame.

* `use_approx_sync`: (bool, default: *false*) Whether to use approximate synchronization for
stereo frames. Set to true if the left and right Cameras do not produce identical
synchronized timestamps for a matching pair of frames.

* `output_dir`: (string, default: *"clouds"*) Path where output is going to be stored
Note that the dense node assumes directory exists.

* `frustum_near_plane_dist`: (double, default: *0.1*) Frustum culling near plane distance
from camera center.

* `frustum_far_plane_dist`: (double, default: *1000.0*) Frustum culling far plane distance
from camera center.

* `voxel_leaf_size`: (double, default: *0*) Point cloud are downsampled using this voxel
leaf size before being published.

* `disp_calc_method`: (string, default: *"libelas"*) Method/library to compute disparity
maps. Can be *opencv* or *libelas*.

* `max_distance`: (double, default: *0*) Discard point triangulated beyond this value
(0 means disabled).

* `stereoscan_threshold`: (double, default: *0*) Points with distance below this value are
considered a match during the fusion stage.

* `fusion_heuristic`: (string, default: *"inverseDepthDistances"*) Heuristic used to fusion
matched points. One of: *simpleMean*, *weigthDistances*, *inverseDepthDistances*.

* `local_area_size`: (int, default: *1*) Number of previous keyframe that are considered to
search for matches.

* `libelas_ipol_gap`: (int, default: *0*) Libelas library parameter (interpolate gaps
smaller than this value).

* `add_corners`: (bool, default: *false*) Libelas library parameter (add support points at
image corners with nearest neighbor disparities).

* `refinement_linear_threshold`: (double, default: *0*) Keyframe pose updates with 3d
distance greater than this value are refined in the refinement thread.

* `refinement_angular_threshold`: (double, default: *0*) Keyframe pose updates with angular
distance greater than this value are refined in the refinement thread.

## Published topics

### `/dense/dense_cloud`

Dense reconstruction point cloud. It only publishes the last `local_area_size`
frames. Note that publishing the entire global point cloud would probably hog
your system.

## Output

Dense node output is stored at the directory pointed by parameter `output_dir`:

* `xxxxxx.pcd`: 3d dense reconstruction point clouds in `.pcd` format. This file
is named with its associated keyframe's id.

* `xxxxxx.txt`: pose info for keyframe and pointcloud with same id.

* `dense_node.log`: log info collected during the dense node run.

# Tools

## Generate PCD point clouds from KITTI ground truth velodyne binaries

```
$ ./devel/lib/dense/kitti_ground_truth

usage: ./devel/lib/dense/kitti_ground_truth [in-calib] [in-poses] [in-velo] [out-cloud] [min-distance]

    in-calib: input file from KITTI ground truth containing camera calibration parameters.
    in-poses: input file from KITTI ground truth containing the list of poses.
    in-velo: input folder from KITTI ground truth containing the velodyne binary clouds.
    out-cloud: output folder where PCD point cloud are going to be stored.
    min-distance: omit points that are farther than this distance threshold.

$ ./devel/lib/dense/kitti_ground_truth \
    path/to/kitti/sequence/velodyne/calib.txt \
    path/to/kitti/sequence/velodyne/poses.txt \
    path/to/kitti/sequence/velodyne/raw/ \
    path/to/output/pcd/ \
    0.0

TODO
```

Note there's a float parameter named `min_distance` at the end, which allows
to filter (omit) those points that are beyond this distance threshold.
This is useful as the velodyne laser may contain noisy points in the first meters,
which we may want to avoid.

## Generate depth maps (`.dmap`) from DENSE node output for KITTI dataset

```
$ ./devel/lib/dense/kitti_dmap_generator

usage: ./devel/lib/dense/kitti_dmap_generator [calibration] [configuration] [poses] [region_size] [pcd_path]

    calibration: input file with camera calibration parameters.
    configuration: input file with configuration parameters.
    poses: intput file containing pose info, outputted from dense node run.
    region_size: project this number of closest keyframes (previous and next) for each depth map.
    pcd_path: output folder where depth maps (`.dmap`) are going to be stored.

$ ./devel/lib/dense/kitti_dmap_generator \
    configurationFiles/kitti_cam_04_to_12.yaml \
    configurationFiles/kitti.yaml \
    dense/node/output/poses.txt \
    30 \
    path/to/output/dmap/

TODO
```

## Plot/show depth maps

Show coloured depth map:

```
$ python scripts/plot_depth_map.py -h
usage: plot_depth_map.py [-h] [--compare COMPARE] [--save SAVE]
                         [--clim_low CLIM_LOW] [--clim_high CLIM_HIGH]
                         dmap

positional arguments:
  dmap                  dmap file

optional arguments:
  -h, --help            show this help message and exit
  --compare COMPARE     dmap file to compare
  --save SAVE           save image to output file
  --clim_low CLIM_LOW   plotter clim low bound
  --clim_high CLIM_HIGH
                        plotter clim high bound

NOTE: Invalid (negative) depth values are mapped to -10
```

```
$ python scripts/plot_depth_map.py ${original_depth_map}.dmap
```

Show coloured absolute differences between both maps. Pixels not containing
valid values on both maps are omitted and assigned a negative value.

```
$ python scripts/plot_depth_map.py ${depth_map}.dmap --compare ${another_depth_map}.dmap
```

Optional arguments `--clim_low` and `--clim_high` to specify the plotter clim bounds.

```
$ python scripts/plot_depth_map.py ${depth_map}.dmap --clim_low -3 --clim_high 50
```

## Process and compare depth maps directories

This script accounts for all computation and plotting at the same time:

```
$ python scripts/compute_and_plot.py
```

You should edit it with your paths and sequence to make it work. This script basically calls the following ones..

The following scripts are used to process two sets of depth maps, generated
from DENSE node output and ground truth. Arguments are the paths to directories
containing the `.dmap` files:

```
$ python scripts/depth_map.py path/to/dense/dmaps/ path/to/velodyne/dmaps/
```

(Check other parameters)

Generated output consists in several npy files, used by the following script to plot the results:

```
$ python scripts/plot_dmap_error.py ${sequence_name}
```

## Profiling

Every dense node run will output useful log data to a file. This file is
located at `output_dir/dense_node.log`.

Using the following script, the log data can be processed to get human-readable
information:

```
$ python scripts/profiling.py /path/to/dense_node.log 

    Keyframes processed per phase
        Disparity:           244
        Heuristic/fusion:    244
        Refinement:           213

    Mean time per phase (secs)
        Disparity:           157.356557377
        Heuristic/fusion:    69.393442623
        Refinement:           3.17840375587

    Heuristic results (points)
        Fusions/matches:     20145818
        Outliers:            4853236
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
