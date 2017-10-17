# dense-sptam ROS package

## Quick start

The `dense-sptam` package provides a Dockerfile you could use to configure
and build the project. Note that the same instructions inside the Dockerfile
could be followed to setup an Ubuntu distro as well.

### Docker

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

### Run it!

Once everything is built, launch the nodes using one of the launchfiles
provided, then play a ROS bag:

```
$ source devel/setup.bash
$ roslaunch src/dense-sptam/launch/kitti.launch &
$ rosbag play --clock path/to/your/kitti.bag
```

### Other docker magic

Using `docker.sh run` the container is run with X11 socket shared, so you can
GUI tools like `rviz` from within the container itself.

## dense node

### Configuration parameters

The dense node allows several configuration parameters. These can be set in the launch
file directly, as done in `launch/kitti.launch` for example:

```
<param name="camera_frame" value="left_camera" />
```

Or you could set it in a YAML configuration file, e.g. `configuration_files/kitti.yaml`:

```
frustum_near_plane_dist: 0.0001
```

and load it from the launch file, e.g. `launch/kitti.launch`:

```
<rosparam command="load" file="$(find dense)/configuration_files/kitti.yaml" />
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

### Published topics

* `/dense/dense_cloud`: Dense reconstruction point cloud. It only publishes the last
`local_area_size` frames. Note that publishing the entire global point cloud would
probably hog your system.

### Output

Dense node output is stored at the directory pointed by parameter `output_dir`:

* `xxxxxx.pcd`: 3d dense reconstruction point clouds in `.pcd` format. This file
is named with its associated keyframe's id.

* `xxxxxx.txt`: pose info for keyframe and pointcloud with same id.

* `dense_node.log`: log info collected during the dense node run.

## Tools

### Generate PCD point clouds from KITTI ground truth velodyne binaries

```
$ ./devel/lib/dense/kitti_ground_truth

usage: ./devel/lib/dense/kitti_ground_truth [in-calib] [in-poses] [in-velo] [out-cloud] [min-distance]

    in-calib: input file from KITTI ground truth containing camera calibration parameters.
    in-poses: input file from KITTI ground truth containing the list of poses.
    in-velo: input folder from KITTI ground truth containing the velodyne binary clouds.
    out-cloud: output folder where PCD point cloud are going to be stored.
    min-distance: omit points that are closer than this distance threshold.

$ ./devel/lib/dense/kitti_ground_truth \
    path/to/kitti/sequence/velodyne/calib.txt \
    path/to/kitti/sequence/velodyne/poses.txt \
    path/to/kitti/sequence/velodyne/raw/ \
    path/to/output/pcd/ \
    0.0

    Processing: path/to/kitti/sequence/velodyne/raw/000000.bin
        saved in: path/to/output/pcd/000000.pcd
    [...]
    Processing: path/to/kitti/sequence/velodyne/raw/004540.bin
        saved in: path/to/output/pcd/004540.pcd
    TOTAL: 4541 clouds
```

Note there's a float parameter named `min_distance` at the end, which allows
to filter (omit) those points that are closer than this distance threshold.
This is useful as the velodyne laser may contain noisy points in the first meters,
which we may want to avoid.

### Generate depth maps (`.dmap`) from DENSE node output for KITTI dataset

```
$ ./devel/lib/dense/kitti_dmap_generator

usage: ./devel/lib/dense/kitti_dmap_generator [calibration] [configuration] [poses] [region_size] [pcd_path]

    calibration: input file with camera calibration parameters.
    configuration: input file with configuration parameters.
    poses: input file containing pose info, outputted from dense node run.
    region_size: project this number of closest keyframes (previous and next) for each depth map.
    pcd_path: input/output folder where pcd files are read from and depth maps are going to be stored.

$ ./devel/lib/dense/kitti_dmap_generator \
    configuration_files/kitti_cam_04_to_12.yaml \
    configuration_files/kitti.yaml \
    dense/node/output/poses.txt \
    30 \
    dense/node/output/pcd

    Poses path: dense/node/output/poses.txt
    pcd directory path: dense/node/output/pcd
    frustumNearPlaneDist: 0.0001
    frustumFarPlaneDist: 50
    voxelLeafSize: 0.1
    disp_calc_method: libelas
    max_distance: 20
    stereoscan_threshold: 0.25
    local_area_size: 10
    libelas_ipol_gap: 1000
    add_corners: 0
    refinement_linear_threshold: 0.01
    refinement_angular_threshold: 0.001
    region_size: 30
    image_width: 1226
    image_height: 370
    camera_matrix: [707.0912, 0, 601.8873;
     0, 707.0912, 183.1104;
     0, 0, 1]
    baseline: 0.537151
    rotation: [1, 0, 0;
     0, 1, 0;
     0, 0, 1]
    projection: [707.0912, 0, 601.8873, 0;
     0, 707.0912, 183.1104, 0;
     0, 0, 1, 0]
    rotation: [1, 0, 0;
     0, 1, 0;
     0, 0, 1]
    projection: [707.0912, 0, 601.8873, -379.8144999943973;
     0, 707.0912, 183.1104, 0;
     0, 0, 1, 0]
    Processing: datasets/kitti/04/velodyne/pcd//000000.pcd
    position: -5.55111e-17
               0
     2.22045e-16
    Processing: datasets/kitti/04/velodyne/pcd//000001.pcd
    position: 0.00128913
    -0.0182162
       1.31064
    [...]
```

### Plot/show depth maps

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

### Profiling

Every dense node run will output useful log data to a file. This file is
located at `output_dir/dense_node.log`.

Using the following script, the log data can be processed to get human-readable
information:

```
$ python src/dense-sptam/scripts/profiling.py --help
usage: profiling.py [-h] [--hypothesis HYPOTHESIS] [--validated VALIDATED]
                    [--show]
                    dense_log sequence_name

positional arguments:
  dense_log             dense node log
  sequence_name         sequence name

optional arguments:
  -h, --help            show this help message and exit
  --hypothesis HYPOTHESIS
                        set hypothesis points number
  --validated VALIDATED
                        set validated points number
  --show                show images instead of saving

$ python scripts/profiling.py /path/to/dense_node.log $my_sequence_name

  Keyframes processed per phase
      Disparity:            243
      Heuristic/fusion:     242
      Refinement:           214

  Mean time per phase (ms)
      Disparity:            159.50617284
      Heuristic/fusion:     75.5082644628
      Refinement:           4.41588785047

  Heuristic results (points)
      Total points created: 29395295
      Fusions/matches:      12049853
      Outliers:             3828178
```

The above script can generate and show/save plots of the collected data. The `--hypothesis`
and `--validated` options allow to set the respective point cloud size values to be shown
in the different plots. See `compute_and_plot.sh` script for more info about this feature.

### Altogether!

This script accounts for all computation and plotting at the same time:

```
$ scripts/compute_and_plot.sh
  usage: scripts/compute_and_plot.sh <dense-log-file> <dense-pcd-dir> <dense-dmap-dir> <gt-dmap-dir> <sequence-name>

```

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

## Examples of use

Let's run and plot results for some benchmark dataset sequences.

### Dataset KITTI - sequence 04

Run dense node and play bag. Note that after bag has finished playing, all ros nodes
are killed, so launchfile ends too.

```
$ roslaunch launch/kitti.launch & rosbag play --clock kitti_04.bag ; rosnode kill -a
```

Generate depth maps from dense node output. Note that this example assumes that output
dir path was the default one (at `~/.ros/clouds/`).

```
$ ./devel/lib/dense/kitti_dmap_generator \
    configuration_files/kitti_cam_04_to_12.yaml \
    configuration_files/kitti.yaml \
    ~/.ros/clouds/poses.txt \
    30 \
    ~/.ros/clouds/
```

Finally, let's process and plot the results, comparing them with the ground truth:

```
$ cd scripts/
$ ./compute_and_plot.sh path/to/dense/dense_node.log path/to/dense/pcd/ path/to/dense/dmaps/ path/to/ground_truth/dmaps/ kitti
```

This will generate 5 png files in the scripts directory with the names kitti{1-5}.png

### Dataset TSUKUBA - sequence daylight

Same comments and descriptions as in previous example.

```
$ roslaunch launch/tsukuba.launch & rosbag play --clock tsukuba_daylight.bag ; rosnode kill -a
```

```
$ ./devel/lib/dense/kitti_dmap_generator \
    configuration_files/tsukuba_cam.yaml \
    configuration_files/tsukuba.yaml \
    ~/.ros/clouds/poses.txt \
    30 \
    ~/.ros/dmaps/
```

Tsukuba dataset provides ground truth depth maps for left and right cameras. Note that we're
always using left camera maps, so choose that as below.

```
$ cd scripts/
$ ./compute_and_plot.sh path/to/dense/dense_node.log path/to/dense/pcd/ path/to/dense/dmaps/ path/to/ground_truth/dmaps/ tsukuba
```

This will generate 5 png files in the scripts directory with the names tsukuba{1-5}.png
