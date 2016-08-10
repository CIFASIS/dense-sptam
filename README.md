# Dense ROS package

## Quick start

Your environment has to be set:

    $ export ROS_MASTER_URI=http://$(hostname):11311/
    $ cd /path/to/your/catkin_workspace/
    $ source ./devel/setup.bash

Then, build it!

    $ catkin_make --pkg sptam -DSHOW_TRACKED_FRAMES=ON
    $ catkin_make --pkg dense

## Run it, baby!

    $ roslaunch ./src/dense/launch/kitti.launch
    $ rosbag play --clock kitti_00.bag
