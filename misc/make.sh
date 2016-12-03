#!/bin/bash

if [ $# -gt 0 ]; then
	catkin_make --pkg sptam -DSHOW_TRACKED_FRAMES=OFF
fi &&
catkin_make --pkg dense
