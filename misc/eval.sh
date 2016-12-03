#!/bin/bash
GT_PATH=../../datasets/kitti/sequences/04/velo_clouds/depth_global
OUR_PATH=../../datasets/kitti/sequences/04/dense_clouds/depth_global
#METHOD=mae
#METHOD=rmse
METHOD=silmse

for i in $(ls $OUR_PATH); do
	echo "===> $i"
	./devel/lib/dense/depth_maps_comparision \
	$GT_PATH/$i \
	$OUR_PATH/$i \
	$METHOD
done
