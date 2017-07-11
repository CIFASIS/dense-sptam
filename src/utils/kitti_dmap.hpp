#ifndef KITTI_GROUND_TRUTH_H
#define KITTI_GROUND_TRUTH_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include "../dense/PointCloudQueue.hpp"
#include "../dense/dense.hpp"

int generate_depth_maps_kitti_global(const char *in_poses_path, const char *in_clouds_path,
                                     const char *out_path, int region_size, double pub_area_filter_min,
                                     Dense *dense_);
int generate_depth_maps_euroc_global(const char *in_poses_path, const char *in_timestamps_path,
                                     const char *in_clouds_path, const char *out_path,
                                     double pub_area_filter_min, Dense *dense_);
int generate_depth_maps_kitti_local(const char *in_poses_path, const char *in_clouds_path,
                                    const char *out_path, double pub_area_filter_min, Dense *dense_);

void saveDepthImage(float *depth_map_data, int img_height, int img_width, const char *filename);

#endif
