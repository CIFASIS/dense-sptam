/*
 * This file is part of DENSE S-PTAM.
 *
 * Copyright (C) 2017 Ariel D'Alessandro
 * For more information see <https://github.com/adalessandro/dense-sptam>
 *
 * DENSE S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DENSE S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DENSE S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors: Ariel D'Alessandro <ariel@vanguardiasur.com.ar>
 *
 * Department of Computer Science
 * Faculty of Exact Sciences, Engineering and Surveying
 * University of Rosario - Argentina
 */

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
									 const char *out_path, int region_size,
									 const sensor_msgs::CameraInfoConstPtr left_info, Camera *camera);
int generate_depth_maps_euroc_global(const char *in_poses_path, const char *in_timestamps_path,
									 const char *in_clouds_path, const char *out_path,
									 const sensor_msgs::CameraInfoConstPtr left_info, Camera *camera);

void saveDepthImage(float *depth_map_data, int img_height, int img_width, const char *filename);

#endif
