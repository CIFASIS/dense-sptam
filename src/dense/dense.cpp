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

#include "boost/filesystem.hpp"
#include "dense.hpp"

#define LOG_FILENAME				"dense_node.log"

namespace fs = boost::filesystem;

Dense::Dense(const sensor_msgs::CameraInfoConstPtr& left_info,
			 const sensor_msgs::CameraInfoConstPtr& right_info,
			 struct dense_parameters *p)
  : left_info_(left_info)
  , right_info_(right_info)
{
	parameters = *p;

	fs::path output_path{parameters.output_dir};
	output_path = fs::absolute(output_path);

	if (!fs::is_directory(output_path)) {
		ROS_ERROR_STREAM("DENSE: output dir " << output_path << " not found!");
		abort();
	}

	ROS_INFO_STREAM("DENSE: output dir " << output_path);


	if (parameters.output_dir.length())
		sprintf(log_filename_, "%s/%s", parameters.output_dir.c_str(), LOG_FILENAME);
	else
		sprintf(log_filename_, "%s", LOG_FILENAME);

	log_file_ = fopen(log_filename_, "w");
	if (!log_file_) {
		ROS_ERROR_STREAM("DENSE: failed to open log file " << log_filename_);
		abort();
	}

	camera_ = new Camera(left_info_, right_info_, parameters.frustum_near_plane_dist,
						 parameters.frustum_far_plane_dist);
	raw_image_pairs_ = new ImageQueue();
	disp_images_ = new DispImageQueue();
	point_clouds_ = new PointCloudQueue(parameters.local_area_size);

	disparityCalcThread = new DisparityCalcThread(this);
	projectionThread_ = new ProjectionThread(this);
	refinementThread_ = new RefinementThread(this);
}

Dense::~Dense()
{
	fclose(log_file_);
}

void Dense::WriteToLog(const char* fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(log_file_, fmt, args);
	va_end(args);
}
