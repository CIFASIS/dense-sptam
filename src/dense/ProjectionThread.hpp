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

#ifndef __PROJECTIONTHREAD_H
#define __PROJECTIONTHREAD_H

#include <thread>
#include <pcl_ros/point_cloud.h>

#include "Camera.hpp"
#include "DispImageQueue.hpp"
#include "PointCloudQueue.hpp"
#include "FrustumCulling.hpp"

#define MY_MISSING_Z					10000.0
#define PIXEL_DISP_INVALID				-10
#define PIXEL_DISP_CORNER				-11

class Dense;

class ProjectionThread
{

public:

	ProjectionThread(Dense *dense);

	inline void WaitUntilFinished()
	{ projectionThread_.join(); }

private:

	struct projection_log {
		unsigned int new_points, match, unmatch, outlier, merged;
		double time_t[2];
	} log_data;

	Dense *dense_;
	std::thread projectionThread_;
	void compute();

	void filterDisp(const DispRawImagePtr disp_raw_img);
	bool isValidDisparity(const float disp);

	PointCloudPtr generateCloud(DispRawImagePtr disp_raw_img, cv::Mat_<int> *match_mat);

	void cameraToWorld(PointCloudPtr cloud, CameraPose::Ptr current_pos);

	Eigen::Vector3d (*fusionHeuristic) (CameraPose::Ptr current_pos, Eigen::Vector3d current_pt,
	CameraPose::Ptr prev_pos, Eigen::Vector3d prev_pt);

	void doStereoscan(PointCloudEntry::Ptr prev_entry, DispImagePtr disp_img,
	FrustumCulling *frustum_left, FrustumCulling *frustum_right,
	CameraPose::Ptr current_pos, double stereoscan_threshold,
	cv::Mat_<int> *match_mat, PointCloudPtr current_cloud);

};

void downsampleCloud(PointCloudPtr cloud, double voxelLeafSize);

#endif /* __PROJECTIONTHREAD_H */
