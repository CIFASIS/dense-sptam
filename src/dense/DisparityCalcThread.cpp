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

#include "DisparityCalcThread.hpp"

#include <boost/smart_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/common.h>

#include "../libelas/src/elas.h"
#include "../libelas/src/image.h"
#include "dense.hpp"
#include "../utils/Time.hpp"

#include "../utils/kitti_dmap.hpp"

DisparityCalcThread::DisparityCalcThread(Dense *dense)
  : dense_(dense)
{
	ROS_INFO("Selected disparity method: %s", dense_->parameters.disp_calc_method.c_str());

	if (dense_->parameters.disp_calc_method == DISP_METHOD_OPENCV)
		disparityCalcThread_ = new std::thread(&DisparityCalcThread::computeCV, this);
	else if (dense_->parameters.disp_calc_method == DISP_METHOD_LIBELAS)
		disparityCalcThread_ = new std::thread(&DisparityCalcThread::computeELAS, this);
	else
		ROS_ERROR("ERROR: Disparity method NOT FOUND: %s", dense_->parameters.disp_calc_method.c_str());
}

DisparityCalcThread::~DisparityCalcThread()
{
	delete disparityCalcThread_;
}

void DisparityCalcThread::computeCV()
{
	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
	ImagePairPtr raw_image_pair;
	ImagePtr raw_left_image, raw_right_image;
	cv::Mat image_left, image_right, dmat;
	DispRawImagePtr disp_raw_img;
	DispImagePtr disp_img;

	while(1) {
		/* Calls to pop() are blocking */
		raw_image_pair = dense_->raw_image_pairs_->pop();

		double start_t, end_t;
		start_t = GetSeg();

		raw_left_image = raw_image_pair->first;
		raw_right_image = raw_image_pair->second;

		image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::MONO8)->image;
		image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::MONO8)->image;

		stereo->compute(image_left, image_right, dmat);

		disp_img = std::make_shared<DispImage>(dmat);
		disp_raw_img = std::make_shared<DispRawImage>(raw_left_image, disp_img);

		if (dense_->disp_images_->push(disp_raw_img) < 0)
			ROS_INFO("##### WARNING: Keyframe %u omitted, projectionThread busy! #####", raw_left_image->header.seq);

		end_t = GetSeg();

		dense_->WriteToLog("disparity,%u,%f\n", raw_left_image->header.seq, end_t - start_t);
		ROS_INFO("Disparity  seq = %u (%f secs)", raw_left_image->header.seq, end_t - start_t);

#if 0 /* Save disparity images */
		char filename[256];
		sprintf(filename, "images/orig_%u.jpg", raw_left_image->header.seq);
		cv::imwrite(filename, image_left);
#endif
	}
}

void DisparityCalcThread::computeELAS()
{
	ImagePairPtr raw_image_pair;
	ImagePtr raw_left_image, raw_right_image;
	cv::Mat image_left, image_right;
	DispRawImagePtr disp_raw_img;
	DispImagePtr disp_img;
	int32_t dims[3];

	float *D1_data = (float*)malloc(dense_->left_info_->width * dense_->left_info_->height * sizeof(float));
	float *D2_data = (float*)malloc(dense_->right_info_->width * dense_->right_info_->height * sizeof(float));
	cv::Mat_<float> *dmat = new cv::Mat_<float>(dense_->left_info_->height, dense_->left_info_->width, D1_data);

	Elas::parameters param(Elas::ROBOTICS);

	param.add_corners = dense_->parameters.add_corners;
	if (dense_->parameters.libelas_ipol_gap)
		param.ipol_gap_width = dense_->parameters.libelas_ipol_gap;

	Elas *elas = new Elas(param);
	dims[2] = dims[0] = dense_->left_info_->width;
	dims[1] = dense_->left_info_->height;

	while(1) {
		/* Calls to pop() are blocking */
		raw_image_pair = dense_->raw_image_pairs_->pop();

		double start_t, end_t;
		start_t = GetSeg();

		raw_left_image = raw_image_pair->first;
		raw_right_image = raw_image_pair->second;

		image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::MONO8)->image;
		image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::MONO8)->image;

		elas->process(image_left.data, image_right.data, D1_data, D2_data, dims);

		disp_img = std::make_shared<DispImage>(dmat);
		disp_raw_img = std::make_shared<DispRawImage>(raw_left_image, disp_img);
		if (dense_->disp_images_->push(disp_raw_img) < 0)
			ROS_INFO("##### WARNING: Keyframe %u omitted, projectionThread busy! #####", raw_left_image->header.seq);

		end_t = GetSeg();

		dense_->WriteToLog("disparity,%u,%f\n", raw_left_image->header.seq, end_t - start_t);
		ROS_INFO("Disparity  seq = %u (%f secs)", raw_left_image->header.seq, end_t - start_t);

#if 0 /* Save disparity image and depth map */
		char filename[256];
		sprintf(filename, "images/%06d.dmap", raw_left_image->header.seq);
		// Alloc memory for depth_map
		float *depth_data = (float*)malloc(dense_->left_info_->width * dense_->left_info_->height * sizeof(float));

		// convert disparity image to depth map
		for ( size_t i = 0; i < (dense_->left_info_->height * dense_->left_info_->width); ++i) {
			float disp = D1_data[i];
			// check if disparity is invalid
			if (!finite(disp) || disp <= 0) {
				depth_data[i] = PIXEL_DEPTH_INVALID;
			} else {
			// compute depth
			float depth = dense_->camera_->getStereoModel().getZ( disp );
			// check if depth is valid
			// if (depth <= dense_->max_distance_) {
					depth_data[i] = depth;
			// }
			// else {
			//		depth_data[i] = PIXEL_DEPTH_INVALID;
			// }
			}
		}

		saveDepthImage(depth_data, dense_->left_info_->height, dense_->left_info_->width, filename);
#endif
	}
}
