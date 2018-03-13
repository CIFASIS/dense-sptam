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

#ifndef DENSEINTERFACE_H
#define DENSEINTERFACE_H

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <tf2_ros/transform_listener.h>
#include <robot_localization/ros_filter.h>

#include "../dense/dense.hpp"

namespace dense
{
	class denseInterface
	{
	public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		denseInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp);
		~denseInterface();

	private:
		void cb_keyframes_path(const nav_msgs::PathConstPtr& path);
		void cb_images(const sensor_msgs::ImageConstPtr& img_msg_left,
					   const sensor_msgs::CameraInfoConstPtr& left_info,
					   const sensor_msgs::ImageConstPtr& img_msg_right,
					   const sensor_msgs::CameraInfoConstPtr& right_info);

		struct dense_parameters parameters;

		/* In/out topics */
		ros::Subscriber sub_path_;
		message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
		message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_l_, sub_info_r_;
		ros::Publisher pub_map_;

		/*
		 * From S-PTAM: Syncronizer for image messages. We don't know a priory which one
		 * will be used, so we have to define both, no common interface :/
		 */

		/* Exact time image topic synchronizer */
		typedef message_filters::sync_policies::ExactTime
			<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
		typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
		std::shared_ptr<ExactSync> exact_sync_;

		/* Approximate time image topic synchronizer */
		typedef message_filters::sync_policies::ApproximateTime
			<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
		typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
		std::shared_ptr<ApproximateSync> approximate_sync_;

		/* Transform */
		tf2_ros::Buffer tfBuffer_;
		tf2_ros::TransformListener transform_listener_;

		/* DENSE */
		Dense *dense_;

		uint32_t last_publish_seq_;
	};
}

#endif /* DENSEINTERFACE_H */
