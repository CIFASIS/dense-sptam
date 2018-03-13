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

#ifndef CAMERA_H
#define CAMERA_H

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Path.h>
#include <robot_localization/ros_filter.h>

class CameraPose;

class Camera
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef std::shared_ptr<Camera> Ptr;

	Camera(const sensor_msgs::CameraInfoConstPtr& left_info,
		   const sensor_msgs::CameraInfoConstPtr& right_info,
		   double frustumNearPlaneDist, double frustumFarPlaneDist);
	~Camera();

	inline image_geometry::StereoCameraModel& getStereoModel()
	{ return stereoCameraModel_; }

	inline double getNearPlaneDist()
	{ return frustumNearPlaneDist_; }
	inline double getFarPlaneDist()
	{ return frustumFarPlaneDist_; }
	inline double GetFOV_LH()
	{ return left_horizontal_FOV; }
	inline double GetFOV_LV()
	{ return left_vertical_FOV; }
	inline double GetFOV_RH()
	{ return right_horizontal_FOV; }
	inline double GetFOV_RV ()
	{ return right_vertical_FOV; }

	CameraPose ComputeRightCameraPose(CameraPose &leftCameraPose);

private:

	/* Compute Field Of View angle for a dimention of the camera */
	inline double computeFOV(const double focal_length, const double image_size)
	{ return 2 * atan(image_size / (2 * focal_length)) * 180 / M_PI; }

	image_geometry::StereoCameraModel stereoCameraModel_;
	cv::Matx33d intrinsic_;
	double frustumNearPlaneDist_, frustumFarPlaneDist_;
	double left_horizontal_FOV, left_vertical_FOV, right_horizontal_FOV, right_vertical_FOV;

};

class CameraPose
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<CameraPose> Ptr;
	typedef Eigen::Vector3d Position;
	typedef Eigen::Quaterniond Orientation;
	typedef Eigen::Matrix3d OrientationMatrix;

	typedef tf2::Transform Transform;
	typedef std::shared_ptr<Transform> TransformPtr;

	inline Position get_position()
	{ return position_; }
	inline cv::Point3d get_cvposition()
	{ cv::Point3d cvpos(position_(0), position_(1), position_(2)); return cvpos; }
	inline Orientation get_orientation()
	{ return orientation_; }
	inline OrientationMatrix get_orientation_matrix()
	{ return orientation_matrix_; }

	bool operator ==(CameraPose& rhs);
	CameraPose applyTransform(TransformPtr base_to_camera);

	inline double distance(CameraPose& pose)
	{ return (this->get_position() - pose.get_position()).norm(); }

	inline Eigen::Vector3d ToWorld(const Eigen::Vector3d& x) const
	{ return orientation_matrix_ * x + position_; }
	inline Eigen::Vector3d ToCamera(const Eigen::Vector3d& x) const
	{ return orientation_matrix_.transpose() * (x - position_); }

	int save(const char *filename, const char *mode="w+");

	CameraPose(Position position, Orientation orientation);
	CameraPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation);
	~CameraPose();

private:

	Position position_;
	Orientation orientation_;
	OrientationMatrix orientation_matrix_;

};

#endif /* CAMERA_H */
