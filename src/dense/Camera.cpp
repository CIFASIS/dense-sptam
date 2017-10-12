#include <boost/smart_ptr.hpp>
#include <ros/common.h>

#include "Camera.hpp"

Camera::Camera(const sensor_msgs::CameraInfoConstPtr& left_info,
			   const sensor_msgs::CameraInfoConstPtr& right_info,
			   double frustumNearPlaneDist,
			   double frustumFarPlaneDist)
  : frustumNearPlaneDist_(frustumNearPlaneDist)
  , frustumFarPlaneDist_(frustumFarPlaneDist)
{
	/* Set frame id so both camera info msgs share the same one */
	sensor_msgs::CameraInfoPtr left_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*left_info);
	sensor_msgs::CameraInfoPtr right_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*right_info);
	left_info_copy->header.frame_id = "stereo";
	right_info_copy->header.frame_id = "stereo";

	/* Get Stereo Camera Model from Camera Info messages */
	stereoCameraModel_.fromCameraInfo(left_info_copy, right_info_copy);

	const image_geometry::PinholeCameraModel& cameraLeft = stereoCameraModel_.left();
	const image_geometry::PinholeCameraModel& cameraRight = stereoCameraModel_.right();

	/* Get rectified intrinsic Matrixes (same for both cameras because they are rectified) */
	cv::Mat projectionLeft = cv::Mat(cameraLeft.projectionMatrix());
	cv::Matx33d intrinsicLeft = projectionLeft(cv::Rect(0,0,3,3));
	cv::Mat projectionRight = cv::Mat(cameraRight.projectionMatrix());
	cv::Matx33d intrinsicRight = projectionRight(cv::Rect(0,0,3,3));

	if (intrinsicLeft != intrinsicRight) {
		ROS_ERROR_STREAM("DENSE: stereo camera intrinsic matrices differ!");
		abort();
	}

	/* Save rectified intrinsic Matrix */
	intrinsic_ = intrinsicLeft;

	/* Compute Field Of View (Frustum) */
	left_horizontal_FOV = computeFOV(intrinsic_(0, 0), cameraLeft.rawRoi().width);
	left_vertical_FOV = computeFOV(intrinsic_(1, 1), cameraLeft.rawRoi().height);
	right_horizontal_FOV = computeFOV(intrinsic_(0, 0), cameraRight.rawRoi().width);
	right_vertical_FOV = computeFOV(intrinsic_(1, 1), cameraRight.rawRoi().height);
}

Camera::~Camera()
{}

CameraPose Camera::ComputeRightCameraPose(CameraPose& leftCameraPose)
{
	return CameraPose(leftCameraPose.ToWorld(Eigen::Vector3d(stereoCameraModel_.baseline(), 0, 0)),
					  leftCameraPose.get_orientation());
}

CameraPose::CameraPose(Position position, Orientation orientation)
  : position_(position)
  , orientation_(orientation)
{
	orientation_matrix_ = orientation_.toRotationMatrix();
}

CameraPose::CameraPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation)
{
	position_ = Position(position.x, position.y, position.z);
	orientation_ = Orientation(orientation.w, orientation.x, orientation.y, orientation.z);
	orientation_matrix_ = orientation_.toRotationMatrix();
}

CameraPose::~CameraPose()
{}

bool CameraPose::operator ==(CameraPose& rhs)
{
	return (this->get_orientation().w() == rhs.get_orientation().w() &&
			this->get_orientation().x() == rhs.get_orientation().x() &&
			this->get_orientation().y() == rhs.get_orientation().y() &&
			this->get_orientation().z() == rhs.get_orientation().z() &&
			this->get_position()(0) == rhs.get_position()(0) &&
			this->get_position()(1) == rhs.get_position()(1) &&
			this->get_position()(2) == rhs.get_position()(2));
}

CameraPose CameraPose::applyTransform(TransformPtr base_to_camera)
{
	tf2::Vector3 tf_position = base_to_camera->getOrigin();
	tf2::Quaternion tf_orientation = base_to_camera->getRotation();

	CameraPose::Position position(tf_position.x(), tf_position.y(), tf_position.z());
	CameraPose::Orientation orientation(tf_orientation.w(), tf_orientation.x(),
										tf_orientation.y(), tf_orientation.z());

	position = position + position_;
	orientation = orientation.inverse() * orientation_;

	return CameraPose(position, orientation_);
}

int CameraPose::save(const char *filename, const char *mode)
{
	FILE *f = fopen(filename, mode);

	if (!f) {
		ROS_INFO("%m");
		return -1;
	}

	fprintf(f, "%f %f %f %f %f %f %f %f %f %f %f %f\n",
			orientation_matrix_.row(0)(0), orientation_matrix_.row(0)(1), orientation_matrix_.row(0)(2), position_(0),
			orientation_matrix_.row(1)(0), orientation_matrix_.row(1)(1), orientation_matrix_.row(1)(2), position_(1),
			orientation_matrix_.row(2)(0), orientation_matrix_.row(2)(1), orientation_matrix_.row(2)(2), position_(2));
	fclose(f);

	return 0;
}
