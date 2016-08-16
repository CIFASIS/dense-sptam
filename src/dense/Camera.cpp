#include "Camera.hpp"

#include <boost/smart_ptr.hpp>
#include <ros/common.h>

Camera::Camera(
    const sensor_msgs::CameraInfoConstPtr& left_info,
    const sensor_msgs::CameraInfoConstPtr& right_info
) {
    /* Set frame id so both camera info msgs share the same one */
    sensor_msgs::CameraInfoPtr left_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*left_info);
    sensor_msgs::CameraInfoPtr right_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*right_info);
    left_info_copy->header.frame_id = "stereo";
    right_info_copy->header.frame_id = "stereo";

    /* Get Stereo Camera Model from Camera Info messages */
    stereoCameraModel_.fromCameraInfo(left_info_copy, right_info_copy);
}

Camera::~Camera()
{}

CameraPose::CameraPose(
    Position position, Orientation orientation
) : position_(position)
  , orientation_(orientation)
{}

CameraPose::~CameraPose()
{}

bool CameraPose::operator ==(CameraPose &rhs)
{
    return (this->get_orientation().x == rhs.get_orientation().x &&
            this->get_orientation().y == rhs.get_orientation().y &&
            this->get_orientation().z == rhs.get_orientation().z &&
            this->get_orientation().w == rhs.get_orientation().w &&
            this->get_position().x == rhs.get_position().x &&
            this->get_position().y == rhs.get_position().y &&
            this->get_position().z == rhs.get_position().z);
}
