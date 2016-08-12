#include "Camera.hpp"

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
