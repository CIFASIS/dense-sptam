#include "dense.hpp"

Dense::Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info)
{
    camera_ = new Camera(left_info, right_info);
    disparityCalcThread_ = new DisparityCalcThread();
}

Dense::~Dense()
{}
