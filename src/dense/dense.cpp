#include "dense.hpp"

Dense::Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info)
{
    camera_ = new Camera(left_info, right_info);

    raw_left_images = new ImageQueue();
    raw_right_images = new ImageQueue();
    disp_images = new DispImageQueue();
    disparityCalcThread_ = new DisparityCalcThread(raw_left_images, raw_right_images, disp_images);
    projectionThread_ = new ProjectionThread(disp_images);
}

Dense::~Dense()
{}
