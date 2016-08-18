#include "dense.hpp"

Dense::Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info)
{
    camera_ = new Camera(left_info, right_info);

    raw_left_images = new ImageQueue();
    raw_right_images = new ImageQueue();
    disp_images = new DispImageQueue();
    point_clouds = new PointCloudQueue();

    disparityCalcThread_ = new DisparityCalcThread(raw_left_images, raw_right_images, disp_images);
    projectionThread_ = new ProjectionThread(disp_images, point_clouds, camera_);
    transformThread_ = new TransformThread(point_clouds, camera_);
    refinementThread_ = new RefinementThread(point_clouds);
}

Dense::~Dense()
{}
