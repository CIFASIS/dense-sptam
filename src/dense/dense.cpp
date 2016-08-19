#include "dense.hpp"

Dense::Dense(
    const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info,
    double frustumNearPlaneDist, double frustumFarPlaneDist, std::string disp_calc_method
) {
    Camera::Ptr camera(new Camera(left_info, right_info, frustumNearPlaneDist, frustumFarPlaneDist));

    raw_left_images = new ImageQueue();
    raw_right_images = new ImageQueue();
    disp_images = new DispImageQueue();
    point_clouds = new PointCloudQueue();

    disparityCalcThread_ = new DisparityCalcThread(left_info, right_info, raw_left_images, raw_right_images,
                                                   disp_images, disp_calc_method);
    projectionThread_ = new ProjectionThread(disp_images, point_clouds, camera);
    transformThread_ = new TransformThread(point_clouds, camera);
    refinementThread_ = new RefinementThread(point_clouds);
}

Dense::~Dense()
{}
