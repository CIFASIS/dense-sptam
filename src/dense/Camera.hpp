#ifndef CAMERA_H
#define CAMERA_H

#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

class Camera
{
public:

    Camera(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info);
    ~Camera();

    inline image_geometry::StereoCameraModel *getStereoModel()
    { return &stereoCameraModel_; }

private:

    image_geometry::StereoCameraModel stereoCameraModel_;

};

#endif /* CAMERA_H */
