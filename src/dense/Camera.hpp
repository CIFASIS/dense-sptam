#ifndef CAMERA_H
#define CAMERA_H

#include <boost/smart_ptr.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

class Camera
{
public:

    Camera(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info);
    ~Camera();

private:

    image_geometry::StereoCameraModel stereoCameraModel_;

};

#endif /* CAMERA_H */
