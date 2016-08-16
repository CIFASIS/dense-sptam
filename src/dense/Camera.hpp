#ifndef CAMERA_H
#define CAMERA_H

#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>

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

class CameraPose
{
public:

    typedef boost::shared_ptr<CameraPose> Ptr;
    typedef geometry_msgs::Point Position;
    typedef geometry_msgs::Quaternion Orientation;

    inline Position get_position()
    { return position_; }
    inline Orientation get_orientation()
    { return orientation_; }

    bool operator ==(CameraPose& rhs);

    CameraPose(Position position, Orientation orientation);
    ~CameraPose();

private:

    Position position_;
    Orientation orientation_;

};

#endif /* CAMERA_H */
