#ifndef CAMERA_H
#define CAMERA_H

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Path.h>

class Camera
{
public:

    typedef boost::shared_ptr<Camera> Ptr;

    Camera(
        const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info,
        double frustumNearPlaneDist, double frustumFarPlaneDist
    );
    ~Camera();

    inline image_geometry::StereoCameraModel& getStereoModel()
    { return stereoCameraModel_; }

    inline double getNearPlaneDist()
    { return frustumNearPlaneDist_; }
    inline double getFarPlaneDist()
    { return frustumFarPlaneDist_; }
    inline double GetFOV_LH()
    { return left_horizontal_FOV; }
    inline double GetFOV_LV()
    { return left_vertical_FOV; }
    inline double GetFOV_RH()
    { return right_horizontal_FOV; }
    inline double GetFOV_RV ()
    { return right_vertical_FOV; }

private:

    /* Compute Field Of View angle for a dimention of the camera */
    inline double computeFOV(const double focal_length, const double image_size)
    { return 2 * atan(image_size / (2 * focal_length)) * 180 / M_PI; }

    image_geometry::StereoCameraModel stereoCameraModel_;
    cv::Matx33d intrinsic_;
    double frustumNearPlaneDist_, frustumFarPlaneDist_;
    double left_horizontal_FOV, left_vertical_FOV, right_horizontal_FOV, right_vertical_FOV;

};

class CameraPose
{
public:

    typedef boost::shared_ptr<CameraPose> Ptr;
    typedef Eigen::Vector3d Position;
    typedef Eigen::Quaterniond Orientation;
    typedef Eigen::Matrix3d OrientationMatrix;

    inline Position get_position()
    { return position_; }
    inline Orientation get_orientation()
    { return orientation_; }
    inline OrientationMatrix get_orientation_matrix()
    { return orientation_matrix_; }

    bool operator ==(CameraPose& rhs);

    inline Position ToWorld(const Position& x) const
    { return orientation_matrix_ * x + position_; }

    CameraPose(Position position, Orientation orientation);
    CameraPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation);
    ~CameraPose();

private:

    Position position_;
    Orientation orientation_;
    OrientationMatrix orientation_matrix_;

};

#endif /* CAMERA_H */
