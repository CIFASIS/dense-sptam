#ifndef CAMERA_H
#define CAMERA_H

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Path.h>
#include <robot_localization/ros_filter.h>

class CameraPose;

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

    CameraPose ComputeRightCameraPose(CameraPose &leftCameraPose);

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

    typedef tf2::Transform Transform;
    typedef boost::shared_ptr<Transform> TransformPtr;

    inline Position get_position()
    { return position_; }
    inline cv::Point3d get_cvposition()
    { cv::Point3d cvpos(position_(0), position_(1), position_(2)); return cvpos; }
    inline Orientation get_orientation()
    { return orientation_; }
    inline OrientationMatrix get_orientation_matrix()
    { return orientation_matrix_; }

    bool operator ==(CameraPose& rhs);
    CameraPose applyTransform(TransformPtr base_to_camera);

    inline double distance(CameraPose& pose)
    { return (this->get_position() - pose.get_position()).norm(); }

    inline Position ToWorld(const Position& x) const
    { return orientation_matrix_ * x + position_; }
    inline Position ToCamera(const Position& x) const
    { return orientation_matrix_.transpose() * (x - position_); }

    int save(const char *filename);

    CameraPose(Position position, Orientation orientation);
    CameraPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation);
    ~CameraPose();

private:

    Position position_;
    Orientation orientation_;
    OrientationMatrix orientation_matrix_;

};

#endif /* CAMERA_H */
