#ifndef DENSEINTERFACE_H
#define DENSEINTERFACE_H

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>

#include "../dense/Camera.hpp"
#include "../dense/DisparityCalcThread.hpp"

namespace dense
{

    class denseInterface
    {
    public:
        denseInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp);
        ~denseInterface();

    private:
        void cb_keyframes_path(const nav_msgs::PathConstPtr& path);
        void cb_images(const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
                       const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info);

        /* In/out topics */
        ros::Subscriber sub_path_;
        message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_l_, sub_info_r_;

        /*
         * From S-PTAM: Syncronizer for image messages. We don't know a priory which one
         * will be used, so we have to define both, no common interface :/
         */

        /* Exact time image topic synchronizer */
        typedef message_filters::sync_policies::ExactTime
            <sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
        boost::shared_ptr<ExactSync> exact_sync_;

        /* Approximate time image topic synchronizer */
        typedef message_filters::sync_policies::ApproximateTime
            <sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
        typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
        boost::shared_ptr<ApproximateSync> approximate_sync_;

        Camera *camera_;

        DisparityCalcThread *disparityCalcThread_;
    };

}
#endif /* DENSEINTERFACE_H */
