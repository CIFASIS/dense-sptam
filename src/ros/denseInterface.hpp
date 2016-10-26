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
#include <robot_localization/ros_filter.h>


#include "../dense/dense.hpp"

namespace dense
{

    class denseInterface
    {
    public:
        denseInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp);
        ~denseInterface();

    private:

        void mySigusr1Handler(int sig);

        void cb_save_cloud(const std_msgs::Empty& dummy);
        void cb_keyframes_path(const nav_msgs::PathConstPtr& path);
        void cb_images(const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
                       const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info);

        /* Parameters */
        std::string odom_frame_, base_frame_, camera_frame_, map_frame_;
        double frustumNearPlaneDist_, frustumFarPlaneDist_, voxelLeafSize_;
        double filter_meanK_, filter_stddev_, filter_radius_, filter_minneighbours_;
        double min_disparity_, stereoscan_threshold_, sigma_;
        std::string disp_calc_method_;
        int local_area_size_, libelas_ipol_gap_;
        bool add_corners_;
        std::string single_cloud_path_;
        double refinement_linear_threshold_, refinement_angular_threshold_;
        double pub_area_filter_min_;

        /* In/out topics */
        ros::Subscriber sub_path_, sub_save_cloud_;
        message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_l_, sub_info_r_;
        ros::Publisher pub_map_, pub_map_bad_;

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

        /* Transform */
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener transform_listener_;

        /* DENSE */
        Dense *dense_;

        uint32_t last_publish_seq_;
    };

}
#endif /* DENSEINTERFACE_H */
