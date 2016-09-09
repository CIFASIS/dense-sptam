#include <boost/smart_ptr.hpp>

#include "denseInterface.hpp"

namespace std
{
    /* TODO: part of the C++14 standard */
    template<typename T, typename ...Args>
    std::unique_ptr<T> make_unique(Args&& ...args)
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}

dense::denseInterface::denseInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp)
{
    /* Parameters */
    bool use_approx_sync;
    nhp.param<bool>("approximate_sync", use_approx_sync, false);
    nhp.param<std::string>("odom_frame", odom_frame_, "odom");
    nhp.param<std::string>("base_link_frame", base_frame_, "base_link");
    nhp.param<std::string>("camera_frame", camera_frame_, "camera");
    nhp.param<std::string>("map_frame", map_frame_, "map");
    nhp.param<double>("FrustumNearPlaneDist", frustumNearPlaneDist_, 0.1);
    nhp.param<double>("FrustumFarPlaneDist", frustumFarPlaneDist_, 1000.0);
    nhp.param<std::string>("disp_calc_method", disp_calc_method_, "opencv");

    /* In/out topics */
    sub_path_ = nhp.subscribe("keyframes", 1, &denseInterface::cb_keyframes_path, this);

    sub_img_l_.subscribe(nhp, "/keyframe/left/image_rect", 1);
    sub_info_l_.subscribe(nhp, "/keyframe/left/camera_info", 1);
    sub_img_r_.subscribe(nhp, "/keyframe/right/image_rect", 1);
    sub_info_r_.subscribe(nhp, "/keyframe/right/camera_info", 1);

    pub_map_ = nhp.advertise<sensor_msgs::PointCloud2>("dense_cloud", 100);

    if (use_approx_sync) {
        approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(10),
                                                    sub_img_l_, sub_info_l_, sub_img_r_, sub_info_r_));
        approximate_sync_->registerCallback(boost::bind(&denseInterface::cb_images, this, _1, _2, _3, _4));
    } else {
        exact_sync_.reset(new ExactSync(ExactPolicy(1), sub_img_l_, sub_info_l_, sub_img_r_, sub_info_r_));
        exact_sync_->registerCallback(boost::bind(&denseInterface::cb_images, this, _1, _2, _3, _4));
    }

    ROS_INFO("DENSE node initialized.");
}

dense::denseInterface::~denseInterface()
{
    std::cout << "Starting DENSE node cleanup..." << std::endl;

    std::cout << "Done!" << std::endl;
    ros::Duration(1.0).sleep();
}

void dense::denseInterface::cb_keyframes_path(const nav_msgs::PathConstPtr& path)
{
    ROS_DEBUG("Received path size = %lu", path->poses.size());

    if (!dense_)
        return;

    for (auto& it: path->poses) {
        CameraPose::Ptr pose(new CameraPose(it.pose.position, it.pose.orientation));
        PointCloudEntry::Ptr entry = dense_->point_clouds_->getEntry(it.header.seq);

        entry->lock();
        if (!entry->get_current_pos() || !(*entry->get_current_pos() == *pose)) {
            entry->set_update_pos(pose);
            dense_->point_clouds_->schedule(entry);
        }
        entry->unlock();
    }

    if (pub_map_.getNumSubscribers() > 0) {
        PointCloudEntry::Ptr entry = dense_->point_clouds_->get_last_init();
        if (entry) {
            entry->get_cloud()->header.frame_id = map_frame_;
            pub_map_.publish(entry->get_cloud());
            ROS_DEBUG("Published seq = %u, size = %lu", entry->get_seq(), entry->get_cloud()->size());
        }
    }
}

void dense::denseInterface::cb_images(
    const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
    const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info
) {
    ROS_DEBUG("Images received.");

    if (!dense_)
        dense_ = new Dense(left_info, right_info, frustumNearPlaneDist_, frustumFarPlaneDist_, disp_calc_method_);

    ImagePtr img_msg_left_copy = boost::make_shared<Image>(*img_msg_left);
    ImagePtr img_msg_right_copy = boost::make_shared<Image>(*img_msg_right);

    ImagePairPtr new_img_pair = boost::make_shared<ImagePair>(img_msg_left_copy, img_msg_right_copy);
    if (dense_->raw_image_pairs_->push(new_img_pair) < 0)
        ROS_INFO("##### WARNING: Keyframe omitted! #####");
}
