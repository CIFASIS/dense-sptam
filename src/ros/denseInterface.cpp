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

    /* In/out topics */
    sub_path_ = nhp.subscribe("keyframes", 1, &denseInterface::cb_keyframes_path, this);

    sub_img_l_.subscribe(nhp, "/keyframe/left/image_rect", 1);
    sub_info_l_.subscribe(nhp, "/keyframe/left/camera_info", 1);
    sub_img_r_.subscribe(nhp, "/keyframe/right/image_rect", 1);
    sub_info_r_.subscribe(nhp, "/keyframe/right/camera_info", 1);

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
    if (!dense_)
        return;

    ROS_INFO("Path size = %lu, Raw size = %lu, Disp size = %lu, Clouds = %lu", path->poses.size(),
            dense_->raw_left_images->size(), dense_->disp_images->size(), dense_->point_clouds->size());
}

void dense::denseInterface::cb_images(
    const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
    const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info
) {
    ROS_DEBUG("Images received.");

    if (!dense_)
        dense_ = new Dense(left_info, right_info);

    ImagePtr img_msg_left_copy = boost::make_shared<Image>(*img_msg_left);
    ImagePtr img_msg_right_copy = boost::make_shared<Image>(*img_msg_right);

    dense_->raw_left_images->push(img_msg_left_copy);
    dense_->raw_right_images->push(img_msg_right_copy);
}
