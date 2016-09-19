#include <boost/smart_ptr.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

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
  : last_publish_seq_(0)
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
    nhp.param<double>("VoxelLeafSize", voxelLeafSize_, 0);
    nhp.param<double>("filter_meanK", filter_meanK_, 0);
    nhp.param<double>("filter_stddev", filter_stddev_, 0);
    nhp.param<double>("filter_radius", filter_radius_, 0);
    nhp.param<double>("filter_minneighbours", filter_minneighbours_, 0);
    nhp.param<double>("min_disparity", min_disparity_, 0);
    nhp.param<double>("stereoscan_threshold", stereoscan_threshold_, 0);
    nhp.param<int>("local_area_size", local_area_size_, 1);

    nhp.param<int>("libelas_ipol_gap", libelas_ipol_gap_, 0);
    nhp.param<bool>("add_corners", add_corners_, false);
    nhp.param<double>("sigma", sigma_, 0);

    nhp.param<double>("refinement_dist_threshold", refinement_dist_threshold_, 0);

    /* Single mode: Load and publish pointcloud, then exit */
    nhp.param<std::string>("single_cloud_path", single_cloud_path_, "");

    pub_map_ = nhp.advertise<sensor_msgs::PointCloud2>("dense_cloud", 100);

    if (single_cloud_path_ != "") {
        PointCloudPtr global_cloud(new PointCloud);
        boost::filesystem::directory_iterator end_itr;
        for (boost::filesystem::directory_iterator itr(single_cloud_path_); itr != end_itr; ++itr) {
            PointCloudPtr cloud(new PointCloud);
            char filename[256];
            sprintf(filename, "%s/%s", single_cloud_path_.c_str(), itr->path().filename().c_str());
            pcl::io::loadPCDFile(filename, *cloud);
            downsampleCloud(cloud, voxelLeafSize_);
            *global_cloud += *cloud;
            ROS_INFO("Read cloud from file %s/%s", single_cloud_path_.c_str(), itr->path().filename().c_str());
        }
        global_cloud->header.frame_id = map_frame_;
        pub_map_.publish(global_cloud);
        ROS_INFO("Published single cloud size = %lu", global_cloud->size());

        return;
    }

    /* In/out topics */
    sub_path_ = nhp.subscribe("keyframes", 1, &denseInterface::cb_keyframes_path, this);
    sub_save_cloud_ = nhp.subscribe("save_cloud", 1, &denseInterface::cb_save_cloud, this);

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

void dense::denseInterface::cb_save_cloud(const std_msgs::Empty& dummy)
{
    dense_->point_clouds_->save_all();
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
        entry->set_update_pos(pose);
        entry->unlock();
    }

    if (pub_map_.getNumSubscribers() > 0) {
        if (dense_->point_clouds_->get_local_area_seq() > last_publish_seq_) {
            last_publish_seq_ = dense_->point_clouds_->get_local_area_seq();

            PointCloudPtr cloud = dense_->point_clouds_->get_local_area_cloud();
            downsampleCloud(cloud, voxelLeafSize_);

            cloud->header.frame_id = map_frame_;
            pub_map_.publish(cloud);
            ROS_INFO("Published seq = %u, size = %lu", cloud->header.seq, cloud->size());
        }
    }
}

void dense::denseInterface::cb_images(
    const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
    const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info
) {
    ROS_DEBUG("Images received.");

    if (!dense_)
        dense_ = new Dense(left_info, right_info, frustumNearPlaneDist_, frustumFarPlaneDist_, voxelLeafSize_,
                           filter_meanK_, filter_stddev_, disp_calc_method_, filter_radius_, filter_minneighbours_,
                           min_disparity_, stereoscan_threshold_, local_area_size_, libelas_ipol_gap_, add_corners_,
                           sigma_, refinement_dist_threshold_);

    ImagePtr img_msg_left_copy = boost::make_shared<Image>(*img_msg_left);
    ImagePtr img_msg_right_copy = boost::make_shared<Image>(*img_msg_right);

    ImagePairPtr new_img_pair = boost::make_shared<ImagePair>(img_msg_left_copy, img_msg_right_copy);
    if (dense_->raw_image_pairs_->push(new_img_pair) < 0)
        ROS_INFO("##### WARNING: Keyframe %u omitted, too busy! #####", img_msg_left->header.seq);
}
