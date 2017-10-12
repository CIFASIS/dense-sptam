#include <boost/smart_ptr.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "denseInterface.hpp"
#include "../utils/kitti_dmap.hpp"

namespace utilities = RobotLocalization::RosFilterUtilities;

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
  : transform_listener_(tfBuffer_)
  , last_publish_seq_(0)
{
	/* Parameters */
	nhp.param<bool>("approximate_sync", parameters.use_approx_sync, false);
	nhp.param<std::string>("base_frame", parameters.base_frame, "base_link");
	nhp.param<std::string>("camera_frame", parameters.camera_frame, "camera");
	nhp.param<std::string>("map_frame", parameters.map_frame, "map");
	nhp.param<double>("frustum_near_plane_dist", parameters.frustum_near_plane_dist, 0.1);
	nhp.param<double>("frustum_far_plane_dist", parameters.frustum_far_plane_dist, 1000.0);
	nhp.param<std::string>("disp_calc_method", parameters.disp_calc_method, "opencv");
	nhp.param<double>("voxel_leaf_size", parameters.voxel_leaf_size, 0);
	nhp.param<double>("max_distance", parameters.max_distance, 0);
	nhp.param<double>("stereoscan_threshold", parameters.stereoscan_threshold, 0);
	nhp.param<std::string>("fusion_heuristic", parameters.fusion_heuristic, "");
	nhp.param<int>("local_area_size", parameters.local_area_size, 1);
	nhp.param<double>("pub_area_filter_min", parameters.pub_area_filter_min, 0);

	nhp.param<int>("libelas_ipol_gap", parameters.libelas_ipol_gap, 0);
	nhp.param<bool>("add_corners", parameters.add_corners, false);
	nhp.param<double>("sigma", parameters.sigma, 0);

	nhp.param<double>("refinement_linear_threshold", parameters.refinement_linear_threshold, 0);
	nhp.param<double>("refinement_angular_threshold", parameters.refinement_angular_threshold, 0);

	nhp.param<std::string>("output_dir", parameters.output_dir, "clouds");

	pub_map_ = nhp.advertise<sensor_msgs::PointCloud2>("dense_cloud", 100);
	pub_map_bad_ = nhp.advertise<sensor_msgs::PointCloud2>("dense_cloud_bad", 100);

	/* In/out topics */
	sub_path_ = nhp.subscribe("keyframes", 1, &denseInterface::cb_keyframes_path, this);

	sub_img_l_.subscribe(nhp, "/keyframe/left/image_rect", 1);
	sub_info_l_.subscribe(nhp, "/keyframe/left/camera_info", 1);
	sub_img_r_.subscribe(nhp, "/keyframe/right/image_rect", 1);
	sub_info_r_.subscribe(nhp, "/keyframe/right/camera_info", 1);

	if (parameters.use_approx_sync) {
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
	this->dense_->point_clouds_->save_all(dense_->parameters.output_dir.c_str());
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
		entry->set_update_pos(pose);
		entry->unlock();
	}

	if (pub_map_.getNumSubscribers() > 0) {
		if (dense_->point_clouds_->get_local_area_seq() > last_publish_seq_) {
			last_publish_seq_ = dense_->point_clouds_->get_local_area_seq();

			PointCloudPtr cloud_good(new PointCloud);
			PointCloudPtr cloud_bad(new PointCloud);
			dense_->point_clouds_->get_local_area_cloud(parameters.pub_area_filter_min, cloud_good, cloud_bad);
			downsampleCloud(cloud_good, parameters.voxel_leaf_size);
			downsampleCloud(cloud_bad, parameters.voxel_leaf_size);

			cloud_good->header.frame_id = cloud_bad->header.frame_id = parameters.map_frame;
			if (pub_map_.getNumSubscribers() > 0)
				pub_map_.publish(cloud_good);
			if (pub_map_bad_.getNumSubscribers() > 0)
				pub_map_bad_.publish(cloud_bad);

			ROS_INFO("Published seq = %u, cloud size (good, bad) = (%lu, %lu)",
					 cloud_good->header.seq, cloud_good->size(), cloud_bad->size());
		}
	}
}

void dense::denseInterface::cb_images(const sensor_msgs::ImageConstPtr& img_msg_left,
									  const sensor_msgs::CameraInfoConstPtr& left_info,
									  const sensor_msgs::ImageConstPtr& img_msg_right,
									  const sensor_msgs::CameraInfoConstPtr& right_info)
{
	ROS_DEBUG("Images received.");

	if (!dense_)
		dense_ = new Dense(left_info, right_info, &parameters);

	/* Get the transformation between the base_frame and the camera_frame */
	ros::Time currentTime = img_msg_left->header.stamp;
	CameraPose::TransformPtr base_to_camera(new CameraPose::Transform);

	if (!utilities::lookupTransformSafe(tfBuffer_, parameters.camera_frame, parameters.base_frame,
										currentTime, *base_to_camera)) {
		ROS_INFO("##### WARNING: Keyframe %u omitted, no cameratobase transform! #####",
				 img_msg_left->header.seq);
		return;
	}

	PointCloudEntry::Ptr entry = dense_->point_clouds_->getEntry(img_msg_left->header.seq);
	entry->set_transform(base_to_camera);

	ImagePtr img_msg_left_copy = boost::make_shared<Image>(*img_msg_left);
	ImagePtr img_msg_right_copy = boost::make_shared<Image>(*img_msg_right);

	ImagePairPtr new_img_pair = boost::make_shared<ImagePair>(img_msg_left_copy, img_msg_right_copy);
	if (dense_->raw_image_pairs_->push(new_img_pair) < 0)
		ROS_INFO("##### WARNING: Keyframe %u omitted, too busy! #####", img_msg_left->header.seq);
}
