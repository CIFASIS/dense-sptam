/*
 * This file is part of DENSE S-PTAM.
 *
 * Copyright (C) 2017 Ariel D'Alessandro
 * For more information see <https://github.com/adalessandro/dense-sptam>
 *
 * DENSE S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DENSE S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DENSE S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors: Ariel D'Alessandro <ariel@vanguardiasur.com.ar>
 *
 * Department of Computer Science
 * Faculty of Exact Sciences, Engineering and Surveying
 * University of Rosario - Argentina
 */

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "ProjectionThread.hpp"
#include "dense.hpp"
#include "../utils/Time.hpp"
#include "../utils/eigen_alignment.hpp"

Eigen::Vector3d fuseSimpleMean(CameraPose::Ptr current_pos, Eigen::Vector3d current_pt,
							   CameraPose::Ptr prev_pos, Eigen::Vector3d prev_pt);
Eigen::Vector3d fuseWeigthDistances(CameraPose::Ptr current_pos, Eigen::Vector3d current_pt,
									CameraPose::Ptr prev_pos, Eigen::Vector3d prev_pt);
Eigen::Vector3d fuseInverseDepth(CameraPose::Ptr current_pos, Eigen::Vector3d current_pt,
								 CameraPose::Ptr prev_pos, Eigen::Vector3d prev_pt);

ProjectionThread::ProjectionThread(Dense *dense)
  : dense_(dense)
  , projectionThread_(&ProjectionThread::compute, this)
{
	if (dense->parameters.fusion_heuristic == "simpleMean") {
		this->fusionHeuristic = fuseSimpleMean;
	} else if (dense->parameters.fusion_heuristic == "weigthDistances") {
		this->fusionHeuristic = fuseWeigthDistances;
	} else if (dense->parameters.fusion_heuristic == "inverseDepthDistances") {
		this->fusionHeuristic = fuseInverseDepth;
	} else {
		ROS_ERROR_STREAM("ProjectionThread: bad parameter fusion_heuristic!");
		abort();
	}

	ROS_INFO_STREAM("ProjectionThread: using fusion heuristic " << dense->parameters.fusion_heuristic);
}

void ProjectionThread::compute()
{
	while(1) {
		/* Calls to pop() are blocking */
		DispRawImagePtr disp_raw_img = dense_->disp_images_->pop();
		PointCloudEntry::Ptr entry = dense_->point_clouds_->getEntry(disp_raw_img->first->header.seq);
		log_data = { 0 };

		entry->lock();

		log_data.time_t[0] = GetSeg();

		CameraPose::Ptr update_pose = entry->get_update_pos();
		if (!update_pose) {
			ROS_INFO("##### WARNING: Keyframe %u omitted, no pose! #####", entry->get_seq());
			entry->unlock();
			continue;
		}

		entry->set_current_pos(update_pose);
		entry->set_update_pos(nullptr);

		/*
		 * FIXME: Is it ok to just use the current_pose as the left camera pose?
		 * Or should we be applying the robot to camera transform first? Like this:
		 *   CameraPose::Ptr pose_left =
		 *      boost::make_shared<CameraPose>(
		 *          current_pose->applyTransform(entry->get_transform()));
		 */
		CameraPose::Ptr pose_left = update_pose;

		entry->unlock();

		CameraPose::Ptr pose_right =
      std::make_shared_aligned<CameraPose>(dense_->camera_->ComputeRightCameraPose(*pose_left));

		FrustumCulling frustum_left(pose_left->get_position(), pose_left->get_orientation_matrix(),
									dense_->camera_->GetFOV_LH(), dense_->camera_->GetFOV_LV(),
									dense_->camera_->getNearPlaneDist(), dense_->camera_->getFarPlaneDist());

		FrustumCulling frustum_right(pose_right->get_position(), pose_right->get_orientation_matrix(),
									 dense_->camera_->GetFOV_LH(), dense_->camera_->GetFOV_LV(),
									 dense_->camera_->getNearPlaneDist(), dense_->camera_->getFarPlaneDist());

		/*
		 * Pixels that were matched/fused are marked in this matrix so they
		 * don't triangulate/add a new point to the map.
		 */
		cv::Mat_<int> *match_mat = new cv::Mat_<int>(disp_raw_img->second->size(), 0);

		PointCloudPtr current_cloud(new PointCloud);

		for (auto& local_area_entry : dense_->point_clouds_->local_area_queue_) {
			/* No need to lock the entry as no one else will alter its current_pose or cloud */
			doStereoscan(local_area_entry, disp_raw_img->second, &frustum_left, &frustum_right,
			pose_left, dense_->parameters.stereoscan_threshold, match_mat, current_cloud);
		}

		filterDisp(disp_raw_img);
		PointCloudPtr cloud = generateCloud(disp_raw_img, match_mat);
		cameraToWorld(cloud, pose_left);

		/* Number of new created points */
		log_data.new_points = cloud->size();

		*cloud += *current_cloud;

		log_data.time_t[1] = GetSeg();

		entry->lock();
		entry->set_cloud(cloud);
		dense_->point_clouds_->push_local_area(entry);
		entry->unlock();

		dense_->WriteToLog("projection,%u,%f,%lu,%lu,%lu,%lu\n", entry->get_seq(),
						   log_data.time_t[1] - log_data.time_t[0], log_data.new_points, log_data.match,
						   log_data.unmatch, log_data.outlier);

		ROS_INFO("Projected seq %u (size = %lu, time = %f)", entry->get_seq(), cloud->size(),
				 log_data.time_t[1] - log_data.time_t[0]);
	}
}

/*
 * ProjectionThread::isValidDisparity
 *
 * Returns whether a pixel disparity value is considered valid or not.
 */
bool ProjectionThread::isValidDisparity(const float disp)
{
	if (!finite(disp) || disp <= 0)
		return false;

	/*
	 * Disparity values that correspond to distances greater than the
	 * maximum allowed distance, are considered invalid.
	 */
	 return dense_->camera_->getStereoModel().getZ(disp) <= dense_->parameters.max_distance;
}

/*
 * ProjectionThread::filterDisp
 *
 * Applies ProjectionThread::isValidDisparity() to every pixel in the
 * disparity map of @disp_raw_img, setting a value of PIXEL_DISP_INVALID
 * to those pixels that are invalid. Valid pixels are not modified.
 */
void ProjectionThread::filterDisp(const DispRawImagePtr disp_raw_img)
{
	ImagePtr raw_left_image = disp_raw_img->first;
	DispImagePtr disp_img = disp_raw_img->second;

	for (unsigned int i = 0; i < raw_left_image->height; i++)
		for (unsigned int j = 0; j < raw_left_image->width; j++)
			if (!isValidDisparity(disp_img->at<float>(i, j)))
				disp_img->at<float>(i, j) = PIXEL_DISP_INVALID;
}

/*
 * ProjectionThread::generateCloud
 *
 * Generate a point cloud based on a disparity map.
 * Every pixel with a valid disparity is re-projected to the 3D space.
 */
PointCloudPtr ProjectionThread::generateCloud(DispRawImagePtr disp_raw_img, cv::Mat_<int> *match_mat)
{
	ImagePtr raw_left_image = disp_raw_img->first;
	DispImagePtr disp_img = disp_raw_img->second;
	PointCloudPtr cloud(new PointCloud);

	cv::Mat image_left(cv_bridge::toCvCopy(raw_left_image,
										   sensor_msgs::image_encodings::MONO8)->image);

	for (unsigned int i = 0; i < raw_left_image->height; i++) {
		for (unsigned int j = 0; j < raw_left_image->width; j++) {
			float disp = disp_img->at<float>(i, j);

			/* Pixels that were matched/fused don't add new points to the map */
			if (match_mat->at<int>(i, j) || !isValidDisparity(disp))
				continue;

			cv::Point3d point;
			cv::Point2d pixel(j, i);

			dense_->camera_->getStereoModel().projectDisparityTo3d(pixel, disp, point);

			Point new_pt3d;
			new_pt3d.fromCV(point);

			uint8_t g = image_left.at<uint8_t>(i, j);
			int32_t rgb = (g << 16) | (g << 8) | g;
			memcpy(&new_pt3d.rgb, &rgb, sizeof(int32_t));

			cloud->push_back(new_pt3d);
		}
	}

	return cloud;
}

/*
 * ProjectionThread::cameraToWorld
 *
 * Transform every point in @cloud from camera to world coordinates using
 * @current_pos as the pose of the camera.
 */
void ProjectionThread::cameraToWorld(PointCloudPtr cloud, CameraPose::Ptr current_pos)
{
	for (auto& it: *cloud)
		it.fromEigen(current_pos->ToWorld(it.asEigen()));
}

enum stereoscan_status {
	/* Point projection is out of image plane */
	STATUS_OUT_OF_IMAGE,
	/* Point disparity value is invalid  */
	STATUS_INVALID_DISPARITY,
	/* Point merge criteria was satisfied, there was a match */
	STATUS_MATCH,
	/* Point didn't match, but wasn't discarded (outlier) */
	STATUS_UNMATCH,
	/* Point didn't match and was discarded as outlier */
	STATUS_OUTLIER,
	/* Sentinel - Length marker */
	STATUS_LENGTH,
};

Eigen::Vector3d fuseSimpleMean(CameraPose::Ptr current_pos, Eigen::Vector3d current_pt,
							   CameraPose::Ptr prev_pos, Eigen::Vector3d prev_pt)
{
	return (prev_pt + current_pt) / 2.0;
}


Eigen::Vector3d fuseWeigthDistances(CameraPose::Ptr current_pos, Eigen::Vector3d current_pt,
									CameraPose::Ptr prev_pos, Eigen::Vector3d prev_pt)
{
	double dist_near, dist_far, alpha;
	Eigen::Vector3d pt_near, pt_far;

	Eigen::Vector3d current_pos_eigen = current_pos->get_position();
	Eigen::Vector3d prev_pos_eigen = prev_pos->get_position();

	double current_pos_dist = cv::norm(EigenToCV(current_pos_eigen - current_pt));
	double prev_pos_dist = cv::norm(EigenToCV(prev_pos_eigen - prev_pt));

	if (current_pos_dist < prev_pos_dist) {
		dist_near = current_pos_dist;
		pt_near = current_pt;
		dist_far = prev_pos_dist;
		pt_far = prev_pt;
	} else {
		dist_near = prev_pos_dist;
		pt_near = prev_pt;
		dist_far = current_pos_dist;
		pt_far = current_pt;
	}

	alpha = dist_near / (2 * dist_far);

	return ((1 - alpha) * pt_near + alpha * pt_far);
}

Eigen::Vector3d fuseInverseDepth(CameraPose::Ptr current_pos, Eigen::Vector3d current_pt,
								 CameraPose::Ptr prev_pos, Eigen::Vector3d prev_pt)
{
	/* Transform current and prev poitns to camera coordinate system */
	Eigen::Vector3d current_pt_camera = current_pos->ToCamera( current_pt );
	Eigen::Vector3d prev_pt_camera = prev_pos->ToCamera( prev_pt );

	/* Compute the euclidean distance of the fusion point using inverse-depth representation */
	double inverse_depth_current = 1.0 / current_pt_camera.norm();
	double inverse_depth_prev = 1.0 / prev_pt_camera.norm();

	/* fuse inverse depths */
	double depth_fusion = (inverse_depth_current + inverse_depth_prev) / 2.0;

	/* Compute unit vector of current camera */
	Eigen::Vector3d unit_vector_pt = current_pt_camera / current_pt_camera.norm();

	/* Compute fusion point in current camera coordinate system */
	Eigen::Vector3d fusion_pt_camera = (1.0 / depth_fusion) * unit_vector_pt;

	/* Transform fusion point to world coordinate system */
	Eigen::Vector3d pt_fusion = current_pos->ToWorld(fusion_pt_camera);

	return pt_fusion;
}

void ProjectionThread::doStereoscan(PointCloudEntry::Ptr prev_entry, DispImagePtr disp_img,
									FrustumCulling *frustum_left, FrustumCulling *frustum_right,
									CameraPose::Ptr current_pos, double stereoscan_threshold,
									cv::Mat_<int> *match_mat, PointCloudPtr current_cloud)
{
	if (!stereoscan_threshold)
		return;

	PointCloudPtr new_prev_cloud(new PointCloud);

	unsigned int status[STATUS_LENGTH] = { 0 };

	for (auto& it: *prev_entry->get_cloud()) {
		/*
		 * Points outside current -stereo- frustum of view are omitted,
		 * i.e. kept in its original cloud.
		 */
		if (!frustum_left->Contains(it.asEigen()) || !frustum_right->Contains(it.asEigen())) {
			status[STATUS_OUT_OF_IMAGE]++;
			new_prev_cloud->push_back(it);
			continue;
		}

		cv::Point3d prev_pt = EigenToCV(current_pos->ToCamera(it.asEigen()));
		cv::Point2i pixel = dense_->camera_->getStereoModel().left().project3dToPixel(prev_pt);

		/*
		 * FIXME: this condition shouldn't be satisfied after applying FrustumCulling.
		 * However, in some cases it is being satisfied, thus we must check it.
		 */
		if (pixel.y < 0 || pixel.x < 0 || disp_img->rows < pixel.y || disp_img->cols < pixel.x) {
			status[STATUS_OUT_OF_IMAGE]++;
			new_prev_cloud->push_back(it);
			continue;
		}

		float disp = disp_img->at<float>(pixel.y, pixel.x);

		/* Pixels with invalid disparity are omitted */
		if (!finite(disp) || disp <= 0) {
			status[STATUS_INVALID_DISPARITY]++;
			new_prev_cloud->push_back(it);
			continue;
		}

		cv::Point3d new_pt_cv;
		dense_->camera_->getStereoModel().projectDisparityTo3d(pixel, disp, new_pt_cv);

		/* StereoScan part */
		if (cv::norm(new_pt_cv - prev_pt) < stereoscan_threshold) {
			CameraPose::Ptr prev_pos = prev_entry->get_current_pos();
			Eigen::Vector3d new_pt_eigen = current_pos->ToWorld(CVToEigen(new_pt_cv));

			if (fusionHeuristic)
				it.fromEigen(fusionHeuristic(current_pos, new_pt_eigen, prev_pos, it.asEigen()));

			it.validate();

			current_cloud->push_back(it);
			/* Mark pixel as matched - Don't triangulate a new point */
			match_mat->at<int>(pixel.y, pixel.x) = 1;

			status[STATUS_MATCH]++;
			continue;
		}

		if (new_pt_cv.z < prev_pt.z) {
			/*
			 * Don't discard points that were behind the new one.
			 * We consider that those points belong to different objects, thus
			 * the point is being occluded and that's why is doesn't get observed
			 * on the image plane.
			 */
			new_prev_cloud->push_back(it);
			status[STATUS_UNMATCH]++;
			continue;
		}

		it.invalidate();

		if (!it.isOutlier()) {
			new_prev_cloud->push_back(it);
			status[STATUS_UNMATCH]++;
			continue;
		}

		status[STATUS_OUTLIER]++;
	}

	log_data.match += status[STATUS_MATCH];
	log_data.unmatch += status[STATUS_UNMATCH];
	log_data.outlier += status[STATUS_OUTLIER];

	ROS_DEBUG("out_of_image/invalid = %u/%u,\tmatch = %u,\tunmatch/outlier = %u/%u",
			  status[STATUS_OUT_OF_IMAGE], status[STATUS_INVALID_DISPARITY], status[STATUS_MATCH],
			  status[STATUS_UNMATCH], status[STATUS_OUTLIER]);

	prev_entry->set_cloud(new_prev_cloud);
}

/*
 * downsampleCloud
 *
 * Downsample @cloud using a voxel grid with a leaf size of @voxelLeafSize.
 * If @voxelLeafSize is 0, not downsample is performed.
 */
void downsampleCloud(PointCloudPtr cloud, double voxelLeafSize)
{
	if (!voxelLeafSize)
		return;

	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud2_filtered(new pcl::PCLPointCloud2());
	pcl::VoxelGrid<pcl::PCLPointCloud2> vgrid;

	pcl::toPCLPointCloud2(*cloud, *cloud2);
	vgrid.setInputCloud(cloud2);
	vgrid.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
	vgrid.filter(*cloud2_filtered);
	pcl::fromPCLPointCloud2(*cloud2_filtered, *cloud);
}
