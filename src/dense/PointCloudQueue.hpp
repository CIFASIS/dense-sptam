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

#ifndef __POINTCLOUD_QUEUE_H
#define __POINTCLOUD_QUEUE_H

#include <mutex>
#include <queue>
#include <condition_variable>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>
#include "pcl/impl/point_types.hpp"

#include "Camera.hpp"
#include "DispImageQueue.hpp"

/* Initial probability for new points. */
#define POINT_NEW_PROBABILITY			1

/* Probability threshold. Below this value, points are considered outliers. */
#define POINT_OUTLIER_PROBABILITY		1

inline Eigen::Vector3d CVToEigen(const cv::Point3d& pt)
{ return Eigen::Vector3d(pt.x, pt.y, pt.z); }

inline cv::Point3d EigenToCV(const Eigen::Vector3d& pt)
{ return cv::Point3d(pt(0), pt(1), pt(2)); }

class Point : public pcl::PointXYZRGB
{

public:

	Point() : pcl::PointXYZRGB()
	{ probability = POINT_NEW_PROBABILITY; }

	inline cv::Point3d asCV(void)
	{ return cv::Point3d(this->x, this->y, this->z); }

	inline Eigen::Vector3d asEigen(void)
	{ return Eigen::Vector3d(this->x, this->y, this->z); }

	inline void fromCV(const cv::Point3d& pt)
	{ this->x = pt.x; this->y = pt.y; this->z = pt.z; }

	inline void fromEigen(const Eigen::Vector3d& pt)
	{ this->x = pt(0); this->y = pt(1); this->z = pt(2); }

	inline void validate()
	{ probability += 1; }

	inline void invalidate()
	{ probability -= 1; }

	inline bool isOutlier()
	{ return probability < POINT_OUTLIER_PROBABILITY; }

	float probability;

};

POINT_CLOUD_REGISTER_POINT_STRUCT (Point,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, rgb, rgb)
	(float, probability, probability)
)

typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef cv::Mat_<cv::Vec3f> MatVec3f;
typedef boost::shared_ptr<MatVec3f> MatVec3fPtr;

class PointCloudEntry
{

public:

	typedef enum EntryState_e {
		LOCAL_MAP,
		GLOBAL_MAP_RAM,
		GLOBAL_MAP_SWAP
	} EntryState;

	typedef boost::shared_ptr<PointCloudEntry> Ptr;

	PointCloudEntry(uint32_t seq);
	~PointCloudEntry();

	int save_cloud(const char *output_dir, std::map<uint32_t, CameraPose::Ptr>& poses);
	int load_cloud(const char *output_dir);

	inline uint32_t get_seq()
	{ return seq_; }
	inline CameraPose::Ptr get_current_pos()
	{ return current_pos_; }
	inline CameraPose::Ptr get_update_pos()
	{ return update_pos_; }
	inline DispRawImagePtr get_disp_raw_img()
	{ return disp_raw_img_; }
	inline MatVec3fPtr get_points_mat()
	{ return points_mat_; }
	inline PointCloudPtr get_cloud()
	{ return cloud_; }
	inline EntryState get_state()
	{ return state_; }
	inline CameraPose::TransformPtr get_transform()
	{ return base_to_camera_; }

	inline void set_current_pos(CameraPose::Ptr current_pos)
	{ current_pos_ = current_pos; }
	inline void set_update_pos(CameraPose::Ptr update_pos)
	{ update_pos_ = update_pos; }
	inline void set_disp_raw_img(DispRawImagePtr disp_raw_img)
	{ disp_raw_img_ = disp_raw_img; }
	inline void set_points_mat(MatVec3fPtr points_mat)
	{ points_mat_ = points_mat; }
	inline void set_cloud(PointCloudPtr cloud)
	{ cloud_ = cloud; }
	inline void set_state(EntryState state)
	{ state_ = state; }
	inline void set_transform(CameraPose::TransformPtr base_to_camera)
	{ base_to_camera_ = base_to_camera; }

	inline void lock()
	{ entry_lock_.lock(); }
	inline void unlock()
	{ entry_lock_.unlock(); }

private:

	uint32_t seq_;
	CameraPose::Ptr current_pos_;
	CameraPose::Ptr update_pos_;

	CameraPose::TransformPtr base_to_camera_;

	DispRawImagePtr disp_raw_img_;
	MatVec3fPtr points_mat_;
	PointCloudPtr cloud_;

	EntryState state_;

	std::mutex entry_lock_;

};

class PointCloudQueue
{

public:

	PointCloudQueue(unsigned max_local_area_size);
	~PointCloudQueue();

	size_t size();

	void generate_poses_txt(const char *output_dir);
	void save_all(const char *output_dir);
	void push_local_area(PointCloudEntry::Ptr entry);
	uint32_t get_local_area_seq();
	void get_local_area_cloud(PointCloudPtr ret);

	std::unordered_map<uint32_t, PointCloudEntry::Ptr> entries_;
	std::map<uint32_t, CameraPose::Ptr> poses_;

	PointCloudEntry::Ptr getEntry(uint32_t seq_, bool force = true);
	std::vector<PointCloudEntry::Ptr> local_area_queue_;

private:

	std::mutex vector_lock_;
	unsigned max_local_area_size_;

};

#endif /* __DISP_IMAGE_QUEUE_H */
