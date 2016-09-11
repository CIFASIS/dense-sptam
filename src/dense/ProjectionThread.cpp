#include "ProjectionThread.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "dense.hpp"

ProjectionThread::ProjectionThread(Dense *dense)
  : dense_(dense)
  , projectionThread_(&ProjectionThread::compute, this)
{}

void ProjectionThread::compute()
{
    while(1) {
        /* Calls to pop() are blocking */
        DispRawImagePtr disp_raw_img = dense_->disp_images_->pop();

        PointCloudEntry::Ptr entry = dense_->point_clouds_->getEntry(disp_raw_img->first->header.seq);
        PointCloudEntry::Ptr last_entry = dense_->point_clouds_->get_last_init();

        entry->lock();
        CameraPose::Ptr pose = entry->get_update_pos();
        entry->set_current_pos(pose);
        entry->set_update_pos(nullptr);
        entry->unlock();

        assert(pose != nullptr);

        filterDisp(disp_raw_img, MIN_DISPARITY_THRESHOLD);
        PointCloudPtr cloud = generateCloud(disp_raw_img);
        cameraToWorld(cloud, pose);
        downsampleCloud(cloud, dense_->voxelLeafSize_);

        entry->lock();
        entry->set_cloud(cloud);
        entry->set_state(PointCloudEntry::IDLE);
        dense_->point_clouds_->set_last_init(entry);
        dense_->point_clouds_->schedule(entry);
        entry->unlock();

        ROS_INFO("ProjectionThread::computed seq = %u (cloud_size = %lu) (queued = %lu)",
                 entry->get_seq(), cloud->size(), dense_->point_clouds_->sizeInitQueue());
    }
}

void ProjectionThread::filterDisp(const DispRawImagePtr disp_raw_img, float min_disparity)
{
    ImagePtr raw_left_image = disp_raw_img->first;
    DispImagePtr disp_img = disp_raw_img->second;

    for (unsigned int i = 0; i < raw_left_image->height; i++)
        for (unsigned int j = 0; j < raw_left_image->width; j++)
            if (disp_img->at<float>(i, j) < min_disparity)
                disp_img->at<float>(i, j) = 0;
}

bool ProjectionThread::isValidPoint(const cv::Vec3f& pt)
{
    /*
     * Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
     * and zero disparities (point mapped to infinity).
     */
    return pt[2] != MY_MISSING_Z && !isinf(pt[2]);
}

PointCloudPtr ProjectionThread::generateCloud(DispRawImagePtr disp_raw_img)
{
    ImagePtr raw_left_image = disp_raw_img->first;
    DispImagePtr disp_img = disp_raw_img->second;
    MatVec3fPtr dense_points_(new MatVec3f);
    PointCloudPtr cloud(new PointCloud);
    pcl::PointXYZRGB new_pt3d;

    cv::Mat image_left(cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::TYPE_8UC1)->image);

    dense_->camera_ ->getStereoModel().projectDisparityImageTo3d(*disp_img, *dense_points_, true);

    for (int32_t u = 0; u < dense_points_->rows; ++u)
        for (int32_t v = 0; v < dense_points_->cols; ++v)
            if (isValidPoint((*dense_points_)(u,v))) {
                memcpy(&new_pt3d.x, &(*dense_points_)(u,v)[0], sizeof (float));
                memcpy(&new_pt3d.y, &(*dense_points_)(u,v)[1], sizeof (float));
                memcpy(&new_pt3d.z, &(*dense_points_)(u,v)[2], sizeof (float));
                uint8_t g = image_left.at<uint8_t>(u,v);
                int32_t rgb = (g << 16) | (g << 8) | g;
                memcpy(&new_pt3d.rgb, &rgb, sizeof (int32_t));

                cloud->push_back(new_pt3d);
            }

    return cloud;
}

void ProjectionThread::cameraToWorld(PointCloudPtr cloud, CameraPose::Ptr current_pos)
{
    CameraPose::Position pos;

    /*
     * Last point cloud
     * Filter with this camera -> Frustum culling
     * Reproject filtered point cloud to disparity
     * Compare with actual disparity
     */

    for (auto& it: *cloud) {
        pos(0) = it.x;
        pos(1) = it.y;
        pos(2) = it.z;
        pos = current_pos->ToWorld(pos);
        it.x = pos(0);
        it.y = pos(1);
        it.z = pos(2);
    }
}

void ProjectionThread::downsampleCloud(PointCloudPtr cloud, double voxelLeafSize)
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
