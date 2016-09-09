#include "TransformThread.hpp"

#include <cv_bridge/cv_bridge.h>

#include "FrustumCulling.hpp"

TransformThread::TransformThread(PointCloudQueue *point_clouds, Camera *camera)
  : point_clouds_(point_clouds)
  , camera_(camera)
  , transformThread_(&TransformThread::compute, this)
{}

void TransformThread::compute()
{
    while(1) {
        /* Blocking call */
        PointCloudEntry::Ptr entry = point_clouds_->popInit();
        PointCloudEntry::Ptr last_entry = point_clouds_->get_last_init();

        entry->lock();
        CameraPose::Ptr pose = entry->get_update_pos();
        entry->set_current_pos(pose);
        entry->set_update_pos(nullptr);
        entry->unlock();

        PointCloudPtr cloud = generateCloud(entry);
        cameraToWorld(cloud, pose);

        entry->lock();
        entry->set_disp_raw_img(nullptr);
        entry->set_points_mat(nullptr);
        entry->set_cloud(cloud);
        entry->set_state(PointCloudEntry::IDLE);
        point_clouds_->set_last_init(entry);
        point_clouds_->schedule(entry);
        entry->unlock();

        ROS_INFO("TransformThread::computed seq = %u (cloud_size = %lu) (queued = %lu)",
                 entry->get_seq(), cloud->size(), point_clouds_->sizeInitQueue());
    }
}

bool TransformThread::isValidPoint(const cv::Vec3f& pt)
{
    /*
     * Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
     * and zero disparities (point mapped to infinity).
     */
    return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !isinf(pt[2]);
}

PointCloudPtr TransformThread::generateCloud(PointCloudEntry::Ptr entry)
{
    MatVec3fPtr dense_points_ = entry->get_points_mat();
    ImagePtr raw_left_image = entry->get_disp_raw_img()->first;
    cv::Mat image_left(cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::TYPE_8UC1)->image);

    PointCloudPtr cloud(new PointCloud);
    pcl::PointXYZRGB new_pt3d;

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

void TransformThread::cameraToWorld(PointCloudPtr cloud, CameraPose::Ptr current_pos)
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
