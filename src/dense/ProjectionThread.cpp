#include "ProjectionThread.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include "dense.hpp"
#include "../../../sptam/src/sptam/utils/Time.hpp"

ProjectionThread::ProjectionThread(Dense *dense)
  : dense_(dense)
  , projectionThread_(&ProjectionThread::compute, this)
{}

void ProjectionThread::compute()
{
    char log_buffer[512];
    unsigned int log_data[3];
    double time_t[3];
    int i;

    while(1) {
        /* Calls to pop() are blocking */
        DispRawImagePtr disp_raw_img = dense_->disp_images_->pop();

        PointCloudEntry::Ptr entry = dense_->point_clouds_->getEntry(disp_raw_img->first->header.seq);
        assert(entry);

        entry->lock();
        time_t[0] = GetSeg();

        CameraPose::Ptr current_pose = entry->get_update_pos();
        if(!current_pose) {
            ROS_INFO("##### WARNING: Keyframe %u omitted, no pose! #####", entry->get_seq());
            entry->unlock();
            continue;
        }

        entry->set_current_pos(current_pose);
        entry->set_update_pos(nullptr);
        /* Check this with Taihú */
        /*
        CameraPose::Ptr pose_left =
                boost::make_shared<CameraPose>(current_pose->applyTransform(entry->get_transform()));
        */
        CameraPose::Ptr pose_left = current_pose;

        entry->unlock();

        CameraPose::Ptr pose_right =
                boost::make_shared<CameraPose>(dense_->camera_->ComputeRightCameraPose(*pose_left));

        FrustumCulling frustum_left(pose_left->get_position(), pose_left->get_orientation_matrix(),
                                    dense_->camera_->GetFOV_LH(), dense_->camera_->GetFOV_LV(),
                                    dense_->camera_->getNearPlaneDist(), dense_->camera_->getFarPlaneDist());

        FrustumCulling frustum_right(pose_right->get_position(), pose_right->get_orientation_matrix(),
                                     dense_->camera_->GetFOV_LH(), dense_->camera_->GetFOV_LV(),
                                     dense_->camera_->getNearPlaneDist(), dense_->camera_->getFarPlaneDist());

        for (i = 0; i < 3; i++)
            log_data[i] = 0;

        for (auto& local_area_entry : dense_->point_clouds_->local_area_queue_) {
            /* No need to lock the entry as no one else will alter its current_pose or cloud */
            PointCloudPtr last_cloud = doStereoscan(local_area_entry->get_cloud(), disp_raw_img->second,
                                                    &frustum_left, &frustum_right,
                                                    pose_left, dense_->stereoscan_threshold_, log_data);
            if (last_cloud)
                local_area_entry->set_cloud(last_cloud);
        }

        time_t[1] = GetSeg();

        sprintf(log_buffer, "stereoscan,%u,%f,%u,%u,%u\n", entry->get_seq(),
                time_t[1] - time_t[0], log_data[0], log_data[1], log_data[2]);
        dense_->WriteToLog(log_buffer);

        filterDisp(disp_raw_img);
        PointCloudPtr cloud = my_generateCloud(disp_raw_img);
        cameraToWorld(cloud, pose_left);

        //downsampleCloud(cloud, dense_->voxelLeafSize_);
        //statisticalFilterCloud(cloud, dense_->filter_meanK_, dense_->filter_stddev_);
        //radiusFilterCloud(cloud, dense_->filter_radius_, dense_->filter_minneighbours_);

        time_t[2] = GetSeg();
        entry->lock();
        entry->set_cloud(cloud);
        dense_->point_clouds_->push_local_area(entry);
        entry->unlock();

        sprintf(log_buffer, "projection,%u,%f\n", entry->get_seq(), time_t[2] - time_t[1]);
        dense_->WriteToLog(log_buffer);
        ROS_INFO("Projected seq %u (size = %lu, time = %f)", entry->get_seq(), cloud->size(),
                 time_t[2] - time_t[0]);
    }
}

unsigned ProjectionThread::calculateValidDisp(const DispRawImagePtr disp_raw_img)
{
    ImagePtr raw_left_image = disp_raw_img->first;
    DispImagePtr disp_img = disp_raw_img->second;
    unsigned count = 0;

    for (unsigned int i = 0; i < raw_left_image->height; i++)
        for (unsigned int j = 0; j < raw_left_image->width; j++)
            if (isValidDisparity(disp_img->at<float>(i, j)))
                    count++;

    return count;
}

void ProjectionThread::filterDisp(const DispRawImagePtr disp_raw_img)
{
    ImagePtr raw_left_image = disp_raw_img->first;
    DispImagePtr disp_img = disp_raw_img->second;

    for (unsigned int i = 0; i < raw_left_image->height; i++)
        for (unsigned int j = 0; j < raw_left_image->width; j++)
            if (!isValidDisparity(disp_img->at<float>(i, j)))
                disp_img->at<float>(i, j) = PIXEL_DISP_INVALID;
}

bool ProjectionThread::isValidDisparity(const float disp)
{
    if (!finite(disp) || disp <= 0)
        return false;
    double dist = dense_->camera_->getStereoModel().getZ(disp);
    assert(finite(dist));
    return dist <= dense_->max_distance_;
}

bool ProjectionThread::isValidPoint(const cv::Vec3f& pt)
{
    /*
     * Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
     * and zero disparities (point mapped to infinity).
     */
    return pt[2] != MY_MISSING_Z && !isinf(pt[2]);
}

PointCloudPtr ProjectionThread::my_generateCloud(DispRawImagePtr disp_raw_img)
{
    ImagePtr raw_left_image = disp_raw_img->first;
    DispImagePtr disp_img = disp_raw_img->second;
    PointCloudPtr cloud(new PointCloud);

    cv::Mat image_left(cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::MONO8)->image);

    for (unsigned int i = 0; i < raw_left_image->height; i++) {
        for (unsigned int j = 0; j < raw_left_image->width; j++) {
            if (!isValidDisparity(disp_img->at<float>(i, j)))
                continue;

            Point new_pt3d;
            cv::Point3d point;
            cv::Point2d pixel(j, i);

            dense_->camera_->getStereoModel().projectDisparityTo3d(pixel, disp_img->at<float>(i, j), point);

            new_pt3d.x = point.x;
            new_pt3d.y = point.y;
            new_pt3d.z = point.z;
            new_pt3d.a = 1;

            uint8_t g = image_left.at<uint8_t>(i, j);
            int32_t rgb = (g << 16) | (g << 8) | g;
            memcpy(&new_pt3d.rgb, &rgb, sizeof (int32_t));

            cloud->push_back(new_pt3d);
        }
    }

    return cloud;
}


PointCloudPtr ProjectionThread::generateCloud(DispRawImagePtr disp_raw_img)
{
    ImagePtr raw_left_image = disp_raw_img->first;
    DispImagePtr disp_img = disp_raw_img->second;
    MatVec3fPtr dense_points_(new MatVec3f);
    PointCloudPtr cloud(new PointCloud);
    Point new_pt3d;

    cv::Mat image_left(cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::MONO8)->image);

    dense_->camera_ ->getStereoModel().projectDisparityImageTo3d(*disp_img, *dense_points_, true);

    for (int32_t u = 0; u < dense_points_->rows; ++u)
        for (int32_t v = 0; v < dense_points_->cols; ++v)
            if (isValidPoint((*dense_points_)(u,v))) {
                memcpy(&new_pt3d.x, &(*dense_points_)(u,v)[0], sizeof (float));
                memcpy(&new_pt3d.y, &(*dense_points_)(u,v)[1], sizeof (float));
                memcpy(&new_pt3d.z, &(*dense_points_)(u,v)[2], sizeof (float));
                new_pt3d.a = 1;
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

void ProjectionThread::statisticalFilterCloud(PointCloudPtr cloud, double filter_meanK, double filter_stddev)
{
    if (!filter_meanK || !filter_stddev)
        return;

    pcl::StatisticalOutlierRemoval<Point> sor;

    sor.setInputCloud(cloud);
    sor.setMeanK(filter_meanK);
    sor.setStddevMulThresh(filter_stddev);
    sor.filter(*cloud);
}

void ProjectionThread::radiusFilterCloud(PointCloudPtr cloud, double filter_radius, double filter_minneighbours)
{
    if (!filter_radius || !filter_minneighbours)
        return;

    pcl::RadiusOutlierRemoval<Point> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(filter_radius);
    outrem.setMinNeighborsInRadius(filter_minneighbours);
    outrem.filter(*cloud);

}

PointCloudPtr ProjectionThread::doStereoscan(PointCloudPtr last_cloud, DispImagePtr disp_img,
                                             FrustumCulling *frustum_left, FrustumCulling *frustum_right,
                                             CameraPose::Ptr current_pos, double stereoscan_threshold,
                                             unsigned int log_data[])
{
    char log_buffer[512];

    if (!stereoscan_threshold)
        return nullptr;

    assert(frustum_left && frustum_right);

    PointCloudPtr new_last_cloud(new PointCloud);

    CameraPose::Position pos;
    unsigned invisible = 0, corner = 0, match = 0, unmatch = 0, outlier = 0;

    for (auto& it: *last_cloud) {
        pos(0) = it.x;
        pos(1) = it.y;
        pos(2) = it.z;

        /* Keep points outside current -stereo- frustum of view */
        if (!frustum_left->Contains(pos) || !frustum_right->Contains(pos)) {
            invisible++;
            new_last_cloud->push_back(it);
            continue;
        }

        pos = current_pos->ToCamera(pos);
        cv::Point3d cvpos(pos(0), pos(1), pos(2));

        cv::Point2i pixel = dense_->camera_->getStereoModel().left().project3dToPixel(cvpos);

        /* Why is this condition satisfied after FrustumCulling filtering? */
        if (pixel.y < 0 || pixel.x < 0 || disp_img->rows < pixel.y || disp_img->cols < pixel.x) {
            invisible++;
            new_last_cloud->push_back(it);
            continue;
        }

        /* Pixels marked as corners (not interpolated) are omitted */
        if (disp_img->at<float>(pixel.y, pixel.x) == PIXEL_DISP_CORNER) {
            corner++;
            new_last_cloud->push_back(it);
            continue;
        }

        /* Pixels marked as invalid disparity are  */
        if (disp_img->at<float>(pixel.y, pixel.x) == PIXEL_DISP_INVALID) {
            corner++;
            new_last_cloud->push_back(it);
            continue;
        }

        if (!finite(disp_img->at<float>(pixel.y, pixel.x)) || disp_img->at<float>(pixel.y, pixel.x) <= 0) {
            corner++;
            new_last_cloud->push_back(it);
            continue;
        }

        /* TODO: check on disparity validity */
        double dist = dense_->camera_->getStereoModel().getZ(disp_img->at<float>(pixel.y, pixel.x));
        assert(finite(dist) && finite(cvpos.z));

        /* StereoScan trick */
        if (std::abs(dist - cvpos.z) < stereoscan_threshold) {
            float new_dist = (dist + cvpos.z) / 2;
            CameraPose::Position new_pos(pos(0), pos(1), new_dist);
            new_pos = current_pos->ToWorld(new_pos);
            it.x = new_pos(0);
            it.y = new_pos(1);
            it.z = new_pos(2);
            it.a++;
            new_last_cloud->push_back(it);
            disp_img->at<float>(pixel.y, pixel.x) = PIXEL_DISP_INVALID;
            match++;
            continue;
        }

        /* Decrement point probability (just a simple counter) */
        if (it.a > 1) {
            it.a--;
            new_last_cloud->push_back(it);
            unmatch++;
            continue;
        } else {
            outlier++;
        }
    }

    log_data[0] += match;
    log_data[1] += unmatch;
    log_data[2] += outlier;
    ROS_DEBUG("invisible/corner = %u/%u,\tmatch = %u,\tunmatch/outlier = %u/%u",
             invisible, corner, match, unmatch, outlier);
    return new_last_cloud;
}

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
