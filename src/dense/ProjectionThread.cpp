#include "ProjectionThread.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "dense.hpp"
#include "../utils/Time.hpp"

#define OUTLIER_VIEWS_THRESHOLD         1

ProjectionThread::ProjectionThread(Dense *dense)
  : dense_(dense)
  , projectionThread_(&ProjectionThread::compute, this)
{}

void ProjectionThread::compute()
{
    struct projection_log log_data;

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
                boost::make_shared<CameraPose>(dense_->camera_->ComputeRightCameraPose(*pose_left));

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

        for (auto& local_area_entry : dense_->point_clouds_->local_area_queue_) {
            /* No need to lock the entry as no one else will alter its current_pose or cloud */
            PointCloudPtr last_cloud = doStereoscan(local_area_entry->get_cloud(), disp_raw_img->second,
                                                    &frustum_left, &frustum_right,
                                                    pose_left, dense_->stereoscan_threshold_, &log_data, match_mat);
            if (last_cloud)
                local_area_entry->set_cloud(last_cloud);
        }

        log_data.time_t[1] = GetSeg();

        dense_->WriteToLog("stereoscan,%u,%f,%u,%u,%u\n", entry->get_seq(),
                           log_data.time_t[1] - log_data.time_t[0],
                           log_data.match, log_data.unmatch, log_data.outlier);

        filterDisp(disp_raw_img);
        PointCloudPtr cloud = generateCloud(disp_raw_img, match_mat);
        cameraToWorld(cloud, pose_left);

        log_data.time_t[2] = GetSeg();

        entry->lock();
        entry->set_cloud(cloud);
        dense_->point_clouds_->push_local_area(entry);
        entry->unlock();

        dense_->WriteToLog("projection,%u,%f\n", entry->get_seq(),
                           log_data.time_t[2] - log_data.time_t[1]);
        ROS_INFO("Projected seq %u (size = %lu, time = %f)", entry->get_seq(),
                 cloud->size(), log_data.time_t[2] - log_data.time_t[0]);
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
    return dense_->camera_->getStereoModel().getZ(disp) <= dense_->max_distance_;
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
            new_pt3d.x = point.x;
            new_pt3d.y = point.y;
            new_pt3d.z = point.z;
            /*
             * FIXME: we're using the alpha channel as the view-counter, which is used to
             * check a point should be discarded when it doesn't match.
             * We should add and use a different field/variable to points, or change
             * the way that points are check and discarded.
             */
            new_pt3d.a = 1;

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
    for (auto& it: *cloud) {
        Eigen::Vector3d pos(it.x, it.y, it.z);
        pos = current_pos->ToWorld(pos);
        it.x = pos(0);
        it.y = pos(1);
        it.z = pos(2);
    }
}

enum stereoscan_status {
    STATUS_OUT_OF_IMAGE,
    STATUS_INVALID,
    STATUS_MATCH,
    STATUS_UNMATCH,
    STATUS_OUTLIER,
    /* Sentinel - Length marker */
    STATUS_LENGTH,
};

PointCloudPtr ProjectionThread::doStereoscan(PointCloudPtr last_cloud, DispImagePtr disp_img,
                                             FrustumCulling *frustum_left, FrustumCulling *frustum_right,
                                             CameraPose::Ptr current_pos, double stereoscan_threshold,
                                             struct projection_log *log_data, cv::Mat_<int> *match_mat)
{
    if (!stereoscan_threshold)
        return nullptr;

    PointCloudPtr new_last_cloud(new PointCloud);

    unsigned int status[STATUS_LENGTH] = { 0 };

    for (auto& it: *last_cloud) {
        Eigen::Vector3d pos(it.x, it.y, it.z);

        /*
         * Points outside current -stereo- frustum of view are omitted,
         * i.e. kept in its original cloud.
         */
        if (!frustum_left->Contains(pos) || !frustum_right->Contains(pos)) {
            status[STATUS_OUT_OF_IMAGE]++;
            new_last_cloud->push_back(it);
            continue;
        }

        pos = current_pos->ToCamera(pos);
        cv::Point3d cvpos(pos(0), pos(1), pos(2));
        cv::Point2i pixel = dense_->camera_->getStereoModel().left().project3dToPixel(cvpos);

        /*
         * FIXME: this condition shouldn't be satisfied after applying FrustumCulling.
         * However, in some cases it is being satisfied, thus we must check it.
         */
        if (pixel.y < 0 || pixel.x < 0 || disp_img->rows < pixel.y || disp_img->cols < pixel.x) {
            status[STATUS_OUT_OF_IMAGE]++;
            new_last_cloud->push_back(it);
            continue;
        }

        float disp = disp_img->at<float>(pixel.y, pixel.x);

        /* Pixels with invalid disparity are omitted */
        if (!finite(disp) || disp <= 0) {
            status[STATUS_INVALID]++;
            new_last_cloud->push_back(it);
            continue;
        }

        cv::Point3d new_cvpos, diff;
        dense_->camera_->getStereoModel().projectDisparityTo3d(pixel, disp, new_cvpos);
        diff = new_cvpos - cvpos;

        /* StereoScan part */
        if (cv::norm(diff) < stereoscan_threshold) {
            cvpos += diff / 2;
            Eigen::Vector3d new_pos(cvpos.x, cvpos.y, cvpos.z);
            new_pos = current_pos->ToWorld(new_pos);

            it.x = new_pos(0);
            it.y = new_pos(1);
            it.z = new_pos(2);
            /*
             * Point matched, increment the view-counter.
             * NOTE: see alpha channel note above.
             */
            it.a++;

            new_last_cloud->push_back(it);
            /* Mark pixel as matched - Don't triangulate a new point */
            match_mat->at<int>(pixel.y, pixel.x) = 1;

            status[STATUS_MATCH]++;
            continue;
        }

        if (new_cvpos.z < cvpos.z) {
            /*
             * Don't discard points that were behind the new one.
             * We consider that those points belong to different objects, thus
             * the point is being occluded and that's why is doesn't get projected
             * on the image plane.
             */
            new_last_cloud->push_back(it);
            status[STATUS_UNMATCH]++;
            continue;
        }

        /* Point didn't match, decrement the view-counter. */
        if (it.a > OUTLIER_VIEWS_THRESHOLD) {
            it.a--;
            new_last_cloud->push_back(it);
            status[STATUS_UNMATCH]++;
            continue;
        }

        /*
         * Point didn't match and view-counter is below threshold,
         * thus the point is considered an outlier and discarded.
         */
        status[STATUS_OUTLIER]++;
    }

    log_data->match += status[STATUS_MATCH];
    log_data->unmatch += status[STATUS_UNMATCH];
    log_data->outlier += status[STATUS_OUTLIER];

    ROS_DEBUG("out_of_image/invalid = %u/%u,\tmatch = %u,\tunmatch/outlier = %u/%u",
              status[STATUS_OUT_OF_IMAGE], status[STATUS_INVALID], status[STATUS_MATCH],
              status[STATUS_UNMATCH], status[STATUS_OUTLIER]);

    return new_last_cloud;
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
