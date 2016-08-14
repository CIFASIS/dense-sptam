#ifndef __POINTCLOUD_QUEUE_H
#define __POINTCLOUD_QUEUE_H

#include <mutex>
#include <pcl_ros/point_cloud.h>

#include "Camera.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

class PointCloudEntry
{
public:

    PointCloudEntry(uint32_t seq, PointCloudPtr cloud);
    ~PointCloudEntry();

private:

    uint32_t seq_;
    CameraPose::Ptr current_;
    CameraPose::Ptr next_;
    PointCloudPtr cloud_;

};

class PointCloudQueue
{
public:

    PointCloudQueue();
    ~PointCloudQueue();

    void push(PointCloudEntry entry);
    size_t size();

private:

    std::mutex queue_lock_;
    std::vector<PointCloudEntry> entries_;

private:

};

#endif /* __DISP_IMAGE_QUEUE_H */
