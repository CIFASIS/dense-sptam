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

    typedef boost::shared_ptr<PointCloudEntry> Ptr;

    PointCloudEntry(uint32_t seq);
    ~PointCloudEntry();

    inline uint32_t get_seq()
    { return seq_; }
    inline CameraPose::Ptr get_current_pos()
    { return current_pos_; }
    inline CameraPose::Ptr get_update_pos()
    { return update_pos_; }
    inline PointCloudPtr get_cloud()
    { return cloud_; }

    inline void set_current_pos(CameraPose::Ptr current_pos)
    { current_pos_ = current_pos; }
    inline void set_update_pos(CameraPose::Ptr update_pos)
    { update_pos_ = update_pos; }
    inline void set_cloud(PointCloudPtr cloud)
    { cloud_ = cloud; }

private:

    uint32_t seq_;
    CameraPose::Ptr current_pos_;
    CameraPose::Ptr update_pos_;
    PointCloudPtr cloud_;

};

class PointCloudQueue
{
public:

    PointCloudQueue();
    ~PointCloudQueue();

    size_t size();

    PointCloudEntry::Ptr setCurrentPos(uint32_t seq, CameraPose::Ptr current_pos);
    PointCloudEntry::Ptr setUpdatePos(uint32_t seq, CameraPose::Ptr update_pos);
    PointCloudEntry::Ptr setCloud(uint32_t seq, PointCloudPtr cloud_pos);

private:

    PointCloudEntry::Ptr getEntry(uint32_t seq_, bool force = false);

    std::mutex queue_lock_;

    /* TODO: Please, use something more efficient here! HINT: a HASH table. */
    std::vector<PointCloudEntry::Ptr> entries_;

private:

};

#endif /* __DISP_IMAGE_QUEUE_H */
