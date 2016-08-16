#ifndef __POINTCLOUD_QUEUE_H
#define __POINTCLOUD_QUEUE_H

#include <mutex>
#include <queue>
#include <condition_variable>
#include <pcl_ros/point_cloud.h>

#include "Camera.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;

class PointCloudEntry
{
public:

    typedef enum EntryState_e {
        CREATED,
        INIT_QUEUE,
        IDLE,
        REFINE_QUEUE
    } EntryState;

    typedef boost::shared_ptr<PointCloudEntry> Ptr;

    PointCloudEntry(uint32_t seq);
    ~PointCloudEntry();

    uint32_t get_seq()
    { return seq_; }
    CameraPose::Ptr get_current_pos()
    { return current_pos_; }
    CameraPose::Ptr get_update_pos()
    { return update_pos_; }
    PointCloudPtr get_cloud()
    { return cloud_; }
    EntryState get_state()
    { return state_; }

    inline void set_current_pos(CameraPose::Ptr current_pos)
    { current_pos_ = current_pos; }
    inline void set_update_pos(CameraPose::Ptr update_pos)
    { update_pos_ = update_pos; }
    inline void set_cloud(PointCloudPtr cloud)
    { cloud_ = cloud; }
    inline void set_state(EntryState state)
    { state_ = state; }

    inline void lock()
    { entry_lock_.lock(); }
    inline void unlock()
    { entry_lock_.unlock(); }

private:

    uint32_t seq_;
    CameraPose::Ptr current_pos_;
    CameraPose::Ptr update_pos_;
    PointCloudPtr cloud_;
    EntryState state_;

    std::mutex entry_lock_;

};

class PointCloudQueue
{
public:

    PointCloudQueue();
    ~PointCloudQueue();

    size_t size();
    size_t sizeInitQueue();
    size_t sizeRefineQueue();

    PointCloudEntry::Ptr getEntry(uint32_t seq_, bool force = true);
    PointCloudEntry::Ptr popInit(bool remove = true);
    PointCloudEntry::Ptr popRefine(bool remove = true);
    void schedule(PointCloudEntry::Ptr entry);

private:

    std::mutex vector_lock_, init_queue_lock_, refine_queue_lock_;
    std::condition_variable empty_init_queue_cv, empty_refine_queue_cv;

    /* TODO: Please, use something more efficient here! HINT: a HASH table. */
    std::vector<PointCloudEntry::Ptr> entries_;
    std::queue<PointCloudEntry::Ptr> init_queue_, refine_queue_;

private:

};

#endif /* __DISP_IMAGE_QUEUE_H */
