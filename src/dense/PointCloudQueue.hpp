#ifndef __POINTCLOUD_QUEUE_H
#define __POINTCLOUD_QUEUE_H

#include <mutex>
#include <queue>
#include <condition_variable>
#include <unordered_map>
#include <pcl_ros/point_cloud.h>

#include "Camera.hpp"
#include "DispImageQueue.hpp"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;

typedef cv::Mat_<cv::Vec3f> MatVec3f;
typedef boost::shared_ptr<MatVec3f> MatVec3fPtr;

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

    int save_cloud();
    int load_cloud();

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

    inline void lock()
    { entry_lock_.lock(); }
    inline void unlock()
    { entry_lock_.unlock(); }

private:

    uint32_t seq_;
    CameraPose::Ptr current_pos_;
    CameraPose::Ptr update_pos_;

    DispRawImagePtr disp_raw_img_;
    MatVec3fPtr points_mat_;
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

    inline void set_last_init(PointCloudEntry::Ptr entry)
    { last_init_ = entry; }
    PointCloudEntry::Ptr get_last_init()
    { return last_init_; }

    PointCloudEntry::Ptr getEntry(uint32_t seq_, bool force = true);
    PointCloudEntry::Ptr popInit(bool remove = true);
    PointCloudEntry::Ptr popRefine(bool remove = true);
    void schedule(PointCloudEntry::Ptr entry);

    PointCloudPtr get_global_cloud();

private:

    std::mutex vector_lock_, init_queue_lock_, refine_queue_lock_;
    std::condition_variable empty_init_queue_cv, empty_refine_queue_cv;

    std::unordered_map<uint32_t, PointCloudEntry::Ptr> entries_;
    std::queue<PointCloudEntry::Ptr> init_queue_, refine_queue_;

    PointCloudEntry::Ptr last_init_;

private:

};

#endif /* __DISP_IMAGE_QUEUE_H */
