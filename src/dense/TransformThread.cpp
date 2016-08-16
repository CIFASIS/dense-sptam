#include "TransformThread.hpp"

TransformThread::TransformThread(
    PointCloudQueue *point_clouds
) : point_clouds_(point_clouds)
  , transformThread_(&TransformThread::compute, this)
{}

void TransformThread::compute()
{
    while(1) {
        /* Blocking call */
        PointCloudEntry::Ptr entry = point_clouds_->popInit();

        entry->lock();
        entry->set_current_pos(entry->get_update_pos());
        entry->set_update_pos(nullptr);
        entry->unlock();

        //cameraToWorld(entry->get_current_pos())
        ROS_INFO("Transformed seq = %u", entry->get_seq());

        entry->lock();
        entry->set_state(PointCloudEntry::IDLE);
        point_clouds_->schedule(entry);
        entry->unlock();
    }
}

//void TransformThread::cameraTo
