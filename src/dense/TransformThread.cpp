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

        cameraToWorld(entry);

        entry->lock();
        entry->set_state(PointCloudEntry::IDLE);
        point_clouds_->schedule(entry);
        entry->unlock();
    }
}

void TransformThread::cameraToWorld(PointCloudEntry::Ptr entry)
{
    CameraPose::Position pos;

    for (auto& it: *entry->get_cloud()) {
        pos(0) = it.x;
        pos(1) = it.y;
        pos(2) = it.z;
        pos = entry->get_current_pos()->ToWorld(pos);
        it.x = pos(0);
        it.y = pos(1);
        it.z = pos(2);
    }
}
