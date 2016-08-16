#include "RefinementThread.hpp"

RefinementThread::RefinementThread(
    PointCloudQueue *point_clouds
) : point_clouds_(point_clouds)
  , refinementThread_(&RefinementThread::compute, this)
{}

void RefinementThread::compute()
{
    while(1) {
        /* Blocking call */
        PointCloudEntry::Ptr entry = point_clouds_->popRefine();

        entry->lock();
        entry->set_current_pos(entry->get_update_pos());
        entry->set_update_pos(nullptr);
        entry->unlock();

        //refinementwork()
        //ROS_INFO("Refined seq = %u", entry->get_seq());

        entry->lock();
        entry->set_state(PointCloudEntry::IDLE);
        point_clouds_->schedule(entry);
        entry->unlock();
    }
}
