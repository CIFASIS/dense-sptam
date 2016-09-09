#include "RefinementThread.hpp"

#include "dense.hpp"

RefinementThread::RefinementThread(Dense *dense)
  : dense_(dense)
  , refinementThread_(&RefinementThread::compute, this)
{}

void RefinementThread::compute()
{
    while(1) {
        /* Blocking call */
        PointCloudEntry::Ptr entry = dense_->point_clouds_->popRefine();

        entry->lock();
        entry->set_current_pos(entry->get_update_pos());
        entry->set_update_pos(nullptr);
        entry->unlock();

        //refinementwork()
        //ROS_INFO("Refined seq = %u", entry->get_seq());

        entry->lock();
        entry->set_state(PointCloudEntry::IDLE);
        dense_->point_clouds_->schedule(entry);
        entry->unlock();

        ROS_DEBUG("RefinementThread::computed seq = %u (queued = %lu)",
                 entry->get_seq(), dense_->point_clouds_->sizeRefineQueue());
    }
}
