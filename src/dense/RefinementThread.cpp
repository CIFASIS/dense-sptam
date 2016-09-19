#include "RefinementThread.hpp"

#include "dense.hpp"

RefinementThread::RefinementThread(Dense *dense)
  : dense_(dense)
  , refinementThread_(&RefinementThread::compute, this)
{}

void RefinementThread::compute()
{
    while(1) {
        for (auto& it : dense_->point_clouds_->entries_) {
            it.second->lock();
            CameraPose::Ptr current_pose = it.second->get_current_pos();
            CameraPose::Ptr update_pose = it.second->get_update_pos();
            it.second->unlock();

            if (it.second->get_state() == PointCloudEntry::LOCAL_MAP)
                continue;

            assert(current_pose != nullptr);

            if (update_pose != nullptr && current_pose->distance(*update_pose) > dense_->refinement_dist_threshold_) {
                if (it.second->get_state() == PointCloudEntry::GLOBAL_MAP_SWAP) {
                    it.second->load_cloud();
                    it.second->set_state(PointCloudEntry::GLOBAL_MAP_RAM);
                }

                do_refinement(it.second);

                it.second->lock();
                it.second->set_current_pos(update_pose);
                it.second->set_update_pos(nullptr);
                it.second->unlock();
            }

            if (it.second->get_state() == PointCloudEntry::GLOBAL_MAP_RAM) {
                it.second->save_cloud();
                it.second->set_state(PointCloudEntry::GLOBAL_MAP_SWAP);
                ROS_INFO("Refinement seq = %u swapped", it.second->get_seq());
            }

            usleep(REFINEMENT_DELAY_US);
        }

        usleep(REFINEMENT_DELAY_US);
    }
}

void RefinementThread::do_refinement(PointCloudEntry::Ptr entry)
{}
