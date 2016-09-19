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
            /* Projection thread is in charge of setting the first pose */
            if (!current_pose) {
                it.second->unlock();
                continue;
            }

            /* Clouds in local area aren't refined nor swapped */
            if (it.second->get_state() == PointCloudEntry::LOCAL_MAP) {
                it.second->unlock();
                continue;
            }

            CameraPose::Ptr update_pose = it.second->get_update_pos();
            it.second->set_current_pos(update_pose);
            it.second->set_update_pos(nullptr);

            it.second->unlock();

            if (update_pose) {
                float linear_dist = current_pose->distance(*update_pose);
                float angular_dist = current_pose->get_orientation().angularDistance(update_pose->get_orientation());

                if (linear_dist > dense_->refinement_linear_threshold_ ||
                    angular_dist > dense_->refinement_angular_threshold_) {
                    if (it.second->get_state() == PointCloudEntry::GLOBAL_MAP_SWAP) {
                        it.second->load_cloud();
                        ROS_INFO("Refinement seq = %u cloud to RAM", it.second->get_seq());
                        it.second->set_state(PointCloudEntry::GLOBAL_MAP_RAM);
                    }

                    do_refinement(it.second);

                    ROS_INFO("Refinement seq = %u cloud REFINED (distance = %f, quaternion = %f)",
                             it.second->get_seq(), current_pose->distance(*update_pose),
                             current_pose->get_orientation().angularDistance(update_pose->get_orientation()));
                }
            }

            if (it.second->get_state() == PointCloudEntry::GLOBAL_MAP_RAM) {
                it.second->save_cloud();
                it.second->set_state(PointCloudEntry::GLOBAL_MAP_SWAP);
                ROS_INFO("Refinement seq = %u cloud SWAPPED", it.second->get_seq());
            }

            /* Let's introduce some delay so we don't burn out the CPU */
            usleep(REFINEMENT_DELAY_US);
        }

        usleep(REFINEMENT_DELAY_US);
    }
}

void RefinementThread::do_refinement(PointCloudEntry::Ptr entry)
{}
