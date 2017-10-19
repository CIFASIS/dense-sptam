/*
 * This file is part of DENSE S-PTAM.
 *
 * Copyright (C) 2017 Ariel D'Alessandro
 * For more information see <https://github.com/adalessandro/dense-sptam>
 *
 * DENSE S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DENSE S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DENSE S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors: Ariel D'Alessandro <ariel@vanguardiasur.com.ar>
 *
 * Department of Computer Science
 * Faculty of Exact Sciences, Engineering and Surveying
 * University of Rosario - Argentina
 */

#include "RefinementThread.hpp"
#include "dense.hpp"
#include "../utils/Time.hpp"

RefinementThread::RefinementThread(Dense *dense)
  : dense_(dense)
  , refinementThread_(&RefinementThread::compute, this)
{}

void RefinementThread::compute()
{
	double start_t, end_t;

	while(1) {

		for (auto& it : dense_->point_clouds_->entries_) {

			if (!it.second)
				continue;

			it.second->lock();

			start_t = GetSeg();

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

				if (linear_dist > dense_->parameters.refinement_linear_threshold ||
					angular_dist > dense_->parameters.refinement_angular_threshold) {

					if (it.second->get_state() == PointCloudEntry::GLOBAL_MAP_SWAP) {
						it.second->load_cloud(dense_->parameters.output_dir.c_str());
						ROS_INFO("Refinement seq = %u cloud to RAM", it.second->get_seq());
						it.second->set_state(PointCloudEntry::GLOBAL_MAP_RAM);
					}

					PointCloudPtr cloud = refine_cloud(it.second->get_cloud(), current_pose, update_pose);
					it.second->set_cloud(cloud);

					end_t = GetSeg();
					dense_->WriteToLog("refinement,%u,%f\n", it.second->get_seq(), end_t - start_t);
					ROS_INFO("Refinement seq = %u cloud REFINED (distance = %f, quaternion = %f)",
							 it.second->get_seq(), current_pose->distance(*update_pose),
							 current_pose->get_orientation().angularDistance(update_pose->get_orientation()));
				}
			}

			if (it.second->get_state() == PointCloudEntry::GLOBAL_MAP_RAM) {
				it.second->save_cloud(dense_->parameters.output_dir.c_str(), dense_->point_clouds_->poses_);
				it.second->set_state(PointCloudEntry::GLOBAL_MAP_SWAP);
				ROS_INFO("Refinement seq = %u cloud SWAPPED", it.second->get_seq());
			}

			/* Let's introduce some delay so we don't burn out the CPU */
			usleep(REFINEMENT_DELAY_US);

		}

		usleep(REFINEMENT_DELAY_US);
	}
}

PointCloudPtr RefinementThread::refine_cloud(PointCloudPtr cloud, CameraPose::Ptr current_pose,
											 CameraPose::Ptr update_pose)
{
	PointCloudPtr new_cloud(new PointCloud);
	CameraPose::Position pos;

	for (auto& it: *cloud) {
		pos(0) = it.x;
		pos(1) = it.y;
		pos(2) = it.z;
		pos = current_pose->ToCamera(pos);
		pos = update_pose->ToWorld(pos);
		it.x = pos(0);
		it.y = pos(1);
		it.z = pos(2);
		new_cloud->push_back(it);
	}

	return new_cloud;
}
