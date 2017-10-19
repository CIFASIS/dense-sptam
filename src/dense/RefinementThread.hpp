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

#ifndef __REFINEMENTTHREAD_H
#define __REFINEMENTTHREAD_H

#include <thread>

#include "PointCloudQueue.hpp"

#define REFINEMENT_DELAY_US				10000

class Dense;

class RefinementThread
{

public:

	RefinementThread(Dense *dense);

	inline void WaitUntilFinished()
	{ refinementThread_.join(); }

private:

	Dense *dense_;

	std::thread refinementThread_;
	void compute();
	PointCloudPtr refine_cloud(PointCloudPtr cloud, CameraPose::Ptr current_pose, CameraPose::Ptr update_pose);

};

#endif /* __REFINEMENTTHREAD_H */
