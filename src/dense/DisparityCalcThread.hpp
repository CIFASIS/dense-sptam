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

#ifndef __DISPARITYCALCTHREAD_H
#define __DISPARITYCALCTHREAD_H

#include <thread>
#include <sensor_msgs/CameraInfo.h>
#include <eigen3/Eigen/Geometry>


#include "DispImageQueue.hpp"
#include "ImageQueue.hpp"

#define DISP_METHOD_OPENCV			"opencv"
#define DISP_METHOD_LIBELAS			"libelas"

class Dense;

class DisparityCalcThread
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	DisparityCalcThread(Dense *dense);
	~DisparityCalcThread();

	inline void WaitUntilFinished()
	{ disparityCalcThread_->join(); }

private:

	Dense *dense_;
	std::thread *disparityCalcThread_;

	void compute();
	void computeCV();
	void computeELAS();

};

void showDispImage(float *disp_data, int img_height, int img_width, const char *filename);

#endif /* __DISPARITYCALCTHREAD_H */
