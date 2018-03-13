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

#ifndef FRUSTUM_CULLING_H
#define FRUSTUM_CULLING_H

#include <eigen3/Eigen/Geometry>

class FrustumCulling
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FrustumCulling(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
				   double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist);

	bool Contains(const Eigen::Vector3d& point);

private:

	Eigen::Vector4d nearPlane_, farPlane_, leftPlane_, rightPlane_, topPlane_, bottomPlane_;

};

#endif /* FRUSTUM_CULLING_H */
