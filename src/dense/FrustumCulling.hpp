#ifndef FRUSTUM_CULLING_H
#define FRUSTUM_CULLING_H

#include <eigen3/Eigen/Geometry>

class FrustumCulling
{

public:

	FrustumCulling(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
				   double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist);

	bool Contains(const Eigen::Vector3d& point);

private:

	Eigen::Vector4d nearPlane_, farPlane_, leftPlane_, rightPlane_, topPlane_, bottomPlane_;

};

#endif /* FRUSTUM_CULLING_H */
