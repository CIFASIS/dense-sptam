#ifndef FRUSTUM_CULLING_H
#define FRUSTUM_CULLING_H

#include <eigen3/Eigen/Geometry>
#include "Camera.hpp"

class FrustumCulling
{
public:

    FrustumCulling(
        const CameraPose& cameraPose, double horizontalFOV, double verticalFOV,
        double nearPlaneDist, double farPlaneDist
    );

    bool Contains(const Eigen::Vector3d& point);

private:

    CameraPose cameraPose_;

    Eigen::Vector4d nearPlane_;
    Eigen::Vector4d farPlane_;
    Eigen::Vector4d leftPlane_;
    Eigen::Vector4d rightPlane_;
    Eigen::Vector4d topPlane_;
    Eigen::Vector4d bottomPlane_;

    void ComputeFrustum(double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist);
};

#endif /* FRUSTUM_CULLING_H */
