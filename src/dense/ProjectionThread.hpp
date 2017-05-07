#ifndef __PROJECTIONTHREAD_H
#define __PROJECTIONTHREAD_H

#include <thread>
#include <pcl_ros/point_cloud.h>

#include "Camera.hpp"
#include "DispImageQueue.hpp"
#include "PointCloudQueue.hpp"
#include "FrustumCulling.hpp"

#define MY_MISSING_Z            10000.0
#define PIXEL_DISP_INVALID      -10
#define PIXEL_DISP_CORNER       -11

class Dense;

class ProjectionThread
{
public:

    ProjectionThread(Dense *dense);

    inline void WaitUntilFinished()
    { projectionThread_.join(); }

private:

    struct projection_log {
        unsigned int match, unmatch, outlier;
        double time_t[3];
    };

    Dense *dense_;
    std::thread projectionThread_;
    void compute();

    void filterDisp(const DispRawImagePtr disp_raw_img);
    bool isValidDisparity(const float disp);

    PointCloudPtr generateCloud(DispRawImagePtr disp_raw_img);

    void cameraToWorld(PointCloudPtr cloud, CameraPose::Ptr current_pos);

    PointCloudPtr doStereoscan(PointCloudPtr last_cloud, DispImagePtr disp_img,
                               FrustumCulling *frustum_left, FrustumCulling *frustum_right,
                               CameraPose::Ptr current_pos, double stereoscan_threshold,
                               struct projection_log *log_data);
};

void downsampleCloud(PointCloudPtr cloud, double voxelLeafSize);

#endif /* __PROJECTIONTHREAD_H */
