#ifndef __PROJECTIONTHREAD_H
#define __PROJECTIONTHREAD_H

#include <thread>
#include <pcl_ros/point_cloud.h>

#include "Camera.hpp"
#include "DispImageQueue.hpp"
#include "PointCloudQueue.hpp"

#define MIN_DISPARITY_THRESHOLD 20
#define MY_MISSING_Z    10000.0

class Dense;

class ProjectionThread
{
public:

    ProjectionThread(Dense *dense);

    inline void WaitUntilFinished()
    { projectionThread_.join(); }

private:

    Dense *dense_;
    std::thread projectionThread_;
    void compute();

    void filterDisp(const DispRawImagePtr disp_raw_img, float min_disparity);
    bool isValidPoint(const cv::Vec3f& pt);
    PointCloudPtr generateCloud(const DispRawImagePtr disp_raw_img);
    void cameraToWorld(PointCloudPtr cloud, CameraPose::Ptr current_pos);
    void downsampleCloud(PointCloudPtr cloud, double voxelLeafSize);
    void filterCloud(PointCloudPtr cloud, double filter_meanK, double filter_stddev);
};

#endif /* __PROJECTIONTHREAD_H */
