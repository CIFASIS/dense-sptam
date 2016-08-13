#ifndef __PROJECTIONTHREAD_H
#define __PROJECTIONTHREAD_H

#include <ros/ros.h>
#include <thread>
#include <pcl_ros/point_cloud.h>

#include "DispImageQueue.hpp"
#include "Camera.hpp"

class ProjectionThread
{
public:

    ProjectionThread(DispImageQueue *disp_images, Camera *camera);

    inline void WaitUntilFinished()
    { projectionThread_.join(); }

private:

    DispImageQueue *disp_images_;
    Camera *camera_;

    std::thread projectionThread_;
    void compute();

    bool isValidPoint(const cv::Vec3f& pt);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processPoints(const DispRawImagePtr disp_raw_img);
};

#endif /* __PROJECTIONTHREAD_H */
