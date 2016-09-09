#ifndef __TRANSFORMTHREAD_H
#define __TRANSFORMTHREAD_H

#include <thread>

#include "PointCloudQueue.hpp"

class TransformThread
{
public:

    TransformThread(PointCloudQueue *point_clouds, Camera *camera);

    inline void WaitUntilFinished()
    { transformThread_.join(); }

private:

    PointCloudQueue *point_clouds_;
    Camera::Ptr camera_;

    std::thread transformThread_;
    void compute();

    bool isValidPoint(const cv::Vec3f& pt);
    PointCloudPtr generateCloud(PointCloudEntry::Ptr entry);
    void cameraToWorld(PointCloudPtr cloud, CameraPose::Ptr current_pos);

};

#endif /* __TRANSFORMTHREAD_H */
