#ifndef __TRANSFORMTHREAD_H
#define __TRANSFORMTHREAD_H

#include <thread>

#include "PointCloudQueue.hpp"

class TransformThread
{
public:

    TransformThread(PointCloudQueue *point_clouds);

    inline void WaitUntilFinished()
    { transformThread_.join(); }

private:

    PointCloudQueue *point_clouds_;

    std::thread transformThread_;
    void compute();
    void cameraToWorld(PointCloudEntry::Ptr entry);

};

#endif /* __TRANSFORMTHREAD_H */
