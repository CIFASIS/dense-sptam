#ifndef __PROJECTIONTHREAD_H
#define __PROJECTIONTHREAD_H

#include <ros/ros.h>
#include <thread>

#include "DispImageQueue.hpp"

class ProjectionThread
{
public:

    ProjectionThread(DispImageQueue *disp_images);

    inline void WaitUntilFinished()
    { projectionThread_.join(); }

private:

    DispImageQueue *disp_images_;

    std::thread projectionThread_;
    void compute();
};

#endif /* __PROJECTIONTHREAD_H */
