#ifndef __DISPARITYCALCTHREAD_H
#define __DISPARITYCALCTHREAD_H

#include <thread>
#include <sensor_msgs/CameraInfo.h>

#include "DispImageQueue.hpp"
#include "ImageQueue.hpp"

#define DISP_METHOD_OPENCV  "opencv"
#define DISP_METHOD_LIBELAS "libelas"

class Dense;

class DisparityCalcThread
{
public:

    DisparityCalcThread(Dense *dense);

    inline void WaitUntilFinished()
    { disparityCalcThread_.join(); }

private:

    Dense *dense_;
    std::thread disparityCalcThread_;

    void compute();
    void computeCV();
    void computeELAS();
};

#endif /* __DISPARITYCALCTHREAD_H */
