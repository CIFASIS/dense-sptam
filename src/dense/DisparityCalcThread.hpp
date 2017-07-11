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

void showDispImage(float *disp_data, int img_height, int img_width, const char *filename);

#endif /* __DISPARITYCALCTHREAD_H */
