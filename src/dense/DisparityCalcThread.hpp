#ifndef __DISPARITYCALCTHREAD_H
#define __DISPARITYCALCTHREAD_H

#include <thread>

#include "DispImageQueue.hpp"
#include "ImageQueue.hpp"

class DisparityCalcThread
{
public:

    DisparityCalcThread(ImageQueue *raw_left_images, ImageQueue *raw_right_images, DispImageQueue *disp_images);

    inline void WaitUntilFinished()
    { disparityCalcThread_.join(); }

private:
    
    ImageQueue *raw_left_images_, *raw_right_images_;
    DispImageQueue *disp_images_;
    std::thread disparityCalcThread_;
    void compute();
};

#endif /* __DISPARITYCALCTHREAD_H */
