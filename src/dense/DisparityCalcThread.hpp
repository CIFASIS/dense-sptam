#ifndef __DISPARITYCALCTHREAD_H
#define __DISPARITYCALCTHREAD_H

#include <thread>
#include <sensor_msgs/CameraInfo.h>

#include "DispImageQueue.hpp"
#include "ImageQueue.hpp"

#define DISP_METHOD_OPENCV  "opencv"
#define DISP_METHOD_LIBELAS "libelas"

class DisparityCalcThread
{
public:

    DisparityCalcThread(
            const sensor_msgs::CameraInfoConstPtr &left_info, const sensor_msgs::CameraInfoConstPtr &right_info,
            ImageQueue *raw_left_images, ImageQueue *raw_right_images, DispImageQueue *disp_images,
            std::string disp_calc_method
    );

    inline void WaitUntilFinished()
    { disparityCalcThread_.join(); }

private:
    
    const sensor_msgs::CameraInfoConstPtr left_info_, right_info_;
    ImageQueue *raw_left_images_, *raw_right_images_;
    DispImageQueue *disp_images_;
    std::string disp_calc_method_;
    std::thread disparityCalcThread_;

    void compute();
    void computeCV();
    void computeELAS();
};

#endif /* __DISPARITYCALCTHREAD_H */
