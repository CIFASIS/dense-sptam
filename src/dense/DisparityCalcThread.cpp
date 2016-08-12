#include "DisparityCalcThread.hpp"

DisparityCalcThread::DisparityCalcThread(ImageQueue *raw_left_images, ImageQueue *raw_right_images)
  : raw_left_images_(raw_left_images)
  , raw_right_images_(raw_right_images)
  , disparityCalcThread_(&DisparityCalcThread::compute, this)
{}

void DisparityCalcThread::compute()
{
    ROS_INFO("DisparityCalcThread compute!");
}
