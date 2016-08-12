#include "DisparityCalcThread.hpp"

DisparityCalcThread::DisparityCalcThread()
  : disparityCalcThread_(&DisparityCalcThread::compute, this)
{}

void DisparityCalcThread::compute()
{
    ROS_INFO("DisparityCalcThread compute!");
}
