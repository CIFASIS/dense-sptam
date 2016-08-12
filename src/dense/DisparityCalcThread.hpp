#ifndef __DISPARITYCALCTHREAD_H
#define __DISPARITYCALCTHREAD_H

#include <ros/ros.h>
#include <thread>

class DisparityCalcThread
{
public:

    DisparityCalcThread();

    inline void WaitUntilFinished()
    { disparityCalcThread_.join(); }

private:
    
    std::thread disparityCalcThread_;
    void compute();
};

#endif /* __DISPARITYCALCTHREAD_H */
