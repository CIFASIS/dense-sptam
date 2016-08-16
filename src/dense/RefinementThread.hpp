#ifndef __REFINEMENTTHREAD_H
#define __REFINEMENTTHREAD_H

#include <thread>

#include "PointCloudQueue.hpp"

class RefinementThread
{
public:

    RefinementThread(PointCloudQueue *point_clouds);

    inline void WaitUntilFinished()
    { refinementThread_.join(); }

private:

    PointCloudQueue *point_clouds_;

    std::thread refinementThread_;
    void compute();

};

#endif /* __REFINEMENTTHREAD_H */
