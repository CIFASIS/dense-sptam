#ifndef __REFINEMENTTHREAD_H
#define __REFINEMENTTHREAD_H

#include <thread>

#include "PointCloudQueue.hpp"

class Dense;

class RefinementThread
{
public:

    RefinementThread(Dense *dense);

    inline void WaitUntilFinished()
    { refinementThread_.join(); }

private:

    Dense *dense_;

    std::thread refinementThread_;
    void compute();

};

#endif /* __REFINEMENTTHREAD_H */
