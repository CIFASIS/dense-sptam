#ifndef __REFINEMENTTHREAD_H
#define __REFINEMENTTHREAD_H

#include <thread>

#include "PointCloudQueue.hpp"

#define REFINEMENT_DELAY_US        10000

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
    void do_refinement(PointCloudEntry::Ptr entry);

};

#endif /* __REFINEMENTTHREAD_H */
