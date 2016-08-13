#include "ProjectionThread.hpp"

ProjectionThread::ProjectionThread(
    DispImageQueue *disp_images
) : disp_images_(disp_images)
  , projectionThread_(&ProjectionThread::compute, this)
{}

void ProjectionThread::compute()
{
    while(1) {
        /* Calls to pop() are blocking */
        DispImagePtr disp_img = disp_images_->pop();
    }
}
