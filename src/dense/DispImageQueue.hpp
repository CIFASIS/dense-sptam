#ifndef __DISP_IMAGE_QUEUE_H
#define __DISP_IMAGE_QUEUE_H

#include <condition_variable>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>

#include "ImageQueue.hpp"

typedef cv::Mat_<float> DispImage;
typedef boost::shared_ptr<DispImage> DispImagePtr;

typedef std::pair<ImagePtr, DispImagePtr> DispRawImage;
typedef boost::shared_ptr<DispRawImage> DispRawImagePtr;

class DispImageQueue
{
public:

    DispImageQueue();
    ~DispImageQueue();

    void push(DispRawImagePtr image);
    DispRawImagePtr pop(bool remove = true);
    size_t size();

private:

    std::mutex image_queue_lock_;
    std::condition_variable empty_queue_cv;
    std::queue<DispRawImagePtr> images_;

};

#endif /* __DISP_IMAGE_QUEUE_H */
