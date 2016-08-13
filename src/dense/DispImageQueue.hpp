#ifndef __DISP_IMAGE_QUEUE_H
#define __DISP_IMAGE_QUEUE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <opencv2/opencv.hpp>

#include "ImageQueue.hpp"

typedef cv::Mat DispImage;
typedef boost::shared_ptr<DispImage> DispImagePtr;

typedef std::pair<DispImagePtr, ImagePtr> DispRawImage;
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
