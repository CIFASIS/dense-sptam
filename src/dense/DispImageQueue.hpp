#ifndef __DISP_IMAGE_QUEUE_H
#define __DISP_IMAGE_QUEUE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <opencv2/opencv.hpp>

typedef cv::Mat DispImage;
typedef boost::shared_ptr<DispImage> DispImagePtr;

class DispImageQueue
{
public:

    DispImageQueue();
    ~DispImageQueue();

    void push(DispImagePtr image);
    DispImagePtr pop(bool remove = true);
    size_t size();

private:

    std::mutex image_queue_lock_;
    std::condition_variable empty_queue_cv;
    std::queue<DispImagePtr> images_;

};

#endif /* __DISP_IMAGE_QUEUE_H */
