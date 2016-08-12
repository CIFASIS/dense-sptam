#ifndef __DISP_IMAGE_QUEUE_H
#define __DIPS_IMAGE_QUEUE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <mutex>
#include <opencv2/opencv.hpp>

typedef cv::Mat DispImage;
typedef boost::shared_ptr<DispImage> DispImagePtr;



class DispImageQueue
{
public:

    DispImageQueue();
    ~DispImageQueue();

    void push(DispImagePtr image);
    DispImagePtr at(unsigned int pos);
    DispImagePtr back();
    size_t size();

private:

    std::mutex image_queue_lock_;
    std::vector<DispImagePtr> images_;

};

#endif /* __DISP_IMAGE_QUEUE_H */
