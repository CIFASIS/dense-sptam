#ifndef __IMAGE_QUEUE_H
#define __IMAGE_QUEUE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <mutex>

class ImageQueue
{
public:

    ImageQueue();
    ~ImageQueue();

    void push(sensor_msgs::ImagePtr image);
    sensor_msgs::ImagePtr at(unsigned int pos);
    sensor_msgs::ImagePtr back();
    size_t size();

private:

    std::mutex image_queue_lock_;
    std::vector<sensor_msgs::ImagePtr> images_;

};

#endif /* __IMAGE_QUEUE_H */
