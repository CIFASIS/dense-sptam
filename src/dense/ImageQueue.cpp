#include "ImageQueue.hpp"

ImageQueue::ImageQueue()
{}

ImageQueue::~ImageQueue()
{}

void ImageQueue::push(sensor_msgs::ImagePtr image)
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    images_.push_back(image);
}

sensor_msgs::ImagePtr ImageQueue::at(unsigned int pos)
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    return images_.at(pos);
}

sensor_msgs::ImagePtr ImageQueue::back()
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    return images_.back();
}

size_t ImageQueue::size()
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    return images_.size();
}
