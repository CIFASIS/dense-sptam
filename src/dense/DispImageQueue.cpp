#include "DispImageQueue.hpp"

DispImageQueue::DispImageQueue()
{}

DispImageQueue::~DispImageQueue()
{}

void DispImageQueue::push(DispImagePtr image)
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    images_.push_back(image);
}

DispImagePtr DispImageQueue::at(unsigned int pos)
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    return images_.at(pos);
}

DispImagePtr DispImageQueue::back()
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    return images_.back();
}

size_t DispImageQueue::size()
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    return images_.size();
}
