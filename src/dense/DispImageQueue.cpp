#include "DispImageQueue.hpp"

DispImageQueue::DispImageQueue()
{}

DispImageQueue::~DispImageQueue()
{}

void DispImageQueue::push(DispRawImagePtr image)
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    images_.push(image);
    empty_queue_cv.notify_all();
}

DispRawImagePtr DispImageQueue::pop(bool remove)
{
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    image_queue_lock_.lock();

    while (images_.empty()) {
        image_queue_lock_.unlock();
        empty_queue_cv.wait(lock);
        image_queue_lock_.lock();
    }

    DispRawImagePtr ret = images_.front();

    if (remove)
        images_.pop();

    image_queue_lock_.unlock();

    return ret;
}

size_t DispImageQueue::size()
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    return images_.size();
}
