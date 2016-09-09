#include "DispImageQueue.hpp"

DispImageQueue::DispImageQueue()
{}

DispImageQueue::~DispImageQueue()
{}

void DispImageQueue::push(DispRawImagePtr image)
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    image_ = image;
    empty_queue_cv.notify_all();
}

DispRawImagePtr DispImageQueue::pop()
{
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    image_queue_lock_.lock();

    while (image_ == nullptr) {
        image_queue_lock_.unlock();
        empty_queue_cv.wait(lock);
        image_queue_lock_.lock();
    }

    DispRawImagePtr ret = image_;
    image_ = nullptr;
    image_queue_lock_.unlock();

    return ret;
}
