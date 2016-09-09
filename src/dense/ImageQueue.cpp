#include "ImageQueue.hpp"

ImageQueue::ImageQueue()
{}

ImageQueue::~ImageQueue()
{}

void ImageQueue::push(ImagePairPtr imagepair)
{
    std::lock_guard<std::mutex> lock(image_queue_lock_);
    image_ = imagepair;
    empty_queue_cv.notify_all();
}

ImagePairPtr ImageQueue::pop()
{
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    image_queue_lock_.lock();

    while (image_ == nullptr) {
        image_queue_lock_.unlock();
        empty_queue_cv.wait(lock);
        image_queue_lock_.lock();
    }

    ImagePairPtr ret = image_;
    image_ = nullptr;
    image_queue_lock_.unlock();

    return ret;
}
