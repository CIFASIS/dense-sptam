#include "ImageQueue.hpp"

ImageQueue::ImageQueue()
{}

ImageQueue::~ImageQueue()
{}

int ImageQueue::push(ImagePairPtr imagepair)
{
	int ret = 0;
	std::lock_guard<std::mutex> lock(image_queue_lock_);

	if (image_ != nullptr)
		ret = -1;

	image_ = imagepair;
	empty_queue_cv.notify_all();

	return ret;
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
