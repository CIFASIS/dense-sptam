/*
 * This file is part of DENSE S-PTAM.
 *
 * Copyright (C) 2017 Ariel D'Alessandro
 * For more information see <https://github.com/adalessandro/dense-sptam>
 *
 * DENSE S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DENSE S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DENSE S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors: Ariel D'Alessandro <ariel@vanguardiasur.com.ar>
 *
 * Department of Computer Science
 * Faculty of Exact Sciences, Engineering and Surveying
 * University of Rosario - Argentina
 */

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
