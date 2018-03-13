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

#ifndef __IMAGE_QUEUE_H
#define __IMAGE_QUEUE_H

#include <sensor_msgs/Image.h>
#include <condition_variable>
#include <mutex>
#include <queue>

typedef sensor_msgs::Image Image;
typedef boost::shared_ptr<Image> ImagePtr;

typedef std::pair<ImagePtr, ImagePtr> ImagePair;
typedef std::shared_ptr<ImagePair> ImagePairPtr;

class ImageQueue
{

public:

	ImageQueue();
	~ImageQueue();

	int push(ImagePairPtr image);
	ImagePairPtr pop();

private:

	std::mutex image_queue_lock_;
	std::condition_variable empty_queue_cv;
	ImagePairPtr image_;

};

#endif /* __IMAGE_QUEUE_H */
