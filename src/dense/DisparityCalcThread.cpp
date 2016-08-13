#include "DisparityCalcThread.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/smart_ptr.hpp>

DisparityCalcThread::DisparityCalcThread(
        ImageQueue *raw_left_images, ImageQueue *raw_right_images, DispImageQueue *disp_images
) : raw_left_images_(raw_left_images)
  , raw_right_images_(raw_right_images)
  , disp_images_(disp_images)
  , disparityCalcThread_(&DisparityCalcThread::compute, this)
{}

void DisparityCalcThread::compute()
{
    while(1) {
        /* Calls to pop() are blocking */
        ImagePtr raw_left_image = raw_left_images_->pop();
        ImagePtr raw_right_image = raw_right_images_->pop();

        cv::Mat image_left(cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::TYPE_8UC1)->image);
        cv::Mat image_right(cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::TYPE_8UC1)->image);

        cv::StereoBM stereo(cv::StereoBM::BASIC_PRESET);
        cv::Mat dmat;

        stereo(image_left, image_right, dmat, CV_32F);
        DispImagePtr disp_img = boost::make_shared<DispImage>(dmat);
        DispRawImagePtr disp_raw_img = boost::make_shared<DispRawImage>(disp_img, raw_left_image);
        disp_images_->push(disp_raw_img);
    }

}
