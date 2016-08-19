#include "DisparityCalcThread.hpp"

#include <boost/smart_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/common.h>

#include "../libelas/src/elas.h"
#include "../libelas/src/image.h"

DisparityCalcThread::DisparityCalcThread(
        const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info,
        ImageQueue *raw_left_images, ImageQueue *raw_right_images, DispImageQueue *disp_images,
        std::string disp_calc_method
) : left_info_(left_info)
  , right_info_(right_info)
  , raw_left_images_(raw_left_images)
  , raw_right_images_(raw_right_images)
  , disp_images_(disp_images)
  , disp_calc_method_(disp_calc_method)
  , disparityCalcThread_(&DisparityCalcThread::compute, this)
{}

void DisparityCalcThread::compute()
{
    ROS_INFO("Selected disparity method: %s", disp_calc_method_.c_str());

    if (disp_calc_method_ == DISP_METHOD_OPENCV)
        computeCV();
    else if (disp_calc_method_ == DISP_METHOD_LIBELAS)
        computeELAS();
}

void DisparityCalcThread::computeCV()
{
    cv::StereoBM stereo(cv::StereoBM::BASIC_PRESET);
    ImagePtr raw_left_image, raw_right_image;
    cv::Mat image_left, image_right, dmat;
    DispRawImagePtr disp_raw_img;
    DispImagePtr disp_img;

    while(1) {
        /* Calls to pop() are blocking */
        raw_left_image = raw_left_images_->pop();
        raw_right_image = raw_right_images_->pop();

        image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;

        stereo(image_left, image_right, dmat, CV_32F);

        disp_img = boost::make_shared<DispImage>(dmat);
        disp_raw_img = boost::make_shared<DispRawImage>(raw_left_image, disp_img);
        disp_images_->push(disp_raw_img);
    }
}

void DisparityCalcThread::computeELAS()
{
    ImagePtr raw_left_image, raw_right_image;
    cv::Mat image_left, image_right;
    DispRawImagePtr disp_raw_img;
    DispImagePtr disp_img;
    int32_t dims[3];

    float *D1_data = (float*)malloc(left_info_->width * left_info_->height * sizeof(float));
    float *D2_data = (float*)malloc(right_info_->width * right_info_->height * sizeof(float));
    cv::Mat_<float> *dmat = new cv::Mat_<float>(left_info_->height, left_info_->width, D1_data);

    Elas::parameters param(Elas::ROBOTICS);
    Elas *elas = new Elas(param);
    dims[2] = dims[0] = left_info_->width;
    dims[1] = left_info_->height;

    while(1) {
        /* Calls to pop() are blocking */
        raw_left_image = raw_left_images_->pop();
        raw_right_image = raw_right_images_->pop();

        image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;

        elas->process(image_left.data, image_right.data, D1_data, D2_data, dims);

        disp_img = boost::make_shared<DispImage>(dmat);
        disp_raw_img = boost::make_shared<DispRawImage>(raw_left_image, disp_img);
        disp_images_->push(disp_raw_img);
    }
}
