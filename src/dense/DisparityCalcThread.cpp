#include "DisparityCalcThread.hpp"

#include <boost/smart_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/common.h>

#include "../libelas/src/elas.h"
#include "../libelas/src/image.h"
#include "dense.hpp"

DisparityCalcThread::DisparityCalcThread(Dense *dense)
  : dense_(dense)
  , disparityCalcThread_(&DisparityCalcThread::compute, this)
{}

void DisparityCalcThread::compute()
{
    ROS_INFO("Selected disparity method: %s", dense_->disp_calc_method_.c_str());

    if (dense_->disp_calc_method_ == DISP_METHOD_OPENCV)
        computeCV();
    else if (dense_->disp_calc_method_ == DISP_METHOD_LIBELAS)
        computeELAS();
}

void DisparityCalcThread::computeCV()
{
    cv::StereoBM stereo(cv::StereoBM::BASIC_PRESET);
    ImagePairPtr raw_image_pair;
    ImagePtr raw_left_image, raw_right_image;
    cv::Mat image_left, image_right, dmat;
    DispRawImagePtr disp_raw_img;
    DispImagePtr disp_img;

    while(1) {
        /* Calls to pop() are blocking */
        raw_image_pair = dense_->raw_image_pairs_->pop();
        raw_left_image = raw_image_pair->first;
        raw_right_image = raw_image_pair->second;

        image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;

        stereo(image_left, image_right, dmat, CV_32F);

        disp_img = boost::make_shared<DispImage>(dmat);
        disp_raw_img = boost::make_shared<DispRawImage>(raw_left_image, disp_img);
        dense_->disp_images_->push(disp_raw_img);
    }
}

void DisparityCalcThread::computeELAS()
{
    ImagePairPtr raw_image_pair;
    ImagePtr raw_left_image, raw_right_image;
    cv::Mat image_left, image_right;
    DispRawImagePtr disp_raw_img;
    DispImagePtr disp_img;
    int32_t dims[3];

    float *D1_data = (float*)malloc(dense_->left_info_->width * dense_->left_info_->height * sizeof(float));
    float *D2_data = (float*)malloc(dense_->right_info_->width * dense_->right_info_->height * sizeof(float));
    cv::Mat_<float> *dmat = new cv::Mat_<float>(dense_->left_info_->height, dense_->left_info_->width, D1_data);

    Elas::parameters param(Elas::ROBOTICS);
    Elas *elas = new Elas(param);
    dims[2] = dims[0] = dense_->left_info_->width;
    dims[1] = dense_->left_info_->height;

    while(1) {
        /* Calls to pop() are blocking */
        raw_image_pair = dense_->raw_image_pairs_->pop();
        raw_left_image = raw_image_pair->first;
        raw_right_image = raw_image_pair->second;

        image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;
        image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::TYPE_8UC1)->image;

        elas->process(image_left.data, image_right.data, D1_data, D2_data, dims);

        disp_img = boost::make_shared<DispImage>(dmat);
        disp_raw_img = boost::make_shared<DispRawImage>(raw_left_image, disp_img);
        dense_->disp_images_->push(disp_raw_img);

        ROS_INFO("DisparityCalcThread::computed seq = %u", raw_left_image->header.seq);
    }
}
