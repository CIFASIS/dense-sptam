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
#include "../utils/Time.hpp"

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
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
    ImagePairPtr raw_image_pair;
    ImagePtr raw_left_image, raw_right_image;
    cv::Mat image_left, image_right, dmat;
    DispRawImagePtr disp_raw_img;
    DispImagePtr disp_img;
    char log_buffer[512];

    while(1) {
        /* Calls to pop() are blocking */
        raw_image_pair = dense_->raw_image_pairs_->pop();

        double start_t, end_t;
        start_t = GetSeg();

        raw_left_image = raw_image_pair->first;
        raw_right_image = raw_image_pair->second;

        image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::MONO8)->image;
        image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::MONO8)->image;

        stereo->compute(image_left, image_right, dmat);

        disp_img = boost::make_shared<DispImage>(dmat);
        disp_raw_img = boost::make_shared<DispRawImage>(raw_left_image, disp_img);

        if (dense_->disp_images_->push(disp_raw_img) < 0)
            ROS_INFO("##### WARNING: Keyframe %u omitted, projectionThread busy! #####", raw_left_image->header.seq);

        end_t = GetSeg();

        sprintf(log_buffer, "disparity,%u,%f\n", raw_left_image->header.seq, end_t - start_t);
        dense_->WriteToLog(log_buffer);
        ROS_INFO("Disparity  seq = %u (%f secs)", raw_left_image->header.seq, end_t - start_t);

#if 0 /* Save disparity images */
        char filename[256];
        sprintf(filename, "images/disparity_%u.jpg", raw_left_image->header.seq);
        showDispImage((float*)dmat.data, dense_->left_info_->height, dense_->left_info_->width, filename);
        sprintf(filename, "images/orig_%u.jpg", raw_left_image->header.seq);
        cv::imwrite(filename, image_left);
#endif
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
    char log_buffer[512];

    float *D1_data = (float*)malloc(dense_->left_info_->width * dense_->left_info_->height * sizeof(float));
    float *D2_data = (float*)malloc(dense_->right_info_->width * dense_->right_info_->height * sizeof(float));
    cv::Mat_<float> *dmat = new cv::Mat_<float>(dense_->left_info_->height, dense_->left_info_->width, D1_data);

    Elas::parameters param(Elas::ROBOTICS);

    if (dense_->sigma_)
        param.sigma = dense_->sigma_;
    param.add_corners = dense_->add_corners_;
    if (dense_->libelas_ipol_gap_)
        param.ipol_gap_width = dense_->libelas_ipol_gap_;

    Elas *elas = new Elas(param);
    dims[2] = dims[0] = dense_->left_info_->width;
    dims[1] = dense_->left_info_->height;

    while(1) {
        /* Calls to pop() are blocking */
        raw_image_pair = dense_->raw_image_pairs_->pop();

        double start_t, end_t;
        start_t = GetSeg();

        raw_left_image = raw_image_pair->first;
        raw_right_image = raw_image_pair->second;

        image_left = cv_bridge::toCvCopy(raw_left_image, sensor_msgs::image_encodings::MONO8)->image;
        image_right = cv_bridge::toCvCopy(raw_right_image, sensor_msgs::image_encodings::MONO8)->image;

        elas->process(image_left.data, image_right.data, D1_data, D2_data, dims);

        disp_img = boost::make_shared<DispImage>(dmat);
        disp_raw_img = boost::make_shared<DispRawImage>(raw_left_image, disp_img);
        if (dense_->disp_images_->push(disp_raw_img) < 0)
            ROS_INFO("##### WARNING: Keyframe %u omitted, projectionThread busy! #####", raw_left_image->header.seq);

        end_t = GetSeg();

        sprintf(log_buffer, "disparity,%u,%f\n", raw_left_image->header.seq, end_t - start_t);
        dense_->WriteToLog(log_buffer);
        ROS_INFO("Disparity  seq = %u (%f secs)", raw_left_image->header.seq, end_t - start_t);

#if 0 /* Save disparity images */
        char filename[256];
        sprintf(filename, "images/disparity_%u.jpg", raw_left_image->header.seq);
        showDispImage(D1_data, dense_->left_info_->height, dense_->left_info_->width, filename);
        sprintf(filename, "images/orig_%u.jpg", raw_left_image->header.seq);
        cv::imwrite(filename, image_left);
#endif
    }
}

static unsigned char colormap[768] = {
    150, 150, 150,
    107, 0, 12,
    106, 0, 18,
    105, 0, 24,
    103, 0, 30,
    102, 0, 36,
    101, 0, 42,
    99, 0, 48,
    98, 0, 54,
    97, 0, 60,
    96, 0, 66,
    94, 0, 72,
    93, 0, 78,
    92, 0, 84,
    91, 0, 90,
    89, 0, 96,
    88, 0, 102,
    87, 0, 108,
    85, 0, 114,
    84, 0, 120,
    83, 0, 126,
    82, 0, 131,
    80, 0, 137,
    79, 0, 143,
    78, 0, 149,
    77, 0, 155,
    75, 0, 161,
    74, 0, 167,
    73, 0, 173,
    71, 0, 179,
    70, 0, 185,
    69, 0, 191,
    68, 0, 197,
    66, 0, 203,
    65, 0, 209,
    64, 0, 215,
    62, 0, 221,
    61, 0, 227,
    60, 0, 233,
    59, 0, 239,
    57, 0, 245,
    56, 0, 251,
    55, 0, 255,
    54, 0, 255,
    52, 0, 255,
    51, 0, 255,
    50, 0, 255,
    48, 0, 255,
    47, 0, 255,
    46, 0, 255,
    45, 0, 255,
    43, 0, 255,
    42, 0, 255,
    41, 0, 255,
    40, 0, 255,
    38, 0, 255,
    37, 0, 255,
    36, 0, 255,
    34, 0, 255,
    33, 0, 255,
    32, 0, 255,
    31, 0, 255,
    29, 0, 255,
    28, 0, 255,
    27, 0, 255,
    26, 0, 255,
    24, 0, 255,
    23, 0, 255,
    22, 0, 255,
    20, 0, 255,
    19, 0, 255,
    18, 0, 255,
    17, 0, 255,
    15, 0, 255,
    14, 0, 255,
    13, 0, 255,
    11, 0, 255,
    10, 0, 255,
    9, 0, 255,
    8, 0, 255,
    6, 0, 255,
    5, 0, 255,
    4, 0, 255,
    3, 0, 255,
    1, 0, 255,
    0, 4, 255,
    0, 10, 255,
    0, 16, 255,
    0, 22, 255,
    0, 28, 255,
    0, 34, 255,
    0, 40, 255,
    0, 46, 255,
    0, 52, 255,
    0, 58, 255,
    0, 64, 255,
    0, 70, 255,
    0, 76, 255,
    0, 82, 255,
    0, 88, 255,
    0, 94, 255,
    0, 100, 255,
    0, 106, 255,
    0, 112, 255,
    0, 118, 255,
    0, 124, 255,
    0, 129, 255,
    0, 135, 255,
    0, 141, 255,
    0, 147, 255,
    0, 153, 255,
    0, 159, 255,
    0, 165, 255,
    0, 171, 255,
    0, 177, 255,
    0, 183, 255,
    0, 189, 255,
    0, 195, 255,
    0, 201, 255,
    0, 207, 255,
    0, 213, 255,
    0, 219, 255,
    0, 225, 255,
    0, 231, 255,
    0, 237, 255,
    0, 243, 255,
    0, 249, 255,
    0, 255, 255,
    0, 255, 249,
    0, 255, 243,
    0, 255, 237,
    0, 255, 231,
    0, 255, 225,
    0, 255, 219,
    0, 255, 213,
    0, 255, 207,
    0, 255, 201,
    0, 255, 195,
    0, 255, 189,
    0, 255, 183,
    0, 255, 177,
    0, 255, 171,
    0, 255, 165,
    0, 255, 159,
    0, 255, 153,
    0, 255, 147,
    0, 255, 141,
    0, 255, 135,
    0, 255, 129,
    0, 255, 124,
    0, 255, 118,
    0, 255, 112,
    0, 255, 106,
    0, 255, 100,
    0, 255, 94,
    0, 255, 88,
    0, 255, 82,
    0, 255, 76,
    0, 255, 70,
    0, 255, 64,
    0, 255, 58,
    0, 255, 52,
    0, 255, 46,
    0, 255, 40,
    0, 255, 34,
    0, 255, 28,
    0, 255, 22,
    0, 255, 16,
    0, 255, 10,
    0, 255, 4,
    2, 255, 0,
    8, 255, 0,
    14, 255, 0,
    20, 255, 0,
    26, 255, 0,
    32, 255, 0,
    38, 255, 0,
    44, 255, 0,
    50, 255, 0,
    56, 255, 0,
    62, 255, 0,
    68, 255, 0,
    74, 255, 0,
    80, 255, 0,
    86, 255, 0,
    92, 255, 0,
    98, 255, 0,
    104, 255, 0,
    110, 255, 0,
    116, 255, 0,
    122, 255, 0,
    128, 255, 0,
    133, 255, 0,
    139, 255, 0,
    145, 255, 0,
    151, 255, 0,
    157, 255, 0,
    163, 255, 0,
    169, 255, 0,
    175, 255, 0,
    181, 255, 0,
    187, 255, 0,
    193, 255, 0,
    199, 255, 0,
    205, 255, 0,
    211, 255, 0,
    217, 255, 0,
    223, 255, 0,
    229, 255, 0,
    235, 255, 0,
    241, 255, 0,
    247, 255, 0,
    253, 255, 0,
    255, 251, 0,
    255, 245, 0,
    255, 239, 0,
    255, 233, 0,
    255, 227, 0,
    255, 221, 0,
    255, 215, 0,
    255, 209, 0,
    255, 203, 0,
    255, 197, 0,
    255, 191, 0,
    255, 185, 0,
    255, 179, 0,
    255, 173, 0,
    255, 167, 0,
    255, 161, 0,
    255, 155, 0,
    255, 149, 0,
    255, 143, 0,
    255, 137, 0,
    255, 131, 0,
    255, 126, 0,
    255, 120, 0,
    255, 114, 0,
    255, 108, 0,
    255, 102, 0,
    255, 96, 0,
    255, 90, 0,
    255, 84, 0,
    255, 78, 0,
    255, 72, 0,
    255, 66, 0,
    255, 60, 0,
    255, 54, 0,
    255, 48, 0,
    255, 42, 0,
    255, 36, 0,
    255, 30, 0,
    255, 24, 0,
    255, 18, 0,
    255, 12, 0,
    255,  6, 0,
    255,  0, 0,
};

void showDispImage(float *disp_data, int img_height, int img_width, const char *filename)
{
    /* Colormap and display the disparity image */
    float min_disparity = 0;
    float max_disparity = 0;

    for (int i = 0; i < img_width * img_height; i++)
        if (disp_data[i] > max_disparity)
            max_disparity = disp_data[i];

    float multiplier = 255.0f / (max_disparity - min_disparity);

    const cv::Mat_<float> dmat(img_height, img_width, (float*)&disp_data[0]);

    cv::Mat_<cv::Vec3b> disparity_color_;
    disparity_color_.create(img_height, img_width);

    for (int row = 0; row < disparity_color_.rows; ++row) {
        const float* d = dmat[row];
        cv::Vec3b *disparity_color = disparity_color_[row],
        *disparity_color_end = disparity_color + disparity_color_.cols;

        for (; disparity_color < disparity_color_end; ++disparity_color, ++d) {
            int index = (*d - min_disparity) * multiplier + 0.5;
            index = std::min(255, std::max(0, index));
            /* Fill as BGR */
            (*disparity_color)[2] = colormap[3*index + 0];
            (*disparity_color)[1] = colormap[3*index + 1];
            (*disparity_color)[0] = colormap[3*index + 2];
        }
    }

    /* Show/save image */
    if (filename) {
        cv::imwrite(filename, disparity_color_);
    } else {
        cv::imshow("disparity_image", disparity_color_);
        cv::waitKey(0);
    }
}

void showDepthImage(float *disp_data, int img_height, int img_width, const char *filename)
{
    /* Colormap and display the depth image */
    float min_disparity = 0;
    float max_disparity = 0;

    for (int i = 0; i < img_width * img_height; i++)
        if (disp_data[i] > max_disparity)
            max_disparity = disp_data[i];

    float multiplier = 255.0f / (max_disparity - min_disparity);

    const cv::Mat_<float> dmat(img_height, img_width, (float*)&disp_data[0]);

    cv::Mat_<cv::Vec3b> disparity_color_;
    disparity_color_.create(img_height, img_width);

    for (int row = 0; row < disparity_color_.rows; ++row) {
        const float* d = dmat[row];
        cv::Vec3b *disparity_color = disparity_color_[row],
        *disparity_color_end = disparity_color + disparity_color_.cols;

        for (; disparity_color < disparity_color_end; ++disparity_color, ++d) {
            int index = (*d - min_disparity) * multiplier + 0.5;
            index = std::min(255, std::max(0, index));
            /* Fill as BGR */
            (*disparity_color)[2] = colormap[3*index + 0];
            (*disparity_color)[1] = colormap[3*index + 1];
            (*disparity_color)[0] = colormap[3*index + 2];
        }
    }

    /* Show/save image */
    if (filename) {
        cv::imwrite(filename, disparity_color_);
    } else {
        cv::imshow("disparity_image", disparity_color_);
        cv::waitKey(0);
    }
}
