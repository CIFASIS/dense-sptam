#include "depth_maps_comparision.hpp"

#include <math.h>
#include <boost/smart_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

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

float *loadDepthImage(const char *filename, int *img_height, int *img_width)
{
    FILE *fp = fopen(filename, "r");
    int i;

    assert(fp);

    fscanf(fp, "%d,%d\n", img_height, img_width);
    float *disp_data = (float*)malloc((*img_width) * (*img_height) * sizeof(float));

    for (i = 0; i < (*img_width) * (*img_height); i++)
        fscanf(fp, "%f,", &disp_data[i]);

    fclose(fp);
    return disp_data;
}

void saveDepthImage(float *disp_data, int img_height, int img_width, const char *filename)
{
    FILE *fp = fopen(filename, "w+");
    int i;

    fprintf(fp, "%d,%d\n", img_height, img_width);

    for (i = 0; i < img_width * img_height; i++)
        fprintf(fp, "%f,", disp_data[i]);

    fclose(fp);
}

void saveColorImage(float *disp_data, int img_height, int img_width, const char *filename)
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

/* Mean absolute error */
void do_mae(float *gt_data, float *our_data, int img_height, int img_width)
{
    unsigned int count = 0, gt_valid = 0, our_valid = 0;
    double total = 0, min, max, curr;

    for (int i = 0; i < img_height * img_width; i++) {
        if (gt_data[i] != -1)
            gt_valid++;
        if (our_data[i] != -1)
            our_valid++;

        if (gt_data[i] == -1 || our_data[i] == -1)
            continue;

        curr = std::abs(gt_data[i] - our_data[i]);

        if (!count)
            min = max = curr;
        if (min > curr)
            min = curr;
        if (max < curr)
            max = curr;

        total += curr;
        count++;
    }

    std::cout << "Total: " << total << std::endl;
    std::cout << "  Valid: gt/our/intersect: " << gt_valid << "/" << our_valid << "/" << count << std::endl;

    total /= count;
    std::cout << "  MAE: " << total << std::endl;
    std::cout << "  Min/max: " << min << "/" << max << std::endl;
}

/* Root-mean-square deviation */
void do_rmse(float *gt_data, float *our_data, int img_height, int img_width)
{
    //cv::Mat_<float> gt_mat(img_height, img_width, data[0]);
    //cv::Mat_<float> our_mat(img_height, img_width, data[1]);
    //gt_mat = gt_mat - our_mat;

    unsigned int count = 0;
    double total = 0, min, max, curr;

    for (int i = 0; i < img_height * img_width; i++) {
        if (gt_data[i] == -1 || our_data[i] == -1)
            continue;

        curr = gt_data[i] - our_data[i];
        curr *= curr;

        if (!count)
            min = max = curr;
        if (min > curr)
            min = curr;
        if (max < curr)
            max = curr;

        total += curr;
        count++;
    }

    std::cout << "Total/count: " << total << "/" << count << std::endl;
    total = sqrt(total / count);
    std::cout << "RMSE: " << total << std::endl;
    std::cout << "Min/max: " << min << "/" << max << std::endl;
}

/*
 * Scale-invariant mean squared error (in log space)
 * David Eigen, "Depth Map prediction from Single Image using Multi-Scale Deep Network"
 */
void do_silmse(float *gt_data, float *our_data, int img_height, int img_width)
{
    unsigned int count = 0;
    double total = 0, total_pow2 = 0, min, max, curr;

    for (int i = 0; i < img_height * img_width; i++) {
        if (gt_data[i] == -1 || our_data[i] == -1)
            continue;

        curr = log(gt_data[i]) - log(our_data[i]);
        total += curr;
        curr *= curr;

        if (!count)
            min = max = curr;
        if (min > curr)
            min = curr;
        if (max < curr)
            max = curr;

        total_pow2 += curr;
        count++;
    }

    std::cout << "Total/count: " << total << "/" << count << std::endl;
    total = total_pow2 / count - (total * total) / (count * count);
    std::cout << "SILMSE: " << total << std::endl;
    std::cout << "Min/max: " << min << "/" << max << std::endl;
}

void do_intersec(float *gt_data, float *our_data, int img_height, int img_width)
{
    for (int i = 0; i < img_height * img_width; i++) {
        if (gt_data[i] == -1 || our_data[i] == -1) {
            gt_data[i] = -1;
            our_data[i] = -1;
        }
    }
}

void do_error_color(float *gt_data, float *our_data, int img_height, int img_width)
{
    for (int i = 0; i < img_height * img_width; i++) {
        if (gt_data[i] == -1 || our_data[i] == -1) {
            gt_data[i] = -1;
            continue;
        }
        gt_data[i] = std::abs(gt_data[i] - our_data[i]);
    }
}

void do_error_graph(float *gt_data, float *our_data, int img_height, int img_width, float size)
{
    unsigned int total = 255 / size + 1;
    unsigned int histogram[total], pos;
    unsigned int i;

    for (i = 0; i < total; i++)
        histogram[i] = 0;

    for (int j = 0; j < img_height * img_width; j++) {
        if (gt_data[j] == -1 || our_data[j] == -1) {
            gt_data[j] = -1;
            continue;
        }
        gt_data[j] = std::abs(gt_data[j] - our_data[j]);
        pos = gt_data[j] / size;
        if (pos >= total) {
            std::cout << "FAILED: pos/total: " << pos << "/" << total << std::endl;
            assert(false);
        }
        histogram[pos]++;
    }

    std::cout << size << std::endl;
    for (i = 0; i < total; i++)
        std::cout << histogram[i] << std::endl;
}

int usage(const char *program_name)
{
    std::cout << "\nusage: " << program_name <<
                 " [gt-image] [our-image] {mae|rmse|silmse|intersect|error_color|error_graph}" << std::endl;
    return -1;
}

int main(int argc, char* argv[])
{
    float *data[2];
    int height[2], width[2];
    char filename[512];

    if (argc < 4)
        return usage(argv[0]);

    data[0] = loadDepthImage(argv[1], &height[0], &width[0]);
    data[1] = loadDepthImage(argv[2], &height[1], &width[1]);
    assert(height[0] == height[1] && width[0] == width[1]);

    if (strcmp(argv[3], "mae") == 0) {
        do_mae(data[0], data[1], height[0], width[0]);
    } else if (strcmp(argv[3], "rmse") == 0) {
        do_rmse(data[0], data[1], height[0], width[0]);
    } else if (strcmp(argv[3], "silmse") == 0) {
        do_silmse(data[0], data[1], height[0], width[0]);
    } else if (strcmp(argv[3], "intersect") == 0) {
        do_intersec(data[0], data[1], height[0], width[0]);
        for (int i = 0; i < 2; i++) {
            sprintf(filename, "%s%s", argv[i + 1], ".valid.jpg");
            saveColorImage(data[i], height[i], width[i], filename);
            std::cout << "Saved valid map: " << filename << std::endl;
        }
    } else if (strcmp(argv[3], "error_color") == 0) {
        do_error_color(data[0], data[1], height[0], width[0]);
        sprintf(filename, "%s%s", argv[1], ".error.jpg");
        saveColorImage(data[0], height[0], width[0], filename);
        std::cout << "Saved error color map: " << filename << std::endl;
    } else if (strcmp(argv[3], "error_graph") == 0) {
        if (argc != 5)
            return usage(argv[0]);
        do_error_graph(data[0], data[1], height[0], width[0], atof(argv[4]));
    } else {
        return usage(argv[0]);
    }

    return 0;
}
