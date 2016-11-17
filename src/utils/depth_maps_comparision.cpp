#include "depth_maps_comparision.hpp"

#include <math.h>
#include <boost/smart_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

float *loadDepthImage(const char *filename, int *img_height, int *img_width)
{
    FILE *fp = fopen(filename, "r");
    int i;

    assert(fp);

    fscanf(fp, "%d %d\n", img_height, img_width);
    float *disp_data = (float*)malloc((*img_width) * (*img_height) * sizeof(float));

    for (i = 0; i < (*img_width) * (*img_height); i++)
        fscanf(fp, "%f ", &disp_data[i]);

    fclose(fp);
    return disp_data;
}

/* Mean absolute error */
void do_mae(float *data[2], int img_height, int img_width)
{
    unsigned int count = 0, gt_valid = 0, our_valid = 0;
    double total = 0, min, max, curr;

    for (int i = 0; i < img_height * img_width; i++) {
        if (data[0][i] != -1)
            gt_valid++;

        if (data[1][i] != -1)
            our_valid++;

        if (data[0][i] == -1 || data[1][i] == -1)
            continue;

        curr = std::abs(data[0][i] - data[1][i]);

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
    std::cout << "gt_valid/our_valid: " << gt_valid << "/" << our_valid << std::endl;
    total /= count;
    std::cout << "MAE: " << total << std::endl;
    std::cout << "Min/max: " << min << "/" << max << std::endl;
}

/* Root-mean-square deviation */
int do_rmse(float *data[2], int img_height, int img_width)
{
    //cv::Mat_<float> gt_mat(img_height, img_width, data[0]);
    //cv::Mat_<float> our_mat(img_height, img_width, data[1]);
    //gt_mat = gt_mat - our_mat;

    unsigned int count = 0;
    double total = 0, min, max, curr;

    for (int i = 0; i < img_height * img_width; i++) {
        if (data[0][i] == -1 || data[1][i] == -1)
            continue;

        curr = data[0][i] - data[1][i];
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
int do_silmse(float *data[2], int img_height, int img_width)
{
    unsigned int count = 0;
    double total = 0, total_pow2 = 0, min, max, curr;

    for (int i = 0; i < img_height * img_width; i++) {
        if (data[0][i] == -1 || data[1][i] == -1)
            continue;

        curr = log(data[0][i]) - log(data[1][i]);
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

int usage(const char *program_name)
{
    std::cout << "\nusage: " << program_name << " [gt-image] [our-image] {mae|rmse|silmse}" << std::endl;
    return -1;
}

int main(int argc, char* argv[])
{
    float *data[2];
    int height[2], width[2];

    if (argc != 4)
        return usage(argv[0]);

    data[0] = loadDepthImage(argv[1], &height[0], &width[0]);
    data[1] = loadDepthImage(argv[2], &height[1], &width[1]);
    assert(height[0] == height[1] && width[0] == width[1]);

    if (strcmp(argv[3], "mae") == 0) {
        do_mae(data, height[0], width[0]);
    } else if (strcmp(argv[3], "rmse") == 0) {
        do_rmse(data, height[0], width[0]);
    } else if (strcmp(argv[3], "silmse") == 0) {
        do_silmse(data, height[0], width[0]);
    } else {
        return usage(argv[0]);
    }

    return 0;
}
