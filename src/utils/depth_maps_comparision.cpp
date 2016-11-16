#include "depth_maps_comparision.hpp"

using namespace std;

void loadDepthImage(const char *filename, float *disp_data, int *img_height, int *img_width)
{
    FILE *fp = fopen(filename, "r");
    int i;

    assert(fp);

    fscanf(fp, "%d %d\n", img_height, img_width);
    disp_data = (float*)malloc((*img_width) * (*img_height) * sizeof(float));
    for (i = 0; i < (*img_width) * (*img_height); i++)
        fscanf(fp, "%f ", &disp_data[i]);

    fclose(fp);
}

int main(int argc, char* argv[])
{
    float *data[2];
    int height[2], width[2];

    if (argc != 3) {
        std::cout << "\nusage: " << argv[0] << " [gt-image] [our-image]" << std::endl;
        return -1;
    }

    loadDepthImage(argv[1], data[0], &height[0], &width[0]);
    loadDepthImage(argv[2], data[1], &height[1], &width[1]);

    return 0;
}
