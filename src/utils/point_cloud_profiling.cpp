#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include "../dense/PointCloudQueue.hpp"
#include "../dense/dense.hpp"

using namespace std;

namespace fs = boost::filesystem;

struct profiling_data {
    unsigned int points_total;
    unsigned int points_hypothesis;
    unsigned int points_validated;
};

static void usage(char *name)
{
    std::cout << "usage:" << std::endl <<
                 "  " << name << " <point_cloud_directory>" << std::endl;
}

int process_point_cloud(fs::path path, struct profiling_data *profiling)
{
    PointCloudPtr cloud(new PointCloud);

    char filename[256];

    sprintf(filename, "%s/%s", path.parent_path().c_str(), path.filename().c_str());
    std::cout << "Processing: " << filename << std::endl;

    pcl::io::loadPCDFile(filename, *cloud);

    for (auto& p : *cloud) {
        profiling->points_total++;

        if (p.probability == POINT_NEW_PROBABILITY)
            profiling->points_hypothesis++;
        else if (p.probability > POINT_NEW_PROBABILITY)
            profiling->points_validated++;
    }

    return 0;
}

int main(int argc, char **argv)
{
    struct profiling_data profiling = { 0 };

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    fs::path path(fs::initial_path<fs::path>());
    path = fs::system_complete(fs::path(argv[1]));

    if (!fs::exists(path) || !fs::is_directory(path)) {
        std::cout << "Dir not found: " << path.filename().c_str() << std::endl;
        return -1;
    }

    const fs::directory_iterator end{};

    for (fs::directory_iterator iter{path}; iter != end; ++iter) {
        if (!fs::is_regular_file(*iter) || iter->path().extension() != ".pcd")
        continue;

        process_point_cloud(iter->path(), &profiling);
    }

    std::cout << std::endl;
    std::cout << "RESULTS: " << std::endl;
    std::cout << "  points total:      " << profiling.points_total << std::endl;
    std::cout << "  points hypothesis: " << profiling.points_hypothesis << std::endl;
    std::cout << "  points validated:  " << profiling.points_validated << std::endl;

    return 0;
}
