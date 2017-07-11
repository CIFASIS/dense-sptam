#include "kitti_dmap.hpp"

using namespace std;

vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> load_poses(const char *filename,
                                                          const char *timestamps_file)
{
    FILE *fp = fopen(filename, "r");
    FILE *fp_camera = NULL;

    vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> poses;
    int ret;

    if (!fp) {
        std::cout << "ERROR: Failed to open poses file" << std::endl;
        exit(1);
    }

    const char *e = strrchr(filename, '.');
    if (!e) {
        std::cout << "ERROR: Failed to process poses file" << std::endl;
        exit(1);
    }

    if (strcmp(e, ".txt") == 0) {
        /* Kitti */
        while (!feof(fp)) {
            Eigen::Matrix3d orientation;
            Eigen::Vector3d position;

            ret = fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                        &orientation.row(0)(0), &orientation.row(0)(1), &orientation.row(0)(2), &position(0),
                        &orientation.row(1)(0), &orientation.row(1)(1), &orientation.row(1)(2), &position(1),
                        &orientation.row(2)(0), &orientation.row(2)(1), &orientation.row(2)(2), &position(2));
            if (ret == 12) {
                poses.push_back(make_pair(orientation, position));
            }
        }
    } else if (strcmp(e, ".csv") == 0) {
        /* Eurocmav */
        fp_camera = fopen(timestamps_file, "r");
        if (!fp_camera) {
            std::cout << "ERROR: Failed to open timestamps file" << std::endl;
            exit(1);
        }

        Eigen::Quaterniond orientation;
        Eigen::Vector3d position;

        while (!feof(fp_camera)) {
            long unsigned int stamp_camera, stamp_vicon, dummy;
            ret = fscanf(fp_camera, "%lu,%lu", &stamp_camera, &dummy);
            if (ret != 2)
                goto err_close;

            while (!feof(fp) && stamp_camera > stamp_vicon) {
                ret = fscanf(fp, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                             &stamp_vicon, &position(0), &position(1), &position(2),
                             &orientation.w(), &orientation.x(), &orientation.y(), &orientation.z());
                if (ret != 8)
                    goto err_close;
            }
            poses.push_back(make_pair(orientation.toRotationMatrix(), position));
        }
    } else {
        std::cout << "ERROR: Unrecognized poses format!" << std::endl;
        exit(1);
    }

err_close:
    fclose(fp);
    if (fp_camera)
        fclose(fp_camera);

    return poses;
}

namespace fs = boost::filesystem;

int generate_depth_maps_kitti_global(const char *in_poses_path, const char *in_clouds_path,
                                     const char *out_path, int region_size,
                                     double pub_area_filter_min, Dense *dense_)
{
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> poses = load_poses(in_poses_path, NULL);

    char cloud_path[256];
    int r, c, i, j, limit_l, limit_h;
    fs::path full_path(fs::initial_path<fs::path>());

    full_path = fs::system_complete(fs::path(in_clouds_path));

    if (!fs::exists(full_path) || !fs::is_directory(full_path)) {
        std::cout << "\nDir not found: " << full_path.filename().c_str() << std::endl;
        return 1;
    }

    for (i = 0; i < (int)poses.size(); ++i) {
        cv::Mat_<float> image(dense_->left_info_->height, dense_->left_info_->width);
        for (r = 0; r < image.rows; r++) {
            for (c = 0; c < image.cols; c++) {
                image.at<float>(r, c) = -1;
            }
        }

        sprintf(cloud_path, "%s/%06d.pcd", in_clouds_path, i);
        if (!fs::is_regular_file(cloud_path)) {
          std::cout << "File " << cloud_path << " does not exist \n";
          continue;
        }
        std::cout << "Processing: " << cloud_path << "\n";

        Eigen::Matrix3d orientation = poses.at(i).first;
        Eigen::Vector3d position = poses.at(i).second;

        std::cout << "position: " << position << std::endl;

        FrustumCulling frustum_left(position, orientation,
                                    dense_->camera_->GetFOV_LH(), dense_->camera_->GetFOV_LV(),
                                    dense_->camera_->getNearPlaneDist(), dense_->camera_->getFarPlaneDist());
        PointCloudPtr global(new PointCloud);

        limit_l = 0;
        limit_h = poses.size();
        if (region_size) {
            limit_l = std::max(i - region_size, limit_l);
            limit_h = std::min(i + region_size, limit_h);
        }

        for (j = limit_l; j < limit_h; j++) {
            sprintf(cloud_path, "%s/%06d.pcd", in_clouds_path, j);
            if (!fs::is_regular_file(cloud_path))
                continue;

            PointCloudPtr cloud(new PointCloud);
            pcl::io::loadPCDFile(cloud_path, *cloud);
            for (auto& p : *cloud) {
                CameraPose::Position pos;
                pos(0) = p.x;
                pos(1) = p.y;
                pos(2) = p.z;
                if (frustum_left.Contains(pos) && p.a >= pub_area_filter_min)
                    global->push_back(p);
            }
        }

        for (auto& p : *global) {
            /* To camera coords */
            CameraPose::Position pos;
            pos(0) = p.x;
            pos(1) = p.y;
            pos(2) = p.z;

            pos = orientation.transpose() * (pos - position);
            if (pos(2) <= 0)
                continue;

            cv::Point3d cvpos(pos(0), pos(1), pos(2));

            cv::Point2i pixel = dense_->camera_->getStereoModel().left().project3dToPixel(cvpos);
            if (pixel.y < 0 || pixel.x < 0 || image.rows < pixel.y || image.cols < pixel.x)
                continue;
            if (image.at<float>(pixel.y, pixel.x) == -1 || image.at<float>(pixel.y, pixel.x) > pos(2))
                image.at<float>(pixel.y, pixel.x) = pos(2);
        }

        sprintf(cloud_path, "%s/%06d.dmap", out_path, i);
        saveDepthImage((float*)image.data, dense_->left_info_->height, dense_->left_info_->width, cloud_path);
    }

    return 0;
}

/* Here we use a single and fixed pointcloud from Leica sensor named data.pcd */
int generate_depth_maps_euroc_global(const char *in_poses_path, const char *in_timestamps_path,
                                     const char *in_clouds_path, const char *out_path,
                                     double pub_area_filter_min, Dense *dense_)
{
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> poses = load_poses(in_poses_path, in_timestamps_path);

    char cloud_path[256];
    unsigned int i;
    int r, c;

    fs::path full_path(fs::initial_path<fs::path>());

    full_path = fs::system_complete(fs::path(in_clouds_path));

    if (!fs::exists(full_path) || !fs::is_directory(full_path)) {
        std::cout << "\nDir not found: " << full_path.filename().c_str() << std::endl;
        return 1;
    }

    sprintf(cloud_path, "%s/%s", in_clouds_path, "data.pcd");
    if (!fs::is_regular_file(cloud_path)) {
        std::cout << "ERROR: File " << cloud_path << " not found!" << std::endl;
        exit(1);
    }

    PointCloudPtr cloud(new PointCloud);
    pcl::io::loadPCDFile(cloud_path, *cloud);

    for (i = 0; i < poses.size(); i++) {
        cv::Mat_<float> image(dense_->left_info_->height, dense_->left_info_->width);
        for (r = 0; r < image.rows; r++) {
            for (c = 0; c < image.cols; c++) {
                image.at<float>(r, c) = -1;
            }
        }

        std::cout << "Processing pose: " << i << "\n";

        Eigen::Matrix3d orientation = poses.at(i).first;
        Eigen::Vector3d position = poses.at(i).second;
        FrustumCulling frustum_left(position, orientation,
                                    dense_->camera_->GetFOV_LH(), dense_->camera_->GetFOV_LV(),
                                    dense_->camera_->getNearPlaneDist(), dense_->camera_->getFarPlaneDist());

        std::cout << orientation << std::endl;
        std::cout << position << std::endl;

        PointCloudPtr global(new PointCloud);
        for (auto& p : *cloud) {
            CameraPose::Position pos;
            pos(0) = p.x;
            pos(1) = p.y;
            pos(2) = p.z;
            if (frustum_left.Contains(pos) && p.a >= pub_area_filter_min)
                global->push_back(p);
        }
        for (auto& p : *global) {
            /* To camera coords */
            CameraPose::Position pos;
            pos(0) = p.x;
            pos(1) = p.y;
            pos(2) = p.z;

            pos = orientation.transpose() * (pos - position);
            if (pos(2) <= 0)
                continue;

            cv::Point3d cvpos(pos(0), pos(1), pos(2));

            cv::Point2i pixel = dense_->camera_->getStereoModel().left().project3dToPixel(cvpos);
            if (pixel.y < 0 || pixel.x < 0 || image.rows < pixel.y || image.cols < pixel.x)
                continue;
            if (image.at<float>(pixel.y, pixel.x) == -1 || image.at<float>(pixel.y, pixel.x) > pos(2))
                image.at<float>(pixel.y, pixel.x) = pos(2);
        }

        sprintf(cloud_path, "%s/%06d.dmap", out_path, i);
        saveDepthImage((float*)image.data, dense_->left_info_->height, dense_->left_info_->width, cloud_path);
    }

    return 0;
}

int generate_depth_maps_kitti_local(const char *in_poses_path, const char *in_clouds_path,
                                    const char *out_path, double pub_area_filter_min, Dense *dense_)
{
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> poses = load_poses(in_poses_path, NULL);

    char cloud_path[256];
    unsigned int i;
    int r, c;
    fs::path full_path(fs::initial_path<fs::path>());

    full_path = fs::system_complete(fs::path(in_clouds_path));

    if (!fs::exists(full_path) || !fs::is_directory(full_path)) {
        std::cout << "\nDir not found: " << full_path.filename().c_str() << std::endl;
        return 1;
    }

    for (i = 0; i < poses.size(); i++) {
        sprintf(cloud_path, "%s/%06d.pcd", in_clouds_path, i);

        if (!fs::is_regular_file(cloud_path)) {
          std::cout << "File " << cloud_path << " does not exist \n";
          continue;
        }

        std::cout << "Processing: " << cloud_path << "\n";

        cv::Mat_<float> image(dense_->left_info_->height, dense_->left_info_->width);
        for (r = 0; r < image.rows; r++) {
            for (c = 0; c < image.cols; c++) {
                image.at<float>(r, c) = -1;
            }
        }

        Eigen::Matrix3d orientation = poses.at(i).first;
        Eigen::Vector3d position = poses.at(i).second;
        FrustumCulling frustum_left(position, orientation,
                                    dense_->camera_->GetFOV_LH(), dense_->camera_->GetFOV_LV(),
                                    dense_->camera_->getNearPlaneDist(), dense_->camera_->getFarPlaneDist());

        PointCloudPtr cloud(new PointCloud);
        pcl::io::loadPCDFile(cloud_path, *cloud);
        for (auto& p : *cloud) {
            /* To camera coords */
            CameraPose::Position pos;
            pos(0) = p.x;
            pos(1) = p.y;
            pos(2) = p.z;
            if (!frustum_left.Contains(pos) || p.a < pub_area_filter_min)
                continue;

            pos = orientation.transpose() * (pos - position);
            if (pos(2) <= 0)
                continue;

            cv::Point3d cvpos(pos(0), pos(1), pos(2));

            cv::Point2i pixel = dense_->camera_->getStereoModel().left().project3dToPixel(cvpos);
            if (pixel.y < 0 || pixel.x < 0 || image.rows < pixel.y || image.cols < pixel.x)
                continue;
            if (image.at<float>(pixel.y, pixel.x) == -1 || image.at<float>(pixel.y, pixel.x) > pos(2))
                image.at<float>(pixel.y, pixel.x) = pos(2);
        }

        sprintf(cloud_path, "%s/%06d.dmap", out_path, i);
        saveDepthImage((float*)image.data, dense_->left_info_->height, dense_->left_info_->width, cloud_path);
    }

    return 0;
}

void saveDepthImage(float *depth_map_data, int img_height, int img_width, const char *filename)
{
    FILE *fp = fopen(filename, "w+");
    int i;

    fprintf(fp, "%d,%d\n", img_height, img_width);

    for (i = 0; i < img_width * img_height; i++)
        fprintf(fp, "%f,", depth_map_data[i]);

    fclose(fp);
}



