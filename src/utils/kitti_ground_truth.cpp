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

#include "kitti_ground_truth.hpp"
#include "../dense/Camera.hpp"

using namespace std;

pair<Eigen::Matrix3d, Eigen::Vector3d> load_calib(const char *filename)
{
	FILE *fp = fopen(filename, "r");
	pair<Eigen::Matrix3d, Eigen::Vector3d> pose;
	char name[5];
	int ret;

	if (!fp)
		exit(1);

	while (!feof(fp)) {
		Eigen::Matrix3d orientation;
		Eigen::Vector3d position;

		ret = fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", name,
					 &orientation.row(0)(0), &orientation.row(0)(1), &orientation.row(0)(2), &position(0),
					 &orientation.row(1)(0), &orientation.row(1)(1), &orientation.row(1)(2), &position(1),
					 &orientation.row(2)(0), &orientation.row(2)(1), &orientation.row(2)(2), &position(2));
		if (ret == 13 && (strcmp(name, "Tr:") == 0)) {
			pose = make_pair(orientation, position);
			break;
		}
	}

	if (feof(fp)) {
		cout << "Calib not found!" << endl;
		exit(1);
	}

	fclose(fp);
	return pose;
}

vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> load_poses(const char *filename)
{
	FILE *fp = fopen(filename, "r");
	vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> poses;
	int ret;

	if (!fp)
		exit(1);

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

	fclose(fp);

	return poses;
}

PointCloudPtr load_velodyne(const char *filename, pair<Eigen::Matrix3d, Eigen::Vector3d> calib,
							Eigen::Matrix3d orientation, Eigen::Vector3d position, float min_distance)
{
	PointCloudPtr ret(new PointCloud);
	Eigen::Matrix3d calib_orientation = calib.first;
	Eigen::Vector3d calib_position = calib.second;

	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	int32_t num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;

	// load point cloud
	FILE *stream;
	stream = fopen(filename, "r");
	num = fread(data, sizeof(float), num, stream) / 4;
	for (int32_t i = 0; i < num; i++) {
		Eigen::Vector3d pt3d;
		pt3d(0) = *px;
		pt3d(1) = *py;
		pt3d(2) = *pz;
		pt3d = calib_orientation * pt3d + calib_position;

		/*
		 * Omit points that are beyond the min_distance threshold.
		 * Note that velodyne is 360º.
		 */
		if (pt3d.norm() < min_distance)
			continue;

		pt3d = orientation * pt3d + position;
		Point pt;
		pt.x = pt3d(0);
		pt.y = pt3d(1);
		pt.z = pt3d(2);
		pt.rgb = *pr;
		ret->push_back(pt);
		px += 4;
		py += 4;
		pz += 4;
		pr += 4;
	}

	fclose(stream);
	free(data);

	return ret;
}

namespace fs = boost::filesystem;

int generate_clouds(const char *in_path, const char *out_path,
					pair<Eigen::Matrix3d, Eigen::Vector3d> calib,
					vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> poses,
					float min_distance)
{
	unsigned int count = 0, i;
	char cloud_path[256];
	fs::path full_path(fs::initial_path<fs::path>());

	full_path = fs::system_complete(fs::path(in_path));

	if (!fs::exists(full_path) || !fs::is_directory(full_path)) {
		std::cout << "\nDir not found: " << full_path.filename().c_str() << std::endl;
		return 1;
	}

	for (i = 0; i < poses.size(); i++) {
		sprintf(cloud_path, "%s/%06d.bin", in_path, i);
		std::cout << "Processing: " << cloud_path << "\n";
		if (!fs::is_regular_file(cloud_path)) {
			std::cout << "    NOT FOUND: " << cloud_path << "\n";
			continue;
		}

		Eigen::Matrix3d orientation = poses.at(count).first;
		Eigen::Vector3d position = poses.at(count).second;
		PointCloudPtr cloud = load_velodyne(cloud_path, calib, orientation, position, min_distance);

		sprintf(cloud_path, "%s/%06d.pcd", out_path, i);
		string filename(cloud_path);
		pcl::io::savePCDFileBinary(filename, *cloud);
		count++;
		std::cout << "    saved in: " << filename.c_str() << "\n";
	}

	cout << "TOTAL: " << count << " clouds" << endl;

	return 0;
}

int main(int argc, char* argv[])
{
	float min_distance;
	int ret = 0;

	if (argc < 6) {
		std::cout << "\nusage: " << argv[0] <<
			" [in-calib] [in-poses] [in-velo] [out-cloud] [min-distance]" << std::endl << std::endl <<
			"    in-calib: input file from KITTI ground truth containing camera calibration parameters." << std::endl <<
			"    in-poses: input file from KITTI ground truth containing the list of poses." << std::endl <<
			"    in-velo: input folder from KITTI ground truth containing the velodyne binary clouds." << std::endl <<
			"    out-cloud: output folder where PCD point cloud are going to be stored." << std::endl <<
			"    min-distance: omit points that are closer than this distance threshold." << std::endl;
		return -1;
	}

	pair<Eigen::Matrix3d, Eigen::Vector3d> calib = load_calib(argv[1]);
	vector<pair<Eigen::Matrix3d, Eigen::Vector3d>> poses = load_poses(argv[2]);
	min_distance = atof(argv[5]);

	ret = generate_clouds(argv[3], argv[4], calib, poses, min_distance);
	if (ret) {
		cout << "Failed to generate clouds" << endl;
		exit(1);
	}

	return 0;
}
