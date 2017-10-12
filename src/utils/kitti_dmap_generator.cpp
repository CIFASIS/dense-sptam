#include <yaml-cpp/yaml.h>

#include "kitti_dmap.hpp"
#include "ProgramOptions.hpp"
#include "../dense/Camera.hpp"

template<typename T>
void loadParameter(const YAML::Node& config, const std::string key, T& ret, const T& default_value)
{
	try {
		ret = config[ key ].as<T>();
	} catch(YAML::KeyNotFound& e) {
		ret = default_value;
	}
}

sensor_msgs::CameraInfoPtr
CreateCameraInfoMsg(const int image_width, const int image_height,
					const cv::Mat& intrinsic, const cv::Mat& rotation,
					const cv::Vec3d& traslation)
{
	// create cameraInfo pointer message
	sensor_msgs::CameraInfoPtr cameraInfo( new sensor_msgs::CameraInfo() );

	// fill message
	cameraInfo->header.frame_id = "stereo";
	cameraInfo->height = image_height;
	cameraInfo->width = image_width;
	cameraInfo->K = {intrinsic.at<double>(0,0), intrinsic.at<double>(0,1), intrinsic.at<double>(0,2),
	intrinsic.at<double>(1,0), intrinsic.at<double>(1,1), intrinsic.at<double>(1,2),
	intrinsic.at<double>(2,0), intrinsic.at<double>(2,1), intrinsic.at<double>(2,2)};

	std::cout << "rotation: " << rotation << std::endl;

	cameraInfo->R = {rotation.at<double>(0,0), rotation.at<double>(0,1), rotation.at<double>(0,2),
	rotation.at<double>(1,0), rotation.at<double>(1,1), rotation.at<double>(1,2),
	rotation.at<double>(2,0), rotation.at<double>(2,1), rotation.at<double>(2,2)};

	// construct [R|t] matrix
	cv::Mat Rt = (cv::Mat_<double>(3,4) << rotation.at<double>(0,0), rotation.at<double>(0,1), rotation.at<double>(0,2), traslation(0),
	rotation.at<double>(1,0), rotation.at<double>(1,1), rotation.at<double>(1,2), traslation(1),
	rotation.at<double>(2,0), rotation.at<double>(2,1), rotation.at<double>(2,2), traslation(2));

	// compute projection matrix
	cv::Mat projection = intrinsic * Rt;
	cameraInfo->P = {projection.at<double>(0,0), projection.at<double>(0,1), projection.at<double>(0,2), projection.at<double>(0,3),
	projection.at<double>(1,0), projection.at<double>(1,1), projection.at<double>(1,2), projection.at<double>(1,3),
	projection.at<double>(2,0), projection.at<double>(2,1), projection.at<double>(2,2), projection.at<double>(2,3)};

	std::cout << "projection: " << projection << std::endl;

	return cameraInfo;
}

int main(int argc, char* argv[])
{
	std::string calibrationFile, parametersFileYML, posesFile, pcdPath, output_path;

	// cargar parametros
	ProgramOptions program_options( argv[0] );

	program_options.addPositionalArgument("calibration", "camera calibration file", calibrationFile);
	program_options.addPositionalArgument("configuration", "configuration file with all the parameters.", parametersFileYML);
	program_options.addPositionalArgument("poses", "poses file", posesFile);
	program_options.addPositionalArgument("pcd_path", "pcd directory path", pcdPath);

	/** Parse the program options */
	try {
		// may throw
		program_options.parse( argc, argv );

		// if count 'help' show help and exit
		if (program_options.count("help") ) {
			std::cerr << program_options << std::endl;
			return 0;
		}

		// throws on error, so do after help in case there are any problems.
		program_options.notify();
	} catch(boost::program_options::error& e) {
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << program_options << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Poses path: " << posesFile << std::endl;
	std::cout << "pcd directory path: " << pcdPath << std::endl;

	/** Load program parameters from configuration file */
	struct dense_parameters parameters;

	try {
		YAML::Node config = YAML::LoadFile( parametersFileYML );

		for (auto it : config) {
			std::string key = it.first.as<std::string>();
			// Dense constructor parameters
			if (key == "frustum_near_plane_dist")
				parameters.frustum_near_plane_dist = it.second.as<double>();
			else if (key == "frustum_far_plane_dist")
				parameters.frustum_far_plane_dist = it.second.as<double>();
			else if (key == "voxel_leaf_size")
				parameters.voxel_leaf_size = it.second.as<double>();
			else if (key == "disp_calc_method")
				parameters.disp_calc_method = it.second.as<std::string>();
			else if (key == "max_distance")
				parameters.max_distance = it.second.as<double>();
			else if (key == "stereoscan_threshold")
				parameters.stereoscan_threshold = it.second.as<double>();
			else if (key == "fusion_heuristic")
				parameters.fusion_heuristic = it.second.as<std::string>();
			else if (key == "local_area_size")
				parameters.local_area_size = it.second.as<int>();
			else if (key == "libelas_ipol_gap")
				parameters.libelas_ipol_gap = it.second.as<double>();
			else if (key == "add_corners")
				parameters.add_corners = it.second.as<bool>();
			else if (key == "sigma")
				parameters.sigma = it.second.as<double>();
			else if (key == "refinement_linear_threshold")
				parameters.refinement_linear_threshold = it.second.as<double>();
			else if (key == "refinement_angular_threshold")
				parameters.refinement_angular_threshold = it.second.as<double>();
			// other parameters
			else if (key == "single_depth_map_region_size")
				parameters.single_depth_map_region_size = it.second.as<int>();
			else if (key == "pub_area_filter_min")
				parameters.pub_area_filter_min = it.second.as<double>();
		}

		std::cout << "frustumNearPlaneDist: " << parameters.frustum_near_plane_dist << std::endl;
		std::cout << "frustumFarPlaneDist: " << parameters.frustum_far_plane_dist << std::endl;
		std::cout << "voxelLeafSize: " << parameters.voxel_leaf_size << std::endl;
		std::cout << "disp_calc_method: " << parameters.disp_calc_method << std::endl;
		std::cout << "max_distance: " << parameters.max_distance << std::endl;
		std::cout << "stereoscan_threshold: " << parameters.stereoscan_threshold << std::endl;
		std::cout << "local_area_size: " << parameters.local_area_size << std::endl;
		std::cout << "libelas_ipol_gap: " << parameters.libelas_ipol_gap << std::endl;
		std::cout << "add_corners: " << parameters.add_corners << std::endl;
		std::cout << "sigma: " << parameters.sigma << std::endl;
		std::cout << "refinement_linear_threshold: " << parameters.refinement_linear_threshold << std::endl;
		std::cout << "refinement_angular_threshold: " << parameters.refinement_angular_threshold << std::endl;
		std::cout << "single_depth_map_region_size: " << parameters.single_depth_map_region_size << std::endl;
		std::cout << "pub_area_filter_min: " << parameters.pub_area_filter_min << std::endl;
	} catch(YAML::BadFile& e) {
		std::cerr << "Could not open configuration file " << parametersFileYML << std::endl;
		return EXIT_FAILURE;
	} catch(YAML::ParserException& e) {
		std::cerr << "Could not parse configuration file " << parametersFileYML << ". " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	/** Load camera calibration */
	int image_width, image_height;
	cv::Mat_<double> intrinsic;
	double baseline;

	try {
		cv::FileStorage config(calibrationFile, cv::FileStorage::READ);

		config["camera_matrix"] >> intrinsic;
		image_width = (int) config["image_width"];
		image_height = (int) config["image_height"];
		baseline = (double) config["baseline"];

		std::cout << "image_width: " << image_width << std::endl;
		std::cout << "image_height: " << image_height << std::endl;
		std::cout << "camera_matrix: " << intrinsic << std::endl;
		std::cout << "baseline: " << baseline << std::endl;
	} catch(YAML::BadFile& e) {
		std::cerr << "Could not open calibration file " << calibrationFile << std::endl;
		return EXIT_FAILURE;
	} catch(YAML::ParserException& e) {
		std::cerr << "Could not parse calibration file " << calibrationFile << ". " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	// set rotation and translation matrices
	cv::Mat rotation = cv::Mat::eye(3,3,CV_64FC1);
	cv::Vec3d traslation(-baseline,0,0);

	// Create CameraInfo messages
	sensor_msgs::CameraInfoPtr left_info = CreateCameraInfoMsg(image_width, image_height, intrinsic, rotation, cv::Vec3d(0,0,0));
	sensor_msgs::CameraInfoPtr right_info = CreateCameraInfoMsg(image_width, image_height, intrinsic, rotation, traslation);;

	//  for (int i = 0; i < 12; ++i)
	//      std::cout << left_info->P[i] << std::endl;

	Camera *camera = new Camera(left_info, right_info, parameters.frustum_near_plane_dist,
								parameters.frustum_far_plane_dist);

	// generate depth maps (.dmap files)
	generate_depth_maps_kitti_global(posesFile.c_str(), pcdPath.c_str(),
									 pcdPath.c_str(), parameters.single_depth_map_region_size,
									 parameters.pub_area_filter_min, left_info, camera);

	return 0;
}
