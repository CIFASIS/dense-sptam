#include "kitti_dmap.hpp"
#include "ProgramOptions.hpp"
#include <yaml-cpp/yaml.h>


template<typename T>
void loadParameter(const YAML::Node& config, const std::string key, T& ret, const T& default_value)
{
  try {
    ret = config[ key ].as<T>();
  } catch(YAML::KeyNotFound& e) {
    ret = default_value;
  }
}


int main(int argc, char* argv[]){


  std::string parametersFileYML, posesFile, pcdPath, output_path;


  // cargar parametros
  ProgramOptions program_options( argv[0] );

  program_options.addPositionalArgument("configuration", "configuration file with all the parameters.", parametersFileYML);
  program_options.addPositionalArgument("poses", "poses file", posesFile);
  program_options.addPositionalArgument("pcd_path", "pcd directory path", pcdPath);

  /** Parse the program options */

  try
  {
    // may throw
    program_options.parse( argc, argv );

    // if count 'help' show help and exit
    if (program_options.count("help") ) {
      std::cerr << program_options << std::endl;
      return 0;
    }

    // throws on error, so do after help in case there are any problems.
    program_options.notify();
  }
  catch(boost::program_options::error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << program_options << std::endl;
    return EXIT_FAILURE;
  }



  std::cout << "Poses path: " << posesFile << std::endl;
  std::cout << "pcd directory path: " << pcdPath << std::endl;


  /** Load program parameters from configuration file */

  size_t single_depth_map_region_size;
  double pub_area_filter_min;

  const sensor_msgs::CameraInfoConstPtr left_info;
  const sensor_msgs::CameraInfoConstPtr right_info;
  double frustumNearPlaneDist;
  double frustumFarPlaneDist;
  double voxelLeafSize;
  double filter_meanK;
  double filter_stddev;
  std::string disp_calc_method;
  double filter_radius;
  double filter_minneighbours;
  double max_distance;
  double stereoscan_threshold;
  int local_area_size;
  int libelas_ipol_gap;
  bool add_corners;
  double sigma;
  double refinement_linear_threshold;
  double refinement_angular_threshold;


  try
  {
    YAML::Node config = YAML::LoadFile( parametersFileYML );

    for (auto it : config)
    {
      std::string key = it.first.as<std::string>();
      // Dense constructor parameters
      if (key == "FrustumNearPlaneDist") frustumNearPlaneDist = it.second.as<double>();
      else if (key == "FrustumFarPlaneDist") frustumFarPlaneDist = it.second.as<double>();
      else if (key == "VoxelLeafSize") voxelLeafSize = it.second.as<double>();
      else if (key == "filter_meanK") filter_meanK = it.second.as<double>();
      else if (key == "filter_stddev") filter_stddev = it.second.as<double>();
      else if (key == "disp_calc_method") disp_calc_method = it.second.as<std::string>();
      else if (key == "filter_radius") filter_radius = it.second.as<double>();
      else if (key == "filter_minneighbours") filter_minneighbours = it.second.as<double>();
      else if (key == "max_distance") max_distance = it.second.as<double>();
      else if (key == "stereoscan_threshold") stereoscan_threshold = it.second.as<double>();
      else if (key == "local_area_size") local_area_size = it.second.as<int>();
      else if (key == "libelas_ipol_gap") libelas_ipol_gap = it.second.as<double>();
      else if (key == "add_corners") add_corners = it.second.as<bool>();
      else if (key == "sigma") sigma = it.second.as<double>();
      else if (key == "refinement_linear_threshold") refinement_linear_threshold = it.second.as<double>();
      else if (key == "refinement_angular_threshold") refinement_angular_threshold = it.second.as<double>();
      // other parameters
      else if (key == "single_depth_map_region_size") single_depth_map_region_size = it.second.as<int>();
      else if (key == "pub_area_filter_min") pub_area_filter_min = it.second.as<double>();
    }

    std::cout << "frustumNearPlaneDist: " << frustumNearPlaneDist << std::endl;
    std::cout << "frustumFarPlaneDist: " << frustumFarPlaneDist << std::endl;
    std::cout << "voxelLeafSize: " << voxelLeafSize << std::endl;
    std::cout << "filter_meanK: " << filter_meanK << std::endl;
    std::cout << "filter_stddev: " << filter_stddev << std::endl;
    std::cout << "disp_calc_method: " << disp_calc_method << std::endl;
    std::cout << "filter_radius: " << filter_radius << std::endl;
    std::cout << "filter_minneighbours: " << filter_minneighbours << std::endl;
    std::cout << "max_distance: " << max_distance << std::endl;
    std::cout << "stereoscan_threshold: " << stereoscan_threshold << std::endl;
    std::cout << "local_area_size: " << local_area_size << std::endl;
    std::cout << "libelas_ipol_gap: " << libelas_ipol_gap << std::endl;
    std::cout << "add_corners: " << add_corners << std::endl;
    std::cout << "sigma: " << sigma << std::endl;
    std::cout << "refinement_linear_threshold: " << refinement_linear_threshold << std::endl;
    std::cout << "refinement_angular_threshold: " << refinement_angular_threshold << std::endl;
    std::cout << "single_depth_map_region_size: " << single_depth_map_region_size << std::endl;
    std::cout << "pub_area_filter_min: " << pub_area_filter_min << std::endl;
  }
  catch(YAML::BadFile& e)
  {
    std::cerr << "Could not open configuration file " << parametersFileYML << std::endl;
    return EXIT_FAILURE;
  }
  catch(YAML::ParserException& e)
  {
    std::cerr << "Could not parse configuration file " << parametersFileYML << ". " << e.what() << std::endl;
    return EXIT_FAILURE;
  }


  // create Dense instance
  Dense *dense = new Dense( left_info, right_info,
                            frustumNearPlaneDist, frustumFarPlaneDist, voxelLeafSize,
                            filter_meanK, filter_stddev, output_path, disp_calc_method,
                            filter_radius, filter_minneighbours, max_distance, stereoscan_threshold,
                            local_area_size, libelas_ipol_gap, add_corners, sigma,
                            refinement_linear_threshold, refinement_angular_threshold );

  // generate depth maps (.dmap files)
  generate_depth_maps_kitti_global( posesFile.c_str(), pcdPath.c_str(),
                                    pcdPath.c_str(), single_depth_map_region_size,
                                    pub_area_filter_min, dense );


  return 0;
}





