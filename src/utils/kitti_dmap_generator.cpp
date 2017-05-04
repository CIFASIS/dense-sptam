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


  std::string parametersFileYML, posesFile, pcdPath;


  // cargar parametros
  ProgramOptions program_options( argv[0] );

  program_options.addPositionalArgument("configuration", "configuration file with all the parameters.", parametersFileYML);
  program_options.addPositionalArgument("poses", "poses file", posesFile);
  program_options.addPositionalArgument("pcd_dir", "pcd directory path", pcdPath);

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

  /** Load program parameters from configuration file */


  std::string single_depth_map_poses;
  std::string single_depth_map_clouds;
  size_t single_depth_map_region_size;
  double pub_area_filter_min;

  const sensor_msgs::CameraInfoConstPtr left_info;
  const sensor_msgs::CameraInfoConstPtr right_info;
  double frustumNearPlaneDist;
  double frustumFarPlaneDist;
  double voxelLeafSize;
  double filter_meanK;
  double filter_stddev;
  std::string output_dir;
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
      if (key == "FrustumNearPlaneDist") frustumNearPlaneDist = it.second.as<double>();
//      else if (key == "frustumFarPlaneDist") frustumFarPlaneDist = it.second.as<double>();
//      else if (key == "voxelLeafSize") voxelLeafSize = it.second.as<double>();
//      else if (key == "filter_meanK") filter_meanK = it.second.as<double>();
//      else if (key == "filter_stddev") filter_stddev = it.second.as<double>();
    }
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



  Dense *dense = new Dense( left_info, right_info,
                           frustumNearPlaneDist, frustumFarPlaneDist, voxelLeafSize,
                           filter_meanK, filter_stddev, output_dir, disp_calc_method,
                           filter_radius, filter_minneighbours, max_distance, stereoscan_threshold,
                           local_area_size, libelas_ipol_gap, add_corners, sigma,
                           refinement_linear_threshold, refinement_angular_threshold);


  generate_depth_maps_kitti_global(single_depth_map_poses.c_str(), single_depth_map_clouds.c_str(),
                                   single_depth_map_clouds.c_str(), single_depth_map_region_size,
                                   pub_area_filter_min, dense);


  return 0;
}





