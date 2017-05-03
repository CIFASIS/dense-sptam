#include "kitti_dmap.hpp"

int main(){


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





