#include "dense.hpp"

Dense::Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info,
             double frustumNearPlaneDist, double frustumFarPlaneDist, double voxelLeafSize,
             std::string output_dir, std::string disp_calc_method,
             double max_distance, double stereoscan_threshold,
             int local_area_size, int libelas_ipol_gap, bool add_corners, double sigma,
             double refinement_linear_threshold, double refinement_angular_threshold)
  : left_info_(left_info)
  , right_info_(right_info)
  , frustumNearPlaneDist_(frustumNearPlaneDist)
  , frustumFarPlaneDist_(frustumFarPlaneDist)
  , voxelLeafSize_(voxelLeafSize)
  , output_dir_(output_dir)
  , disp_calc_method_(disp_calc_method)
  , max_distance_(max_distance)
  , stereoscan_threshold_(stereoscan_threshold)
  , local_area_size_(local_area_size)
  , libelas_ipol_gap_(libelas_ipol_gap)
  , add_corners_(add_corners)
  , sigma_(sigma)
  , refinement_linear_threshold_(refinement_linear_threshold)
  , refinement_angular_threshold_(refinement_angular_threshold)
{
    log_file_ = fopen("dense_node.log", "w");
    assert(log_file_);

    camera_ = new Camera(left_info_, right_info_, frustumNearPlaneDist_, frustumFarPlaneDist_);
    raw_image_pairs_ = new ImageQueue();
    disp_images_ = new DispImageQueue();
    point_clouds_ = new PointCloudQueue(local_area_size_);

    disparityCalcThread = new DisparityCalcThread(this);
    projectionThread_ = new ProjectionThread(this);
    refinementThread_ = new RefinementThread(this);
}

Dense::~Dense()
{
    fclose(log_file_);
}

void Dense::WriteToLog(const char* fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vfprintf(log_file_, fmt, args);
    va_end(args);
}
