#include "boost/filesystem.hpp"
#include "dense.hpp"

#define LOG_FILENAME        "dense_node.log"

namespace fs = boost::filesystem;

Dense::Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info,
			 struct dense_parameters *parameters)
  : left_info_(left_info)
  , right_info_(right_info)
{
	frustumNearPlaneDist_ = parameters->frustum_near_plane_dist;
	frustumFarPlaneDist_ = parameters->frustum_far_plane_dist;
	voxelLeafSize_ = parameters->voxel_leaf_size;
	output_dir_ = parameters->output_dir;
	disp_calc_method_ = parameters->disp_calc_method;
	max_distance_ = parameters->max_distance;
	stereoscan_threshold_ = parameters->stereoscan_threshold;
	fusion_heuristic_ = parameters->fusion_heuristic;
	local_area_size_ = parameters->local_area_size;
	libelas_ipol_gap_ = parameters->libelas_ipol_gap;
	add_corners_ = parameters->add_corners;
	sigma_ = parameters->sigma;
	refinement_linear_threshold_ = parameters->refinement_linear_threshold;
	refinement_angular_threshold_ = parameters->refinement_angular_threshold;

    fs::path output_path{output_dir_};
    output_path = fs::absolute(output_path);

    if (!fs::is_directory(output_path)) {
        ROS_ERROR_STREAM("DENSE: output dir " << output_path << " not found!");
        abort();
    }

    ROS_INFO_STREAM("DENSE: output dir " << output_path);

    log_file_ = fopen(LOG_FILENAME, "w");
    if (!log_file_) {
        ROS_ERROR_STREAM("DENSE: failed to open log file " << LOG_FILENAME);
        abort();
    }

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
