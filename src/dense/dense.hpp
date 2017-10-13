#ifndef DENSE_H
#define DENSE_H

#include "../dense/Camera.hpp"
#include "../dense/DisparityCalcThread.hpp"
#include "../dense/ProjectionThread.hpp"
#include "../dense/RefinementThread.hpp"
#include "../dense/ImageQueue.hpp"

#define PIXEL_DEPTH_INVALID			-1

struct dense_parameters {
	std::string base_frame;
	std::string camera_frame;
	std::string map_frame;
	double frustum_near_plane_dist;
	double frustum_far_plane_dist;
	double voxel_leaf_size;
	double max_distance;
	double stereoscan_threshold;
	std::string disp_calc_method;
	std::string fusion_heuristic;
	int local_area_size;
	int libelas_ipol_gap;
	bool add_corners;
	std::string output_dir;
	double refinement_linear_threshold;
	double refinement_angular_threshold;
	bool use_approx_sync;
};

class Dense
{
public:
	Dense(const sensor_msgs::CameraInfoConstPtr& left_info,
		  const sensor_msgs::CameraInfoConstPtr& right_info,
		  struct dense_parameters *parameters);
	~Dense();

	struct dense_parameters parameters;

	void WriteToLog(const char* fmt, ...);

	const sensor_msgs::CameraInfoConstPtr left_info_, right_info_;

	Camera *camera_;
	ImageQueue *raw_image_pairs_;
	DispImageQueue  *disp_images_;
	PointCloudQueue *point_clouds_;

	ProjectionThread *projectionThread_;

private:
	DisparityCalcThread *disparityCalcThread;
	RefinementThread *refinementThread_;

	char log_filename_[512];
	FILE *log_file_;
};

typedef boost::shared_ptr<Dense> DensePtr;

#endif /* DENSE_H */
