#ifndef DENSE_H
#define DENSE_H

#include "../dense/Camera.hpp"
#include "../dense/DisparityCalcThread.hpp"
#include "../dense/ProjectionThread.hpp"
#include "../dense/RefinementThread.hpp"
#include "../dense/ImageQueue.hpp"

#define PIXEL_DEPTH_INVALID     -1


class Dense
{
public:

    Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info,
          double frustumNearPlaneDist, double frustumFarPlaneDist, double voxelLeafSize,
          std::string output_dir, std::string disp_calc_method, double max_distance,
          double stereoscan_threshold, std::string fusion_heuristic, int local_area_size, int libelas_ipol_gap,
          bool add_corners, double sigma, double refinement_linear_threshold, double refinement_angular_threshold
    );
    ~Dense();

    void WriteToLog(const char* fmt, ...);

    const sensor_msgs::CameraInfoConstPtr left_info_, right_info_;
    double frustumNearPlaneDist_, frustumFarPlaneDist_, voxelLeafSize_;
    std::string output_dir_, disp_calc_method_;
    double max_distance_, stereoscan_threshold_;
    std::string fusion_heuristic_;
    int local_area_size_;
    int libelas_ipol_gap_;
    bool add_corners_;
    double sigma_;
    double refinement_linear_threshold_, refinement_angular_threshold_;

    Camera *camera_;
    ImageQueue *raw_image_pairs_;
    DispImageQueue  *disp_images_;
    PointCloudQueue *point_clouds_;

    ProjectionThread *projectionThread_;

private:

    DisparityCalcThread *disparityCalcThread;

    RefinementThread *refinementThread_;

    FILE *log_file_;
};

typedef boost::shared_ptr<Dense> DensePtr;

#endif /* DENSE_H */
