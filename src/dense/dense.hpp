#ifndef DENSE_H
#define DENSE_H

#include "../dense/Camera.hpp"
#include "../dense/DisparityCalcThread.hpp"
#include "../dense/ProjectionThread.hpp"
#include "../dense/RefinementThread.hpp"
#include "../dense/ImageQueue.hpp"

class Dense
{
public:

    Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info,
          double frustumNearPlaneDist, double frustumFarPlaneDist, double voxelLeafSize,
          double filter_meanK, double filter_stddev, std::string disp_calc_method,
          double filter_radius, double filter_minneighbours, double min_disparity
    );
    ~Dense();

    const sensor_msgs::CameraInfoConstPtr left_info_, right_info_;
    double frustumNearPlaneDist_, frustumFarPlaneDist_, voxelLeafSize_;
    double filter_meanK_, filter_stddev_;
    std::string disp_calc_method_;
    double filter_radius_, filter_minneighbours_;
    double min_disparity_;

    Camera *camera_;
    ImageQueue *raw_image_pairs_;
    DispImageQueue  *disp_images_;
    PointCloudQueue *point_clouds_;

private:

    DisparityCalcThread *disparityCalcThread;
    ProjectionThread *projectionThread_;
    RefinementThread *refinementThread_;

};

typedef boost::shared_ptr<Dense> DensePtr;

#endif /* DENSE_H */
