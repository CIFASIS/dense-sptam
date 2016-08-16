#ifndef DENSE_H
#define DENSE_H

#include "../dense/Camera.hpp"
#include "../dense/DisparityCalcThread.hpp"
#include "../dense/ProjectionThread.hpp"
#include "../dense/TransformThread.hpp"
#include "../dense/RefinementThread.hpp"
#include "../dense/ImageQueue.hpp"

class Dense
{
public:

    Dense(const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info);
    ~Dense();

    ImageQueue *raw_left_images, *raw_right_images;
    DispImageQueue  *disp_images;
    PointCloudQueue *point_clouds;

private:

    Camera *camera_;
    DisparityCalcThread *disparityCalcThread_;
    ProjectionThread *projectionThread_;
    TransformThread *transformThread_;
    RefinementThread *refinementThread_;

};

#endif /* DENSE_H */
