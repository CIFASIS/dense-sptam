#include "ProjectionThread.hpp"

#include "dense.hpp"

ProjectionThread::ProjectionThread(Dense *dense)
  : dense_(dense)
  , projectionThread_(&ProjectionThread::compute, this)
{}

void ProjectionThread::compute()
{
    while(1) {
        /* Calls to pop() are blocking */
        DispRawImagePtr disp_raw_img = dense_->disp_images_->pop();
        filterDisp(disp_raw_img, MIN_DISPARITY_THRESHOLD);
        MatVec3fPtr points_mat = processPoints(disp_raw_img);
        PointCloudEntry::Ptr entry = dense_->point_clouds_->getEntry(disp_raw_img->first->header.seq);

        entry->lock();
        entry->set_disp_raw_img(disp_raw_img);
        entry->set_points_mat(points_mat);
        dense_->point_clouds_->schedule(entry);
        entry->unlock();

        ROS_INFO("ProjectionThread::computed seq = %u", entry->get_seq());
    }
}

void ProjectionThread::filterDisp(const DispRawImagePtr disp_raw_img, float min_disparity)
{
    ImagePtr raw_left_image = disp_raw_img->first;
    DispImagePtr disp_img = disp_raw_img->second;

    for (unsigned int i = 0; i < raw_left_image->height; i++)
        for (unsigned int j = 0; j < raw_left_image->width; j++)
            if (disp_img->at<float>(i, j) < min_disparity)
                disp_img->at<float>(i, j) = 0;
}


MatVec3fPtr ProjectionThread::processPoints(const DispRawImagePtr disp_raw_img)
{
    DispImagePtr disp_img = disp_raw_img->second;
    MatVec3fPtr dense_points_(new MatVec3f);

    dense_->camera_ ->getStereoModel().projectDisparityImageTo3d(*disp_img, *dense_points_, true);

    return dense_points_;
}
