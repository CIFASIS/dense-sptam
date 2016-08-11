#ifndef DENSEINTERFACE_H
#define DENSEINTERFACE_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Path.h>

namespace dense
{

    class denseInterface
    {
    public:
        denseInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp);
        ~denseInterface();

    private:
        void cb_keyframes_path(const nav_msgs::PathConstPtr& path);

        ros::Subscriber sub_path_;
    };

}
#endif /* DENSEINTERFACE_H */
