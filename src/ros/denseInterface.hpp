#ifndef DENSEINTERFACE_H
#define DENSEINTERFACE_H

#include <ros/ros.h>

namespace dense
{

    class denseInterface
    {
        public:

        denseInterface(ros::NodeHandle& nh, ros::NodeHandle& nhp);
        ~denseInterface();
    };

}
#endif /* DENSEINTERFACE_H */
