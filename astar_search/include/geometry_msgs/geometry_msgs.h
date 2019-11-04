#ifndef GEOMETRY_MSGS.H
#define GEOMETRY_MSGS.H

#include <string>
#include <AutoKernel/ak.h>

namespace geometry_msgs
{
    struct Header //std_msgs::Header
    {
        Header() : seq(0), stamp(), frame_id()
        {}
        uint32_t seq;
        AK::TTime stamp; //ros::time
        std::string frame_id;
    };

    struct Point
    {
        Point() : x(0.0), y(0.0), z(0.0) {}
        double x;
        double y;
        double z;
    };

    struct Orientation
    {
        double x;
        double y;
        double z;
        double w;
    };
    struct Quaternion //geometry_msgs::Quaternion
    {
        double x;
        double y;
        double z;
        double w;
    };


    struct Pose //geometry_msgs::Pose
    {
        Point position;
        Orientation orientation;
    };

    struct PoseStamped //geometry_msgs::PoseStamped
    {
        Header header;
        Pose pose;
    };
}

#endif //GEOMETRY_MSGS.H