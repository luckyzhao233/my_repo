#ifndef NAV_MSGS.H
#define NAV_MSGS.H

#include <string>
#include "geometry_msgs/geometry_msgs.h"
#include <AutoKernel/ak.h>
namespace nav_msgs
{
    struct Header //std_msgs/Header
    {
        Header() : seq(0), stamp(), frame_id()
        {}
        uint32_t seq;
        AK::TTime stamp; //ros::time
        std::string frame_id;
    };
    struct MapMetaData //nav_msgs::MapMetaData
    {
        MapMetaData() : map_load_time(), resolution(0.0), width(0), height(0), origin()
        {}
        AK::TTime map_load_time;
        float resolution;
        uint32_t width;
        uint32_t height;
        geometry_msgs::Pose origin;
    };

    struct OccupancyGrid //nav_msgs::OccupancyGrid
    {
        Header header;
        MapMetaData info;
        std::vector<int8_t> data;
    };

    struct Path //nav_msgs::Path
    {
        Header header;
        geometry_msgs::PoseStamped[] poses;
    };    
}

#endif // NAV_MSGS.H