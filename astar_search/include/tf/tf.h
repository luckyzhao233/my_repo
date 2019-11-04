#ifndef TF.H
#define TF.H

#include <string>
#include <vector>
#include <map>

#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/MinMax.h"
#include "tf/LinearMath/QuadWord.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Scalar.h"
#include "tf/LinearMath/Transform.h"
#include "tf/LinearMath/Vector3.h"
#include "geometry_msgs/geometry_msgs.h"
namespace tf
{

    typedef tf::Vector3 Point;
    typedef tf::Transform Pose;

    static const double QUATERNION_TOLERANCE = 0.1f;

    //tf::pointTFToMsg
    static inline void pointTFToMsg(const Point& bt_v, geometry_msgs::Point& msg_v)
    {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};

    //tf::quaternionTFtoMsg
    static inline void quaternionTFToMsg(const Quaternion& bt, geometry_msgs::Quaternion& msg) 
    {
        if (fabs(bt.length2() - 1 ) > QUATERNION_TOLERANCE) 
        {
            //ROS_WARN("TF to MSG: Quaternion Not Properly Normalized");
            Quaternion bt_temp = bt; 
            bt_temp.normalize();
            msg.x = bt_temp.x(); msg.y = bt_temp.y(); msg.z = bt_temp.z();  msg.w = bt_temp.w();
        }
        else
        {
        msg.x = bt.x(); msg.y = bt.y(); msg.z = bt.z();  msg.w = bt.w();
        }
    };

    //tf::poseTFToMsg
    static inline void poseTFToMsg(Pose bt, geometry_msgs::Pose msg) 
    {pointTFToMsg(bt.getOrigin(), msg.position);  quaternionTFToMsg(bt.getRotation(), msg.orientation);};

    //tf::poseMsgToTF
    static inline void poseMsgToTF(const geometry_msgs::Pose& msg, Pose& bt)
    {bt = Transform(Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), Vector3(msg.position.x, msg.position.y, msg.position.z));};

    //tf::createQuaternionMsgFromYaw
    inline geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw)  
    {
        Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        geometry_msgs::Quaternion q_msg;
        quaternionTFToMsg(q, q_msg);
        return q_msg;
    };
}


#endif