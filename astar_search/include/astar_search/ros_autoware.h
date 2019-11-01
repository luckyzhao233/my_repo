#include <math.h>
#include <string>

#if defined(BT_USE_DOUBLE_PRECISION)
  typedef double btScalar;
#else
  typedef float btScalar;
#endif
typedef unsigned char uint8;      /* Unsigned 8 bit quantity  */
typedef signed char int8;         /* Signed 8 bit quantity    */
typedef unsigned short uint16;    /* Unsigned 16 bit quantity */
typedef signed short int16;       /* Signed 16 bit quantity   */
typedef unsigned int uint32;      /* Unsigned 32 bit quantity */
typedef signed int int32;         /* Signed 32 bit quantity   */
typedef float float32;               /* Single precision         */
                                  /* floating point           */
typedef double float64;              /* Double precision         */
                                  /* floating point           */

static const double QUATERNION_TOLERANCE = 0.1f;                                  
struct btVector3
{
  float64 x;
  float64 y;
  float64 z;
};

struct GeometryMsgsPoint
{
  float64 x;
  float64 y;
  float64 z;
};


struct Orientation
{
  float64 x;
  float64 y;
  float64 z;
  float64 w;
};

struct GeometryMsgsQuaternion //geometry_msgs::Quaternion
{
  float64 x;
  float64 y;
  float64 z;
  float64 w;
};


struct GeometryMsgsPose //geometry_msgs::Pose
{
  Point position;
  Orientation orientation;
};

struct Header //std_msgs/Header
{
  Header() : seq(0), stamp(), frame_id()
  {}
  uint32_t seq;
  uint64_t stamp;
  std::string frame_id;
};

struct MapMetaData //nav_msgs/MapMetaData
{
  time map_load_time;
  float32 resolution;
  uint32 width;
  uint32 height;
  GeometryMsgsPose origin;
};

struct OccupancyGrid //nav_msgs::OccupancyGrid
{
  Header header;
  MapMetaData info;
  int8[] data;
};

struct Path //nav_msgs::Path
{
  Header header;
  PoseStamped[] poses;
};

struct PoseStamped //geometry_msgs::PoseStamped
{
  Header header;
  GeometryMsgsPose pose;
};

class btTransform
{
  btMatrix3x3 m_basis;
  btVector3 m_origin;

  public:
    
    SIMD_FORCE_INLINE btVector3& getOrigin() {return m_origin;}
};

class btQuaternion
{
  public:
    btScalar length2() const
    {
      return dot(*this);
    }

    btQuaternion& normalize()
    {

    }
};
typedef btTransform Pose; //tf::Pose
typedef btTransform Transform; //tf::Transform
typedef btQuaternion Quaternion; 
typedef btVector3 Point;

//tf::pointTFToMsg
static inline void pointTFToMsg(const Point& bt_v, GeometryMsgsPoint& msg_v)
{msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};

//tf::quaternionTFtoMsg
static inline void quaternionTFToMsg(const Quaternion& bt, GeometryMsgsQuaternion& msg) 
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
static inline void poseTFToMsg(Pose bt, GeometryMsgsPose msg) 
{pointTFToMsg(bt.getOrigin(), msg.position);  quaternionTFToMsg(bt.getRotation(), msg.orientation);};

//tf::poseMsgToTF
static inline void poseMsgToTF(const GeometryMsgsPose& msg, Pose& bt)
{bt = Transform(Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), Vector3(msg.position.x, msg.position.y, msg.position.z));};


inline GeometryMsgsQuaternion createQuaternionMsgFromYaw(double yaw)  
{
    Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    GeometryMsgsQuaternion q_msg;
    quaternionTFToMsg(q, q_msg);
    return q_msg;
};//tf::createQuaternionMsgFromYaw


