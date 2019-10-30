struct Point
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

struct GeometryMsgsPose //geometry_msgs::Pose
{
  Point position;
  Orientation orientation;
};

struct TfPose //tf::Pose
{
    /* data */
};


struct TfTransform //tf::Transform
{
    /* data */
};

void poseMsgToTF(GeometryMsgsPose ros_pose, TfPose tf_pose); //tf::poseMsgToTF
void poseTFToMsg(TfPose tf_pose, GeometryMsgsPose ros_pose); //tf::poseTFToMsg
void pointTFToMsg(Point point, Point point_msg); //tf::pointTFToMsg
Orientation createQuaternionMsgFromYaw(double theta); //tf::createQuaternionMsgFromYaw

