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
class btVector3
{

  public:
  /**@brief Constructor from scalars 
   * @param x X value
   * @param y Y value 
   * @param z Z value 
   */
	SIMD_FORCE_INLINE btVector3(const btScalar& _x, const btScalar& _y, const btScalar& _z)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = btScalar(0.f);
	}
  public:
  /**@brief Return the x value */
		SIMD_FORCE_INLINE const btScalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
		SIMD_FORCE_INLINE const btScalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
		SIMD_FORCE_INLINE const btScalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
		SIMD_FORCE_INLINE const btScalar& w() const { return m_floats[3]; }
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
class btMatrix3x3
{
  public:
  void getRotation(btQuaternion& q) const
	{
#if (defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE))|| defined (BT_USE_NEON)
        btScalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();
        btScalar s, x;
        
        union {
            btSimdFloat4 vec;
            btScalar f[4];
        } temp;
        
        if (trace > btScalar(0.0)) 
        {
            x = trace + btScalar(1.0);

            temp.f[0]=m_el[2].y() - m_el[1].z();
            temp.f[1]=m_el[0].z() - m_el[2].x();
            temp.f[2]=m_el[1].x() - m_el[0].y();
            temp.f[3]=x;
            //temp.f[3]= s * btScalar(0.5);
        } 
        else 
        {
            int i, j, k;
            if(m_el[0].x() < m_el[1].y()) 
            { 
                if( m_el[1].y() < m_el[2].z() )
                    { i = 2; j = 0; k = 1; }
                else
                    { i = 1; j = 2; k = 0; }
            }
            else
            {
                if( m_el[0].x() < m_el[2].z())
                    { i = 2; j = 0; k = 1; }
                else
                    { i = 0; j = 1; k = 2; }
            }

            x = m_el[i][i] - m_el[j][j] - m_el[k][k] + btScalar(1.0);

            temp.f[3] = (m_el[k][j] - m_el[j][k]);
            temp.f[j] = (m_el[j][i] + m_el[i][j]);
            temp.f[k] = (m_el[k][i] + m_el[i][k]);
            temp.f[i] = x;
            //temp.f[i] = s * btScalar(0.5);
        }

        s = btSqrt(x);
        q.set128(temp.vec);
        s = btScalar(0.5) / s;

        q *= s;
#else    
		btScalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();

		btScalar temp[4];

		if (trace > btScalar(0.0)) 
		{
			btScalar s = btSqrt(trace + btScalar(1.0));
			temp[3]=(s * btScalar(0.5));
			s = btScalar(0.5) / s;

			temp[0]=((m_el[2].y() - m_el[1].z()) * s);
			temp[1]=((m_el[0].z() - m_el[2].x()) * s);
			temp[2]=((m_el[1].x() - m_el[0].y()) * s);
		} 
		else 
		{
			int i = m_el[0].x() < m_el[1].y() ? 
				(m_el[1].y() < m_el[2].z() ? 2 : 1) :
				(m_el[0].x() < m_el[2].z() ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;

			btScalar s = btSqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + btScalar(1.0));
			temp[i] = s * btScalar(0.5);
			s = btScalar(0.5) / s;

			temp[3] = (m_el[k][j] - m_el[j][k]) * s;
			temp[j] = (m_el[j][i] + m_el[i][j]) * s;
			temp[k] = (m_el[k][i] + m_el[i][k]) * s;
		}
		q.setValue(temp[0],temp[1],temp[2],temp[3]);
#endif
	}
}

class btTransform
{
  btMatrix3x3 m_basis;
  btVector3 m_origin;

  public:
    
    SIMD_FORCE_INLINE btVector3& getOrigin() {return m_origin;}
    /**@brief Return a quaternion representing the rotation */
    btQuaternion getRotation() const { 
      btQuaternion q;
      m_basis.getRotation(q);
      return q;
    }
};

class btQuaternion
{
  public:
    btQuaternion();
    ~btQuaternion();
  public:
    btScalar length2() const
    {
      return dot(*this);
    }

    /**@brief Normalize the quaternion 
   * Such that x^2 + y^2 + z^2 +w^2 = 1 */
    btQuaternion& normalize() 
    {
      #if defined (BT_USE_SSE_IN_API) && defined (BT_USE_SSE)
          __m128	vd;
          
          vd = _mm_mul_ps(mVec128, mVec128);
          
              __m128 t = _mm_movehl_ps(vd, vd);
          vd = _mm_add_ps(vd, t);
          t = _mm_shuffle_ps(vd, vd, 0x55);
          vd = _mm_add_ss(vd, t);

          vd = _mm_sqrt_ss(vd);
          vd = _mm_div_ss(vOnes, vd);
              vd = bt_pshufd_ps(vd, 0); // splat
          mVec128 = _mm_mul_ps(mVec128, vd);
          
          return *this;
      #else    
          return *this /= length();
      #endif
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

//tf::createQuaternionMsgFromYaw
inline GeometryMsgsQuaternion createQuaternionMsgFromYaw(double yaw)  
{
    Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    GeometryMsgsQuaternion q_msg;
    quaternionTFToMsg(q, q_msg);
    return q_msg;
};


