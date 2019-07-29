#ifndef ROS_NODE
#define ROS_NODE

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"         // du msg
#include "geometry_msgs/Vector3Stamped.h"       // encoder msg
#include "geometry_msgs/QuaternionStamped.h"    // Quaternion msg

struct Quat{
  float x;
  float y;
  float z;
  float w;
};

class RosNode
{

protected:
  int _queue_size;

  ros::NodeHandle _nh;
  ros::Publisher _pub_enc;    // publish imu encoder message
  ros::Publisher _pub_du;     // publish imu duty cycle message
  ros::Subscriber _sub_du;    // subscriber to desired duty cycle message from user

  ros::Time _time;
  std::string _name;

public:
  float _enc[3];
  float _cmd_du[4];

  RosNode();
  RosNode(ros::NodeHandle nh, std::string name);
  ~RosNode();
  void publishAllMsgs(const float enc[3], const float du[4]);
  void publishEncMsg(const float enc[3]);
  void publishDuMsg(const float du[3]);

  void cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
};

#endif // ROS_NODE


