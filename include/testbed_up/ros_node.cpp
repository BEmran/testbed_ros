#include "ros_node.h"

RosNode::RosNode()
{
}

/*****************************************************************************************
RosNode: construct an object for ros node
******************************************************************************************/
RosNode::RosNode(ros::NodeHandle nh ,std::string name){
  _nh = nh;
  _name = name;
  _queue_size = 10;

  _pub_enc = _nh.advertise <geometry_msgs::Vector3Stamped>("testbed/sensors/row/encoders", _queue_size);
  _pub_du  = _nh.advertise <geometry_msgs::TwistStamped>  ("testbed/motors/du"           , _queue_size);

  _sub_du  = _nh.subscribe("testbed/cmd/du"   , _queue_size, &RosNode::cmdDuCallback , this);

  _cmd_du[0] = 0.0;
  _cmd_du[1] = 0.0;
  _cmd_du[2] = 0.0;
  _cmd_du[3] = 0.0;
}

/*****************************************************************************************
publishMsgs: Publish all msg
******************************************************************************************/
void RosNode::publishAllMsgs(const float enc[3], const float du[4]){
  _time = ros::Time::now();
  publishEncMsg(enc);
  publishDuMsg(du);
}

/*****************************************************************************************
publishEncMsg: Publish encoder msgs
******************************************************************************************/
void RosNode::publishEncMsg(const float enc[3]){
  geometry_msgs::Vector3Stamped msg_enc;

  msg_enc.header.stamp = _time;
  msg_enc.header.seq++;
  msg_enc.vector.x = enc[0];
  msg_enc.vector.y = enc[1];
  msg_enc.vector.z = enc[2];
  _pub_enc.publish(msg_enc);
}

/*****************************************************************************************
publishDuMsg: Publish motors' inputs du msgs
******************************************************************************************/
void RosNode::publishDuMsg(const float du[4]){
  geometry_msgs::TwistStamped msg_du;

  msg_du.header.stamp = _time;
  msg_du.header.seq++;
  msg_du.twist.linear.z  = du[0];
  msg_du.twist.angular.x = du[1];
  msg_du.twist.angular.y = du[2];
  msg_du.twist.angular.z = du[3];
  _pub_du.publish(msg_du);
}

/*****************************************************************************************
duCmdCallback: Read command duty cycle
******************************************************************************************/
void RosNode::cmdDuCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _cmd_du[0] = msg->twist.linear.z;
  _cmd_du[1] = msg->twist.angular.x;
  _cmd_du[2] = msg->twist.angular.y;
  _cmd_du[3] = msg->twist.angular.z;

}
