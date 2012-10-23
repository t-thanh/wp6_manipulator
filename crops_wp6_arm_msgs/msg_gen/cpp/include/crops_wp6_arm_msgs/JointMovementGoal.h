/* Auto-generated by genmsg_cpp for file /home/tienthanh/workspace/ros/fuerte/mebios/tienthanh/crops_wp6_manipulator/crops_wp6_arm_msgs/msg/JointMovementGoal.msg */
#ifndef CROPS_WP6_ARM_MSGS_MESSAGE_JOINTMOVEMENTGOAL_H
#define CROPS_WP6_ARM_MSGS_MESSAGE_JOINTMOVEMENTGOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "sensor_msgs/JointState.h"

namespace crops_wp6_arm_msgs
{
template <class ContainerAllocator>
struct JointMovementGoal_ {
  typedef JointMovementGoal_<ContainerAllocator> Type;

  JointMovementGoal_()
  : jointGoal()
  {
  }

  JointMovementGoal_(const ContainerAllocator& _alloc)
  : jointGoal(_alloc)
  {
  }

  typedef  ::sensor_msgs::JointState_<ContainerAllocator>  _jointGoal_type;
   ::sensor_msgs::JointState_<ContainerAllocator>  jointGoal;


  typedef boost::shared_ptr< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct JointMovementGoal
typedef  ::crops_wp6_arm_msgs::JointMovementGoal_<std::allocator<void> > JointMovementGoal;

typedef boost::shared_ptr< ::crops_wp6_arm_msgs::JointMovementGoal> JointMovementGoalPtr;
typedef boost::shared_ptr< ::crops_wp6_arm_msgs::JointMovementGoal const> JointMovementGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace crops_wp6_arm_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "03f8456b346613dcdf3d0e35b6b1a281";
  }

  static const char* value(const  ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x03f8456b346613dcULL;
  static const uint64_t static_value2 = 0xdf3d0e35b6b1a281ULL;
};

template<class ContainerAllocator>
struct DataType< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "crops_wp6_arm_msgs/JointMovementGoal";
  }

  static const char* value(const  ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
sensor_msgs/JointState jointGoal\n\
\n\
================================================================================\n\
MSG: sensor_msgs/JointState\n\
# This is a message that holds data to describe the state of a set of torque controlled joints. \n\
#\n\
# The state of each joint (revolute or prismatic) is defined by:\n\
#  * the position of the joint (rad or m),\n\
#  * the velocity of the joint (rad/s or m/s) and \n\
#  * the effort that is applied in the joint (Nm or N).\n\
#\n\
# Each joint is uniquely identified by its name\n\
# The header specifies the time at which the joint states were recorded. All the joint states\n\
# in one message have to be recorded at the same time.\n\
#\n\
# This message consists of a multiple arrays, one for each part of the joint state. \n\
# The goal is to make each of the fields optional. When e.g. your joints have no\n\
# effort associated with them, you can leave the effort array empty. \n\
#\n\
# All arrays in this message should have the same size, or be empty.\n\
# This is the only way to uniquely associate the joint name with the correct\n\
# states.\n\
\n\
\n\
Header header\n\
\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.jointGoal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct JointMovementGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::crops_wp6_arm_msgs::JointMovementGoal_<ContainerAllocator> & v) 
  {
    s << indent << "jointGoal: ";
s << std::endl;
    Printer< ::sensor_msgs::JointState_<ContainerAllocator> >::stream(s, indent + "  ", v.jointGoal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CROPS_WP6_ARM_MSGS_MESSAGE_JOINTMOVEMENTGOAL_H

