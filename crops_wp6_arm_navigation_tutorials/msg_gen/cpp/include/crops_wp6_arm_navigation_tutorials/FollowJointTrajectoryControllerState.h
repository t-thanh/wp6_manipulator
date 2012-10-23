/* Auto-generated by genmsg_cpp for file /home/tienthanh/workspace/ros/fuerte/mebios/tienthanh/crops_wp6_manipulator/crops_wp6_arm_navigation_tutorials/msg/FollowJointTrajectoryControllerState.msg */
#ifndef CROPS_WP6_ARM_NAVIGATION_TUTORIALS_MESSAGE_FOLLOWJOINTTRAJECTORYCONTROLLERSTATE_H
#define CROPS_WP6_ARM_NAVIGATION_TUTORIALS_MESSAGE_FOLLOWJOINTTRAJECTORYCONTROLLERSTATE_H
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

#include "std_msgs/Header.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace crops_wp6_arm_navigation_tutorials
{
template <class ContainerAllocator>
struct FollowJointTrajectoryControllerState_ {
  typedef FollowJointTrajectoryControllerState_<ContainerAllocator> Type;

  FollowJointTrajectoryControllerState_()
  : header()
  , joint_names()
  , desired()
  , actual()
  , error()
  {
  }

  FollowJointTrajectoryControllerState_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , joint_names(_alloc)
  , desired(_alloc)
  , actual(_alloc)
  , error(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_names_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  joint_names;

  typedef  ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>  _desired_type;
   ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>  desired;

  typedef  ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>  _actual_type;
   ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>  actual;

  typedef  ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>  _error_type;
   ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator>  error;


  typedef boost::shared_ptr< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct FollowJointTrajectoryControllerState
typedef  ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<std::allocator<void> > FollowJointTrajectoryControllerState;

typedef boost::shared_ptr< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState> FollowJointTrajectoryControllerStatePtr;
typedef boost::shared_ptr< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState const> FollowJointTrajectoryControllerStateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace crops_wp6_arm_navigation_tutorials

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b11d532a92ee589417fdd76559eb1d9e";
  }

  static const char* value(const  ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb11d532a92ee5894ULL;
  static const uint64_t static_value2 = 0x17fdd76559eb1d9eULL;
};

template<class ContainerAllocator>
struct DataType< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "crops_wp6_arm_navigation_tutorials/FollowJointTrajectoryControllerState";
  }

  static const char* value(const  ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
string[] joint_names\n\
trajectory_msgs/JointTrajectoryPoint desired\n\
trajectory_msgs/JointTrajectoryPoint actual\n\
trajectory_msgs/JointTrajectoryPoint error  # Redundant, but useful\n\
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
================================================================================\n\
MSG: trajectory_msgs/JointTrajectoryPoint\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
duration time_from_start\n\
";
  }

  static const char* value(const  ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.joint_names);
    stream.next(m.desired);
    stream.next(m.actual);
    stream.next(m.error);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct FollowJointTrajectoryControllerState_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::crops_wp6_arm_navigation_tutorials::FollowJointTrajectoryControllerState_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "joint_names[]" << std::endl;
    for (size_t i = 0; i < v.joint_names.size(); ++i)
    {
      s << indent << "  joint_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_names[i]);
    }
    s << indent << "desired: ";
s << std::endl;
    Printer< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.desired);
    s << indent << "actual: ";
s << std::endl;
    Printer< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.actual);
    s << indent << "error: ";
s << std::endl;
    Printer< ::trajectory_msgs::JointTrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.error);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CROPS_WP6_ARM_NAVIGATION_TUTORIALS_MESSAGE_FOLLOWJOINTTRAJECTORYCONTROLLERSTATE_H

