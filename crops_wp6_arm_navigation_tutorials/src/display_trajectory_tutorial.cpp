#include <arm_navigation_msgs/DisplayTrajectory.h>
//#include <planning_environment/monitors/joint_state_monitor.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "display_trajectory_publisher");
  boost::thread spin_thread(&spinThread);
  ros::NodeHandle root_handle;
  //planning_environment::JointStateMonitor joint_state_monitor;
  ros::Publisher display_trajectory_publisher = root_handle.advertise<arm_navigation_msgs::DisplayTrajectory>("joint_path_display", 1);
  while(display_trajectory_publisher.getNumSubscribers() < 1 && root_handle.ok())
  {
    ROS_INFO("Waiting for subscriber");
    ros::Duration(0.1).sleep();
  }
  arm_navigation_msgs::DisplayTrajectory display_trajectory;
  unsigned int num_points = 100;

  display_trajectory.model_id = "arm";
  display_trajectory.trajectory.joint_trajectory.header.frame_id = "base_link";
  display_trajectory.trajectory.joint_trajectory.header.stamp = ros::Time::now();
  display_trajectory.trajectory.joint_trajectory.joint_names.push_back("body1");
  display_trajectory.trajectory.joint_trajectory.points.resize(num_points);

  for(unsigned int i=0; i < num_points; i++)
  {    
    display_trajectory.trajectory.joint_trajectory.points[i].positions.push_back(-(M_PI*i/4.0)/num_points);
  }
  //display_trajectory.robot_state.joint_state =  joint_state_monitor.getJointStateRealJoints();
  ROS_INFO("Publishing path for display");
  display_trajectory_publisher.publish(display_trajectory);
  //joint_state_monitor.stop();
  ros::shutdown();
  spin_thread.join();
  return(0);
}
