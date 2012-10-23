/*
 * pr2_joint_trajectory_client.h
 *
 *  Created on: 27.09.2012
 *      Author: Tien Thanh Nguyen
 */

#ifndef FOLLOW_JOINT_TRAJECTORY_CLIENT_H_
#define FOLLOW_JOINT_TRAJECTORY_CLIENT_H_

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace crops_arm_tutorials
{

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class Pr2JointTrajectoryClient
{
public:
  Pr2JointTrajectoryClient();
  virtual ~Pr2JointTrajectoryClient();

  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal);
  pr2_controllers_msgs::JointTrajectoryGoal makeArmUpTrajectory();
  actionlib::SimpleClientGoalState getState();

private:
  ros::NodeHandle nh_;
  TrajClient traj_client_;
  ros::Subscriber joint_state_sub_;
  std::vector<std::string> joint_names_;
  bool got_joint_state_;
  std::vector<double> current_joint_state_;
  ros::AsyncSpinner spinner_;

  void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
  trajectory_msgs::JointTrajectory filterJointTrajectory(const trajectory_msgs::JointTrajectory &input);
};

} /* namespace crops_arm_tutorials */
#endif /* FOLLOW_JOINT_TRAJECTORY_CLIENT_H_ */
