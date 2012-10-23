/*
 * pr2_joint_trajectory_client.cpp
 *
 *  Created on: 27.09.2012
 *      Author: Tien Thanh Nguyen
 */

#include <crops_arm_tutorials/pr2_joint_trajectory_client.h>

namespace crops_arm_tutorials
{

Pr2JointTrajectoryClient::Pr2JointTrajectoryClient() :
    traj_client_("/crops_arm_controller/joint_trajectory_action", true), got_joint_state_(false), spinner_(1)
{
  joint_names_.push_back("joint1");
  joint_names_.push_back("joint2");
  joint_names_.push_back("joint3");
  joint_names_.push_back("joint4");
  joint_names_.push_back("joint5");
  joint_names_.push_back("joint6");
  joint_names_.push_back("joint7");
  joint_names_.push_back("joint8");
  joint_names_.push_back("joint9");

  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &Pr2JointTrajectoryClient::jointStateCB, this);
  spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server");
  }
}

Pr2JointTrajectoryClient::~Pr2JointTrajectoryClient()
{
}

void Pr2JointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void Pr2JointTrajectoryClient::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  traj_client_.sendGoal(goal);
}

pr2_controllers_msgs::JointTrajectoryGoal Pr2JointTrajectoryClient::makeArmUpTrajectory()
{
  const size_t NUM_TRAJ_POINTS = 4;
  const size_t NUM_JOINTS = 9;

  // positions after calibration
  std::vector<double> calibration_positions(NUM_JOINTS);
  calibration_positions[0] = 0.00;
  calibration_positions[1] = 0.00;
  calibration_positions[2] = 0.00;
  calibration_positions[3] = 0.00;
  calibration_positions[4] = 0.00;
  calibration_positions[5] = 0.00;
  calibration_positions[6] = 0.00;
  calibration_positions[7] = 0.00;
  calibration_positions[8] = 0.00;

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = 0.2;
  straight_up_positions[1] = 1.2;
  straight_up_positions[2] = 0.2;
  straight_up_positions[3] = 0.2;
  straight_up_positions[4] = 0.2;
  straight_up_positions[5] = 0.2;
  straight_up_positions[6] = 0.2;
  straight_up_positions[7] = 0.2;
  straight_up_positions[8] = 0.2;

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(ind);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(ind);
  trajectory.points[ind].positions = calibration_positions;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(ind);
  trajectory.points[ind].positions = straight_up_positions;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(ind);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = calibration_positions;

  //  // all Velocities 0
  //  for (size_t i = 0; i < NUM_TRAJ_POINTS; ++i)
  //  {
  //    trajectory.points[i].velocities.resize(NUM_JOINTS);
  //    for (size_t j = 0; j < NUM_JOINTS; ++j)
  //    {
  //      trajectory.points[i].velocities[j] = 0.0;
  //    }
  //  }

  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory = filterJointTrajectory(trajectory);
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState Pr2JointTrajectoryClient::getState()
{
  return traj_client_.getState();
}

trajectory_msgs::JointTrajectory Pr2JointTrajectoryClient::filterJointTrajectory(
    const trajectory_msgs::JointTrajectory &input)
{
  ros::service::waitForService("trajectory_filter/filter_trajectory");
  arm_navigation_msgs::FilterJointTrajectory::Request req;
  arm_navigation_msgs::FilterJointTrajectory::Response res;
  ros::ServiceClient filter_trajectory_client_ = nh_.serviceClient<arm_navigation_msgs::FilterJointTrajectory>(
      "trajectory_filter/filter_trajectory");

  req.trajectory = input;
  req.allowed_time = ros::Duration(1.0);

  if (filter_trajectory_client_.call(req, res))
  {
    if (res.error_code.val == res.error_code.SUCCESS)
      ROS_INFO("Requested trajectory was filtered");
    else
      ROS_WARN("Requested trajectory was not filtered. Error code: %d", res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to filter trajectory failed %s", filter_trajectory_client_.getService().c_str());
  }

  return res.trajectory;
}

} /* namespace crops_arm_tutorials */

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "pr2_joint_trajectory_client");

  crops_arm_tutorials::Pr2JointTrajectoryClient arm;
  // Start the trajectory
  arm.startTrajectory(arm.makeArmUpTrajectory());
  // Wait for trajectory completion
  while (!arm.getState().isDone() && ros::ok())
  {
    //usleep(50000);
    ros::spin();
  }
}
