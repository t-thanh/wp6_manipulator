#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
//
#include <planning_environment/models/collision_models.h>
#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/package.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <std_srvs/Empty.h>
#include <planning_environment/models/model_utils.h>
//
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;
//
  planning_environment::CollisionModels* collisionModels;
  planning_models::KinematicState* kinematicState;

  collisionModels = new planning_environment::CollisionModels("robot_description");
  kinematicState = new planning_models::KinematicState(collisionModels->getKinematicModel());

  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ROS_INFO("Waiting for planning scene service");
  ros::ServiceClient set_planning_scene_diff_client = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

  set_planning_scene_diff_client.waitForExistence();
  ROS_INFO("Planning scene service is now available");

  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
  planning_environment::convertKinematicStateToRobotState(*kinematicState, ros::Time::now(), collisionModels->getWorldFrameId(), planning_scene_req.planning_scene_diff.robot_state);
  
  if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
  }
  ROS_INFO("Successfully set planning scene");

  collisionModels->revertPlanningScene(kinematicState);
  collisionModels->setPlanningScene(planning_scene_res.planning_scene);
//
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  arm_navigation_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "base_link";
  desired_pose.link_name = "body9";
  desired_pose.pose.position.x = 0.5;
  desired_pose.pose.position.y = 1.188;
  desired_pose.pose.position.z = 1.3;

  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
