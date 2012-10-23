#include <ros/ros.h>
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

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_scene_node");
  ros::NodeHandle n;

  planning_environment::CollisionModels* collisionModels;
  planning_models::KinematicState* kinematicState;

  collisionModels = new planning_environment::CollisionModels("robot_description");
  kinematicState = new planning_models::KinematicState(collisionModels->getKinematicModel());

  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ROS_INFO("Waiting for planning scene service");
  ros::ServiceClient set_planning_scene_diff_client = n.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

  set_planning_scene_diff_client.waitForExistence();
  ROS_INFO("Planning scene service is now available");

  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
  planning_environment::convertKinematicStateToRobotState(*kinematicState, ros::Time::now(), collisionModels->getWorldFrameId(), planning_scene_req.planning_scene_diff.robot_state);
  
  if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
    return -1;
  }
  ROS_INFO("Successfully set planning scene");

  collisionModels->revertPlanningScene(kinematicState);
  collisionModels->setPlanningScene(planning_scene_res.planning_scene);
  ros::spin();
  return 0;
}
