#include <ros/ros.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <planning_environment/models/collision_models.h>
#include <ros/time.h>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/package.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <std_srvs/Empty.h>
#include <planning_environment/models/model_utils.h>

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char **argv){
  ros::init (argc, argv, "get_fk");
  ros::NodeHandle rh;

  planning_environment::CollisionModels* collisionModels;
  planning_models::KinematicState* kinematicState;

  collisionModels = new planning_environment::CollisionModels("robot_description");
  kinematicState = new planning_models::KinematicState(collisionModels->getKinematicModel());

  ros::AsyncSpinner spinner(1); 
  spinner.start();

  ros::service::waitForService("crops_wp6_arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("crops_wp6_arm_kinematics/get_constraint_aware_ik");
  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);

  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("crops_wp6_arm_kinematics/get_ik_solver_info");
  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("crops_wp6_arm_kinematics/get_constraint_aware_ik");
  ros::ServiceClient set_planning_scene_diff_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
  ROS_INFO("Waiting for planning scene service");
  set_planning_scene_diff_client.waitForExistence();
  ROS_INFO("Planning scene service is now available");

  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
  planning_environment::convertKinematicStateToRobotState(*kinematicState, ros::Time::now(), collisionModels->getWorldFrameId(), planning_scene_req.planning_scene_diff.robot_state);

  if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
    return -1;
  }

  collisionModels->revertPlanningScene(kinematicState);
  collisionModels->setPlanningScene(planning_scene_res.planning_scene);

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(-1);
  }

  // define the service messages
  kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;

  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "body9";

  gpik_req.ik_request.pose_stamped.header.frame_id = "base_link";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.35;
  gpik_req.ik_request.pose_stamped.pose.position.y = 0.788;
  gpik_req.ik_request.pose_stamped.pose.position.z = 1.0;

  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0;

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
    {
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
      {
        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
      } 
    }
    else
    {
      ROS_ERROR("Inverse kinematics failed");
    }
  }
  else
  {
    ROS_ERROR("Inverse kinematics service call failed");
  }
  ros::shutdown();
}
