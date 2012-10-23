#include <ros/ros.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
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

int main(int argc, char **argv){
  ros::init (argc, argv, "get_state_validity_test");
  ros::NodeHandle rh;

  planning_environment::CollisionModels* collisionModels;
  planning_models::KinematicState* kinematicState;

  collisionModels = new planning_environment::CollisionModels("robot_description");
  kinematicState = new planning_models::KinematicState(collisionModels->getKinematicModel());

  ros::AsyncSpinner spinner(1); 
  spinner.start();

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  vis_marker_publisher_ = rh.advertise<visualization_msgs::Marker>("state_validity_markers", 128);
  vis_marker_array_publisher_ = rh.advertise<visualization_msgs::MarkerArray>("state_validity_markers_array", 128);

  ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME);
  ros::ServiceClient get_planning_scene_client = 
    rh.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

  arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
  arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

  if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) {
    ROS_WARN("Can't get planning scene");
    return -1;
  }

  //planning_environment::CollisionModels collision_models("robot_description");
  //planning_models::KinematicState* state = 
  //  collision_models.setPlanningScene(planning_scene_res.planning_scene);
  
  planning_environment::convertKinematicStateToRobotState(*kinematicState, ros::Time::now(), collisionModels->getWorldFrameId(), planning_scene_req.planning_scene_diff.robot_state);
  
  std::vector<std::string> arm_names = 
    collisionModels->getKinematicModel()->getModelGroup("arm")->getUpdatedLinkModelNames();
  std::vector<std::string> joint_names = 
    collisionModels->getKinematicModel()->getModelGroup("arm")->getJointModelNames();

  if(argc > 1) {
    std::stringstream s(argv[1]);
    double val;
    s >> val;
    std::map<std::string, double> nvalues;
    nvalues["body1"] = val;
    
    kinematicState->setKinematicState(nvalues);
  }
  
  std_msgs::ColorRGBA good_color, collision_color, joint_limits_color;
  good_color.a = collision_color.a = joint_limits_color.a = .8;

  good_color.g = 1.0;
  collision_color.r = 1.0;
  joint_limits_color.b = 1.0;
  
  std_msgs::ColorRGBA point_markers;
  point_markers.a = 1.0;
  point_markers.r = 1.0;
  point_markers.g = .8;

  std_msgs::ColorRGBA color;
  visualization_msgs::MarkerArray arr;
  if(!kinematicState->areJointsWithinBounds(joint_names)) {
    color = joint_limits_color;
  } else if(collisionModels->isKinematicStateInCollision(*kinematicState)) {
    color = collision_color;
    collisionModels->getAllCollisionPointMarkers(*kinematicState,
                                                 arr,
                                                 point_markers,
                                                 ros::Duration(0.2));
  } else {
    color = good_color;
  }

  collisionModels->getRobotMarkersGivenState(*kinematicState,
                                             arr,
                                             color,
                                             "arm",
                                             ros::Duration(0.2),
                                             &arm_names);

  while(ros::ok()) {    
    vis_marker_array_publisher_.publish(arr);
    ros::spinOnce();
    ros::Duration(.1).sleep();
  }
  collisionModels->revertPlanningScene(kinematicState);
  ros::shutdown();
}
