/*
 * MeBioS-ROS packages - Robot Operating System code by the KUL
 * Copyright (C) 2012  KUL
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * joint_movement_action_controller.h
 *
 *  Created on: 28.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#ifndef JOINT_MOVEMENT_ACTION_CONTROLLER_H__
#define JOINT_MOVEMENT_ACTION_CONTROLLER_H__

#include <vector>

#include <ros/node_handle.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <AbstractCrops.h>
#include <sensor_msgs/JointState.h>
#include <crops_arm_msgs/JointMovementAction.h>

#include <arm_navigation_msgs/JointLimits.h>

namespace crops
{

class JointMovementActionController
{
  typedef actionlib::SimpleActionServer<crops_arm_msgs::JointMovementAction> JMAS;
  typedef actionlib::SimpleActionClient<crops_arm_msgs::JointMovementAction> JMAC;

public:
  JointMovementActionController(boost::shared_ptr<AbstractCrops> crops);
  virtual ~JointMovementActionController();

private:
  // robot and joint state
  std::vector<std::string> joints_; // controlled joints, same order as expected by the KNI (index 0 = first motor, ...)
  //std::vector<std::string> gripper_joints_;
  boost::shared_ptr<AbstractCrops> crops_;

  sensor_msgs::JointState movement_goal_;

  // action server
  void executeCB(const JMAS::GoalConstPtr &goal);
  JMAS action_server_;

  bool suitableJointGoal(const std::vector<std::string>& jointGoalNames);

  sensor_msgs::JointState adjustJointGoalPositionsToMotorLimits(const sensor_msgs::JointState &jointGoal);

};
}

#endif /* JOINT_MOVEMENT_ACTION_CONTROLLER_H_ */
