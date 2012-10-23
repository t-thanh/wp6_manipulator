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
 * AbstractCrops.cpp
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#include <AbstractCrops.h>

namespace crops
{

AbstractCrops::AbstractCrops()
{
  joint_names_.resize(NUM_JOINTS);
  joint_types_.resize(NUM_JOINTS);

  motor_angles_.resize(NUM_MOTORS);
  motor_velocities_.resize(NUM_MOTORS);
  motor_limits_.resize(NUM_MOTORS); 

  /* ********* get parameters ********* */
  // ros::NodeHandle pn("~");
  ros::NodeHandle n;

  std::string robot_desc_string;

  if (!n.getParam("robot_description", robot_desc_string))
  {
    ROS_FATAL("Couldn't get a robot_description from the param server");
    return;
  }

  urdf::Model model;
  model.initString(robot_desc_string);

  XmlRpc::XmlRpcValue joint_names;

  // Gets all of the joints
  if (!n.getParam("crops_joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", n.getNamespace().c_str());
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", n.getNamespace().c_str());
  }
  if (joint_names.size() != (size_t)NUM_JOINTS)
  {
    ROS_ERROR("Wrong number of joints! was: %d, expected: %zu", joint_names.size(), NUM_JOINTS);
  }
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    XmlRpc::XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
          n.getNamespace().c_str());
    }

    joint_names_[i] = (std::string)name_value;
    joint_types_[i] = urdf::Joint::REVOLUTE; // all of our joints are of type revolute

    motor_limits_[i].joint_name = (std::string)name_value;
    motor_limits_[i].min_position = model.getJoint(joint_names_[i])->limits->lower;
    motor_limits_[i].max_position = model.getJoint(joint_names_[i])->limits->upper;

  }
}

AbstractCrops::~AbstractCrops()
{
}

void AbstractCrops::freezeRobot()
{
  // do nothing (can be overridden)
}

void AbstractCrops::refreshMotorStatus()
{
  // do nothing (can be overridden)
}

/* ******************************** joints + motors ******************************** */

int AbstractCrops::getJointIndex(std::string joint_name)
{
  for (int i = 0; i < (int)joint_names_.size(); i++)
  {
    if (joint_names_[i] == joint_name)
      return i;
  }

  ROS_ERROR("Joint not found: %s.", joint_name.c_str());
  return -1;
}

std::vector<std::string> AbstractCrops::getJointNames()
{
  return joint_names_;
}

std::vector<int> AbstractCrops::getJointTypes()
{
  return joint_types_;
}

std::vector<double> AbstractCrops::getMotorAngles()
{
  return motor_angles_;
}

std::vector<double> AbstractCrops::getMotorVelocities()
{
  return motor_velocities_;
}

std::vector<arm_navigation_msgs::JointLimits> AbstractCrops::getMotorLimits()
{
  return motor_limits_;
}

double AbstractCrops::getMotorLimitMax(std::string joint_name)
{
  for (size_t i = 0; i < motor_limits_.size(); i++)
  {
    if (motor_limits_[i].joint_name == joint_name)
    {
      return motor_limits_[i].max_position;
    }
  }

  return -1;
}

double AbstractCrops::getMotorLimitMin(std::string joint_name)
{
  for (size_t i = 0; i < motor_limits_.size(); i++)
  {
    if (motor_limits_[i].joint_name == joint_name)
    {
      return motor_limits_[i].min_position;
    }
  }

  return -1;
}

}
