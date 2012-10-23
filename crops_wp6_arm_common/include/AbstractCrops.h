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
 * AbstractCrops.h
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#ifndef ABSTRACTCROPS_H_
#define ABSTRACTCROPS_H_

#include <ros/ros.h>
#include <urdf/joint.h>
#include <urdf/model.h>

#include <SpecifiedTrajectory.h>
#include <crops_constants.h>

#include <arm_navigation_msgs/JointLimits.h>

namespace crops
{

class AbstractCrops
{
public:
  AbstractCrops();
  virtual ~AbstractCrops();

  virtual void refreshEncoders() = 0;
  virtual bool executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj) = 0;
  virtual void freezeRobot();

  /**
   * Move the joint to the desired angle. Do not wait for result,
   * but return immediately.
   *
   * @param jointIndex the joint to move
   * @param turningAngle the target angle
   * @return true iff command was successfully sent to Crops
   */
  virtual bool moveJoint(int jointIndex, double turningAngle) = 0;

  virtual int getJointIndex(std::string joint_name);

  virtual std::vector<std::string> getJointNames();
  virtual std::vector<int> getJointTypes();

  virtual std::vector<double> getMotorAngles();
  virtual std::vector<double> getMotorVelocities();

  virtual std::vector<arm_navigation_msgs::JointLimits> getMotorLimits();
  virtual double getMotorLimitMax(std::string joint_name);
  virtual double getMotorLimitMin(std::string joint_name);

  virtual void refreshMotorStatus();
  virtual bool someMotorCrashed() = 0;
  virtual bool allJointsReady() = 0;
  virtual bool allMotorsReady() = 0;


protected:
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;

  std::vector<double> motor_angles_;
  std::vector<double> motor_velocities_;


  std::vector<arm_navigation_msgs::JointLimits> motor_limits_;
};

}

#endif /* ABSTRACTCROPS_H_ */
