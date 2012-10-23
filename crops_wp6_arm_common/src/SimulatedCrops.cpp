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
 * SimulatedCrops.cpp
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#include "../include/SimulatedCrops.h"

namespace crops
{

SimulatedCrops::SimulatedCrops() :
  AbstractCrops()
{
  // Creates a "hold current position" trajectory.
  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = ros::Time::now().toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(NUM_MOTORS);
  for (size_t j = 0; j < NUM_MOTORS; ++j)
	{
		hold[0].splines[j].coef[0] = 0.000;
	}

  current_trajectory_ = hold_ptr;
}

SimulatedCrops::~SimulatedCrops()
{
}

void SimulatedCrops::refreshEncoders()
{
  const SpecifiedTrajectory &traj = *current_trajectory_;

  // Determines which segment of the trajectory to use
  size_t seg = 0;
  while (seg + 1 < traj.size() && traj[seg + 1].start_time <= ros::Time::now().toSec())
  {
    seg++;
  }

  for (size_t j = 0; j < traj[seg].splines.size(); j++)
  {
    double pos_t, vel_t, acc_t;
    sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, ros::Time::now().toSec()
        - traj[seg].start_time, pos_t, vel_t, acc_t);

    motor_angles_[j] = pos_t;
    motor_velocities_[j] = vel_t;
  }
}

bool SimulatedCrops::executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj_ptr)
{
  // ------- wait until start time
  ros::Time::sleepUntil(ros::Time(traj_ptr->at(0).start_time));

  current_trajectory_ = traj_ptr;
  return true;
}

bool SimulatedCrops::moveJoint(int jointIndex, double turningAngle){
  ROS_ERROR("moveJoint() not yet implemented for SimulatedCrops!");
  return false;
}

bool SimulatedCrops::someMotorCrashed() {
  return false;
}

bool SimulatedCrops::allJointsReady() {
  return true;
}

bool SimulatedCrops::allMotorsReady() {
  return true;
}


}
