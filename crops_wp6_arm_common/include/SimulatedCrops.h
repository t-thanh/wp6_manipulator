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
 * SimulatedCrops.h
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#ifndef SIMULATEDCROPS_H_
#define SIMULATEDCROPS_H_

#include "AbstractCrops.h"
#include "spline_functions.h"

namespace crops
{

class SimulatedCrops : public crops::AbstractCrops
{
public:
  SimulatedCrops();
  virtual ~SimulatedCrops();

  virtual void refreshEncoders();
  virtual bool executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj);
  virtual bool moveJoint(int jointIndex, double turningAngle);

  virtual bool someMotorCrashed();
  virtual bool allJointsReady();
  virtual bool allMotorsReady();

private:
  boost::shared_ptr<SpecifiedTrajectory> current_trajectory_;
};

}

#endif /* SIMULATEDCROPS_H_ */
