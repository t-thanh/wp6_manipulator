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
 * CropsNode.h
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#ifndef CROPSNODE_H_
#define CROPSNODE_H_

#include <ros/ros.h>

#include <JointStatePublisher.h>
#include <joint_trajectory_action_controller.h>
#include <joint_movement_action_controller.h>

#include <AbstractCrops.h>
#include <SimulatedCrops.h>

namespace crops
{

/**
 * @brief This is the node providing all publishers/services/actions relating to the Crops arm.
 *
 * It actually calls several other classes to do the real work.
 */
class CropsNode
{
public:
  CropsNode();
  virtual ~CropsNode();
  int loop();

private:
  boost::shared_ptr<crops::AbstractCrops> crops;
  ros::NodeHandle nh;  // just to make sure that there is at least one node handle which doesn't go out of scope

};

}

#endif /* CROPSNODE_H_ */
