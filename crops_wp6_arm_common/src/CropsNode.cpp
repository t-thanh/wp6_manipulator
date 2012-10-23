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
 * CropsNode.cpp
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#include <CropsNode.h>

namespace crops
{

CropsNode::CropsNode()
{
  //bool simulation;
  std::string crops_type;
  ros::NodeHandle pn("~");
  ros::NodeHandle n;

  //pn.param("simulation", simulation, false); //TODO: Should look in the launch to find the right param

  //if (simulation)
    crops.reset(new SimulatedCrops());
  //else
  //{
  //}
}

CropsNode::~CropsNode()
{
}

int CropsNode::loop()
{
  ros::Rate loop_rate(25);

  JointStatePublisher jointStatePublisher(crops);
  JointMovementActionController jointMovementActionController(crops);
  JointTrajectoryActionController jointTrajectoryActionController(crops);

  while (ros::ok())
  {
    crops->refreshEncoders();
    jointStatePublisher.update();
    jointTrajectoryActionController.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crops");
  crops::CropsNode crops_node;

  crops_node.loop();

  return 0;
}
