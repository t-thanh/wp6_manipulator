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
 * JointStatePublisher.cpp
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#include "JointStatePublisher.h"

namespace crops
{

JointStatePublisher::JointStatePublisher(boost::shared_ptr<AbstractCrops> crops) :
  crops(crops)
{
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::JointState> ("joint_states", 1000);
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::update()
{
  /* ************** Publish joint angles ************** */
  sensor_msgs::JointStatePtr msg = boost::make_shared<sensor_msgs::JointState>();
  std::vector<std::string> joint_names = crops->getJointNames();
  std::vector<double> angles = crops->getMotorAngles();
  std::vector<double> vels = crops->getMotorVelocities();

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    msg->name.push_back(joint_names[i]);
    msg->position.push_back(angles[i]);
    msg->velocity.push_back(vels[i]);
  }


  msg->header.stamp = ros::Time::now();
  pub.publish(msg); // NOTE: msg must not be changed after publishing; use reset() if necessary (http://www.ros.org/wiki/roscpp/Internals)
}

}
