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
 * crops_constants.h
 *
 *  Created on: 25.09.2012
 *      Author: Tien Thanh Nguyen <tienthanh.nguyen@biw.kuleuven.be>
 */

#ifndef CROPS_CONSTANTS_H_
#define CROPS_CONSTANTS_H_

namespace crops
{
/// The number of motors in the crops (= all 9 joints)
const size_t NUM_MOTORS = 9;

/// The number of joints in the crops (= all 9 joints)
const size_t NUM_JOINTS = NUM_MOTORS;

/// KNI time is in 10 milliseconds (most of the time), ROS time is in seconds
static const double KNI_TO_ROS_TIME = 100.0;

/// the conversion factor from KNI coordinates (in mm) to ROS coordinates (in m)
static const double KNI_TO_ROS_LENGTH = 0.001;

/// velocity limit <= 180 [enc / 10 ms]
static const int KNI_MAX_VELOCITY = 180;

/// acceleration limit = 1 or 2 [enc / (10 ms)^2]
static const int KNI_MAX_ACCELERATION = 2;

static const size_t MOVE_BUFFER_SIZE = 16;  // TODO: find out exact value

} // namespace crops


#endif /* CROPS_CONSTANTS_H_ */
