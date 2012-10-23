#test client for ik_trajectory_tutorial

import roslib
roslib.load_manifest('crops_wp6_arm_navigation_tutorials')
import rospy
from crops_wp6_arm_navigation_tutorials.srv import ExecuteCartesianIKTrajectory
from geometry_msgs.msg import Pose
import time
import sys
import pdb
import tf

#execute a Cartesian trajectory defined by a root frame, a list of 
#positions (x,y,z), and a list of orientations (quaternions: x,y,z,w)
def call_execute_cartesian_ik_trajectory(frame, positions, orientations):
    rospy.wait_for_service("execute_cartesian_ik_trajectory")

    #fill in the header (don't need seq)
    header = roslib.msg.Header()
    header.frame_id = frame
    header.stamp = rospy.get_rostime()

    #fill in the poses
    poses = []
    for (position, orientation) in zip(positions, orientations):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        poses.append(pose)

    #call the service to execute the trajectory
    print "calling execute_cartesian_ik_trajectory"
    try:
        s = rospy.ServiceProxy("execute_cartesian_ik_trajectory", \
                                   ExecuteCartesianIKTrajectory)
        resp = s(header, poses)
    except rospy.ServiceException, e:
        print "error when calling execute_cartesian_ik_trajectory: %s"%e
        return 0
    return resp.success

#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])

#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":
    rospy.init_node("test_cartesian_ik_trajectory_executer")
    tf_listener = tf.TransformListener()
    time.sleep(.5) #give the transform listener time to get some frames

    # not needed, fix tutorial
    joint_names = ["joint1",
                   "joint2",
                   "joint3",
                   "joint4",
                   "joint5",
                   "joint6",
                   "joint7",
                   "joint8",
                   "joint9"]

    positions = [[.12, .78, 1.23], [-0.29, 0.76, 0.93]]
    orientations = [[.00, -.09, 0.0, 1.0], [0.25, -0.21, .18, .62]]

    success = call_execute_cartesian_ik_trajectory("/base_link", \
            positions, orientations)

    #check the final pose
    (trans, rot) = tf_listener.lookupTransform('base_link', 'body9', rospy.Time(0))
    print "end Cartesian pose: trans", pplist(trans), "rot", pplist(rot)

    if success:
        print "trajectory succeeded!"
    else:
        print "trajectory failed."
