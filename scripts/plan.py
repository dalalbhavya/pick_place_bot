#! /usr/bin/env python3

import imp
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState

def joint_state_callback(msg):
    positions = [[0,0,0,0,0,0]]
    velocities = [[0,0,0,0,0,0]]
    times = [[1]]
    print("actual = \n", msg.actual)
    joint_state_publisher = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    joint_state_msg = JointTrajectory()
    joint_state_msg.header = msg.header
    joint_state_msg.joint_names = msg.joint_names
    for position, velocity, time in zip(positions, velocities, times):
        joint_state_point_msg = JointTrajectoryPoint()
        joint_state_point_msg.positions = position
        joint_state_point_msg.velocities = velocity
        joint_state_point_msg.time_from_start.secs = time
        joint_state_point_msg.time_from_start.nsecs = 0

def main():
    rospy.init_node('joint_state_node', anonymous=True)
    rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, joint_state_callback)

    rospy.spin()


if __name__ == "__main__":
    main()

