#! /usr/bin/env python3

import imp
import rospy
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState

def joint_state_callback(msg):
    print("actual = \n", msg.actual)
    joint_state_publisher = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    joint_state_msg = JointTrajectory()
    joint_state_msg.header = msg.header
    joint_state_msg.joint_names = msg.joint_names
    joint_state_msg.points.positions = [1,1,1,1,1,1]
    joint_state_msg.points.velocities = [1,1,1,1,1,1]
        

def main():
    rospy.init_node('joint_state_node', anonymous=True)
    rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, joint_state_callback)

    rospy.spin()


if __name__ == "__main__":
    main()

