#! /usr/bin/env python3
import sys
import copy

from sklearn import tree
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_demo", anonymous= True)
    
    robot = moveit_commander.RobotCommander()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
    planning_frame = group.get_planning_frame()
    print(planning_frame)

    group.clear_pose_targets()
    


if __name__ == "__main__":
    main()