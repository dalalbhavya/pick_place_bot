#! /usr/bin/env python3
from distutils.util import execute
import imp
import sys
import copy

from sklearn import tree
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState

def main():
    initial_pose_joint_space = [-0.498977, 0.580166, -1.69259, -2.10077, 2.85878, -0.924968]
    goal_pose_joint_space = [1.01352, 0.277848, -1.90593, -1.52476, 1.34359, 2.25875]
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_demo", anonymous= True)
    
    robot = moveit_commander.RobotCommander()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    


    display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
    planning_frame = group.get_planning_frame()
    print(planning_frame)

    group.set_planning_pipeline_id("chomp")
    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
    group.set_max_velocity_scaling_factor(1.0)

    for i in range(len(group_variable_values)):
        group_variable_values[i] = initial_pose_joint_space[i]

    group.go(group_variable_values, wait = True)
    print("Starting trajectory in 1 second")
    rospy.sleep(1)
    group.stop()

    goal_pose_for_plan = JointState()
    goal_pose_for_plan.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    goal_pose_for_plan.velocity = [0,0,0,0,0,0]
    goal_pose_for_plan.position = [1.01352, 0.277848, -1.90593, -1.52476, 1.34359, 2.25875]
    for i in range(len(group_variable_values)):
        group_variable_values[i] = goal_pose_joint_space[i]

    plan = group.plan(goal_pose_for_plan)
    print(plan)
    group.execute(plan)

    print(type(goal_pose_for_plan))


    




if __name__ == "__main__":
    main()