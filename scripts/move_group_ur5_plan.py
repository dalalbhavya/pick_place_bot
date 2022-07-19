#! /usr/bin/env python3
from distutils.util import execute
import imp
import sys
import copy
from unittest import result
from io import BytesIO

from sklearn import tree
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import RobotTrajectory
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

    plan_moveit_planner = group.plan(goal_pose_for_plan)
    print(plan_moveit_planner[1])
    group.execute(plan_moveit_planner[1]) 

    print(type(plan_moveit_planner[1]))
    create_robot_trajectory()

def create_robot_trajectory():
    computed_trajectory = [[-0.499, -0.499, -0.499, -0.4915, -0.4712, -0.4347, -0.3803, -0.3077, -0.218, -0.1134, 0.0032, 0.1281, 0.2573, 0.3864, 0.5113, 0.6279, 0.7325, 0.8222, 0.8948, 0.9493, 0.9858, 1.0061, 1.0135, -0.0, -0.0, 0.0747, 0.2028, 0.3651, 0.5445, 0.726, 0.8968, 1.0462, 1.1658, 1.2491, 1.2918, 1.2918, 1.2491, 1.1658, 1.0462, 0.8968, 0.726, 0.5445, 0.3651, 0.2028, 0.0747, -0.0, -0.0, 0.7473, 1.2812, 1.6228, 1.7936, 1.8149, 1.7082, 1.4946, 1.1957, 0.8327, 0.427, 0.0, -0.427, -0.8327, -1.1957, -1.4946, -1.7082, -1.8149, -1.7936, -1.6228, -1.2812, -0.7473, 0.0], [0.5802, 0.5802, 0.5802, 0.5787, 0.5746, 0.5673, 0.5564, 0.5419, 0.524, 0.5031, 0.4798, 0.4548, 0.429, 0.4032, 0.3782, 0.3549, 0.334, 0.3161, 0.3016, 0.2907, 0.2834, 0.2793, 0.2778, 0.0, 0.0, -0.0149, -0.0405, -0.073, -0.1088, -0.1451, -0.1792, -0.2091, -0.233, -0.2497, -0.2582, -0.2582, -0.2497, -0.233, -0.2091, -0.1792, -0.1451, -0.1088, -0.073, -0.0405, -0.0149, 0.0, 0.0, -0.1494, -0.2561, -0.3244, -0.3585, -0.3628, -0.3414, -0.2987, -0.239, -0.1664, -0.0854, -0.0, 0.0854, 0.1664, 0.239, 0.2987, 0.3414, 0.3628, 0.3585, 0.3244, 0.2561, 0.1494, -0.0], [-1.6926, -1.6926, -1.6926, -1.6936, -1.6965, -1.7017, -1.7093, -1.7196, -1.7322, -1.747, -1.7634, -1.781, -1.7993, -1.8175, -1.8351, -1.8515, -1.8663, -1.8789, -1.8892, -1.8969, -1.902, -1.9049, -1.9059, 0.0, 0.0, -0.0105, -0.0286, -0.0515, -0.0768, -0.1024, -0.1265, -0.1476, -0.1644, -0.1762, -0.1822, -0.1822, -0.1762, -0.1644, -0.1476, -0.1265, -0.1024, -0.0768, -0.0515, -0.0286, -0.0105, 0.0, 0.0, -0.1054, -0.1807, -0.2289, -0.253, -0.256, -0.2409, -0.2108, -0.1687, -0.1175, -0.0602, -0.0, 0.0602, 0.1175, 0.1687, 0.2108, 0.2409, 0.256, 0.253, 0.2289, 0.1807, 0.1054, 0.0], [-2.1008, -2.1008, -2.1008, -2.0979, -2.0902, -2.0763, -2.0556, -2.0279, -1.9938, -1.9539, -1.9095, -1.862, -1.8128, -1.7636, -1.716, -1.6716, -1.6318, -1.5976, -1.57, -1.5492, -1.5353, -1.5276, -1.5248, -0.0, -0.0, 0.0285, 0.0772, 0.139, 0.2074, 0.2765, 0.3415, 0.3984, 0.444, 0.4757, 0.492, 0.492, 0.4757, 0.444, 0.3984, 0.3415, 0.2765, 0.2074, 0.1391, 0.0772, 0.0285, -0.0, -0.0, 0.2846, 0.4879, 0.618, 0.6831, 0.6912, 0.6505, 0.5692, 0.4554, 0.3171, 0.1626, 0.0, -0.1626, -0.3171, -0.4554, -0.5692, -0.6505, -0.6912, -0.6831, -0.618, -0.4879, -0.2846, 0.0], [2.8588, 2.8588, 2.8588, 2.8513, 2.831, 2.7944, 2.7399, 2.6671, 2.5773, 2.4725, 2.3557, 2.2306, 2.1012, 1.9718, 1.8467, 1.7299, 1.6251, 1.5352, 1.4625, 1.408, 1.3714, 1.3511, 1.3436, 0.0, 0.0, -0.0749, -0.2032, -0.3658, -0.5454, -0.7273, -0.8984, -1.0481, -1.1679, -1.2513, -1.2941, -1.2941, -1.2513, -1.1679, -1.0481, -0.8984, -0.7273, -0.5454, -0.3658, -0.2032, -0.0749, 0.0, 0.0, -0.7487, -1.2834, -1.6256, -1.7968, -1.8181, -1.7112, -1.4973, -1.1978, -0.8342, -0.4278, -0.0, 0.4278, 0.8342, 1.1978, 1.4973, 1.7112, 1.8181, 1.7968, 1.6256, 1.2834, 0.7487, 0.0], [-0.9249, -0.9249, -0.9249, -0.9022, -0.848, -0.7625, -0.6455, -0.4971, -0.3172, -0.1059, 0.1338, 0.3947, 0.6669, 0.9392, 1.2001, 1.4398, 1.6511, 1.8309, 1.9793, 2.0963, 2.1819, 2.236, 2.2587, 0.0, 0.0001, 0.2266, 0.541, 0.8553, 1.1696, 1.4839, 1.7982, 2.1125, 2.3966, 2.609, 2.7221, 2.7221, 2.609, 2.3966, 2.1124, 1.7981, 1.4839, 1.1696, 0.8553, 0.541, 0.2266, 0.0, 0.0, 2.265, 3.1441, 3.1427, 3.1427, 3.1426, 3.1423, 3.143, 2.8412, 2.1242, 1.131, -0.0001, -1.1312, -2.1243, -2.8413, -3.143, -3.1423, -3.1426, -3.1427, -3.1427, -3.1441, -2.265, -0.0]]
    H = int(len(computed_trajectory[0])/3)

    robot_trajectory = RobotTrajectory()
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    joint_pos_list = []
    joint_vel_list = []
    joint_acc_list = []

    all_msgs = []
    for i in range(len(computed_trajectory[0:H+1])):
        for j in range(len(computed_trajectory)):
            joint_pos_list.append(computed_trajectory[j][i])
            joint_vel_list.append(computed_trajectory[j][i+H+1])
            joint_acc_list.append(computed_trajectory[j][i+2*(H+1)])
        
        joint_trajectory_points = JointTrajectoryPoint()
        joint_trajectory_points.positions = joint_pos_list
        joint_trajectory_points.velocities = joint_vel_list
        joint_trajectory_points.accelerations = joint_acc_list
        duration = Duration()
        duration.data.secs = int(i*0.1)
        duration.data.nsecs = ((i*0.1) - int(i*0.1))*1000000000        
        joint_trajectory.points = joint_trajectory_points
        robot_trajectory.joint_trajectory = joint_trajectory

        all_msgs.append(robot_trajectory)



if __name__ == "__main__":
    main()