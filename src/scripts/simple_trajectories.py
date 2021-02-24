#!/usr/bin/env python

import adapy
import rospy

import pdb
pdb.set_trace()

rospy.init_node("adapy_simple_traj")
rate = rospy.Rate(10)

if not rospy.is_shutdown():
    ada = adapy.Ada(True)
    viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")
    canURDFUri = "package://pr_assets/data/objects/can.urdf"
    sodaCanPose = [1.0, 0.0, 0.73, 0, 0, 0, 1]
    tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
    tablePose = [1., 0.0, 0.0, 0.707107, 0, 0, 0.707107]
    world = ada.get_world()
    can = world.add_body_from_urdf(canURDFUri, sodaCanPose)
    table = world.add_body_from_urdf(tableURDFUri, tablePose)

    collision = ada.get_self_collision_constraint()

    arm_skeleton = ada.get_arm_skeleton()
    positions = arm_skeleton.get_positions()
    arm_state_space = ada.get_arm_state_space()

    positions2 = positions.copy()
    positions3 = positions.copy()
    positions4 = positions.copy()
    positions2[0] += 0.5
    positions2[2] += 0.2
    positions3[0] += 1.5
    positions3[2] += 0.4
    positions4[1] -= 0.4
    positions4[2] += 0.6

    waypoints = [(0.0, positions), (1.0, positions2), (2.0, positions3), (3.0, positions4)]
    traj = ada.compute_joint_space_path(arm_state_space, waypoints)
    ada.execute_trajectory(traj)

    rospy.sleep(1.0)
