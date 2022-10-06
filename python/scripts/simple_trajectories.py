#!/usr/bin/env python

import adapy
import rospy
import sys

import pdb
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

rospy.init_node("adapy_simple_traj")
roscpp_init('adapy_simple_traj', [])
rate = rospy.Rate(10)
IS_SIM = True

if not rospy.is_shutdown():
    ada = adapy.Ada(IS_SIM)
    if not IS_SIM:
        if not ada.start_trajectory_controllers():
            print("Could not start trajectory controller.")
            sys.exit(1) 
    rospy.sleep(1)  # wait for ada to initialize

    viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")
    canURDFUri = "package://pr_assets/data/objects/can.urdf"
    sodaCanPose = [1.0, 0.0, 0.73, 0, 0, 0, 1]
    tableURDFUri = "package://pr_assets/data/objects/large_white_box.urdf"
    tablePose = [0.45, -0.15, 0.55, 0.707107, 0, 0, 0.707107]
    # tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
    # tablePose = [0.73, 0.73, 0., 0.707107, 0, 0, 0.707107]
    world = ada.get_world()
    can = world.add_body_from_urdf(canURDFUri, sodaCanPose)
    table = world.add_body_from_urdf(tableURDFUri, tablePose)

    collision = ada.get_world_collision_constraint()

    positions = ada.get_arm_positions()

    positions2 = positions.copy()
    positions3 = positions.copy()
    positions4 = positions.copy()
    positions2[0] += 0.5
    positions2[2] += 0.2
    positions3[0] += 1.5
    positions3[2] += 0.4
    positions4[0] += 0.8
    positions4[1] -= 0.4
    positions4[2] += 1.4

    waypoints = [(0.0, positions), (1.0, positions2), (2.0, positions3), (3.0, positions4)]
    waypoints_rev = [(0.0, positions4), (1.0, positions3), (2.0, positions2), (3.0, positions)]
    
    print("")
    print("CONTINUE TO CONFIGURATION")
    print("")
    traj = ada.plan_to_configuration(positions4, collision)
    if not traj:
        print("")
        print("Did not find collision free path!")
        traj = ada.plan_to_configuration(positions4)

    traj_rev = ada.compute_joint_space_path(waypoints_rev)

    print("")
    print("CONTINUE TO EXECUTE")
    print("")
    pdb.set_trace()

    ada.execute_trajectory(traj)

    offset = [0., 0.1, 0.]
    offset_rev = [0., -0.1, 0.]
    hand_node = rospy.get_param("adaConf/hand_base")
    traj_off = ada.plan_to_offset(hand_node, offset, collision)
    traj_off_rev = ada.plan_to_offset(hand_node, offset_rev, collision)

    print("")
    print("CONTINUE TO OFFSET")
    print("")
    pdb.set_trace()

    if traj_off:
        # move to grasp position
        ada.execute_trajectory(traj_off)

        # close hand to grasp object
        PRESHAPE = [1.1, 1.1]
        ada.get_hand().execute_preshape(PRESHAPE)
        print("")
        print("CLOSING HAND")
        print("")
        
        if traj_off_rev:
            # return to offset position
            ada.execute_trajectory(traj_off_rev)

    print("")
    print("CONTINUE TO EXECUTE REVERSE")
    print("")
    pdb.set_trace()

    ada.execute_trajectory(traj_rev)
    
    ada.get_hand().open()
    print("")
    print("OPENING HAND")
    print("")

    print("")
    print("DONE! CONTINUE TO EXIT")
    print("")
    pdb.set_trace()
    if not IS_SIM:
        if not ada.stop_trajectory_controllers():
            print("Could not stop trajectory controllers.")
