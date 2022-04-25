#!/usr/bin/env python

import adapy
import rospy
import sys

import pdb
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import numpy as np

rospy.init_node("adapy_tsr_example")
roscpp_init('adapy_tsr_example', [])
rate = rospy.Rate(10)
IS_SIM = True

if not rospy.is_shutdown():
    ada = adapy.Ada(IS_SIM)
    if not IS_SIM:
        if not ada.start_trajectory_controllers():
            print("Could not start trajectory controller.")
            sys.exit(1) 
    rospy.sleep(1)  # wait for ada to initialize

    viewer = ada.start_viewer("dart_markers/tsr_example", "map")
    canURDFUri = "package://pr_assets/data/objects/can.urdf"
    sodaCanPose = [0.2, 0.2, 0., 0, 0, 0, 1]
    world = ada.get_world()
    can = world.add_body_from_urdf(canURDFUri, sodaCanPose)

    collision = ada.get_self_collision_constraint()

    # get arm and hand instances
    arm_skeleton = ada.get_arm_skeleton()
    arm_state_space = ada.get_arm_state_space()
    hand = ada.get_hand()
    hand_node = hand.get_end_effector_body_node()
    positions = ada.get_arm_positions()

    viewer.add_frame(hand_node)

    print("")
    print("CONTINUE TO CREATE TSR")
    print("")
    pdb.set_trace()

    # get default TSR for grasping can
    grasp_tsr = adapy.get_default_TSR()
    T0_w = grasp_tsr.get_T0_w()
    print("T0_w", T0_w)
    
    # reset Tw_e to origin
    grasp_tsr.set_Tw_e(T0_w)
    print("Tw_e", grasp_tsr.get_Tw_e())

    # set T0_w to can origin
    T0_w[0, 3] = sodaCanPose[0]
    T0_w[1, 3] = sodaCanPose[1]
    grasp_tsr.set_T0_w(T0_w)

    # set default Bw
    Bw = np.zeros([6, 2])
    grasp_tsr.set_Bw(Bw)

    # visualize T0_w
    canTSR = viewer.add_tsr_marker(grasp_tsr)
    print("T0_w", grasp_tsr.get_T0_w())

    # set Tw_e to grasp the can from the top
    Tw_e = [[-1.,  0.,  0.,  0.00],
            [ 0.,  1.,  0.,  0.00],
            [ 0.,  0., -1.,  0.15],
            [ 0.,  0.,  0.,  1.00]]
    grasp_tsr.set_Tw_e(Tw_e)

    # set Bw
    Bw[5, 0] = -np.pi
    Bw[5, 1] = np.pi
    grasp_tsr.set_Bw(Bw)
    
    # visualize Tw_e
    graspTSR = viewer.add_tsr_marker(grasp_tsr)
    print("Tw_e", grasp_tsr.get_Tw_e())

    print("")
    print("PLAN TO TSR")
    print("")
    pdb.set_trace()
 
    trials, maxTrials = 0, 10
    while trials < maxTrials:

        # plan_to_tsr may require more than one try to find a succesful path
        traj = ada.plan_to_tsr(rospy.get_param("adaConf/end_effector"), grasp_tsr)
        if traj:
            break
        trials += 1

    print("")
    print("CONTINUE TO EXECUTE")
    print("")
    pdb.set_trace()

    ada.execute_trajectory(traj)

    print("")
    print("DONE! CONTINUE TO EXIT")
    print("")
    pdb.set_trace()
    if not IS_SIM:
        if not ada.stop_trajectory_controllers():
            print("Could not stop trajectory controllers.")
