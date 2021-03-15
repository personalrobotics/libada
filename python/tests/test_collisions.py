#!/usr/bin/env python

import adapy
import rospy

# import pdb
# pdb.set_trace()

rospy.init_node("adapy_simple_traj")
rate = rospy.Rate(10)

ada = adapy.Ada(True)
viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")
canURDFUri = "package://pr_assets/data/objects/can.urdf"
sodaCanPose = [0.5, 0.0, 0.73, 0, 0, 0, 1]
tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
tablePose = [0.5, 0.0, 0.0, 0.707107, 0, 0, 0.707107]
world = ada.get_world()
can = world.add_body_from_urdf(canURDFUri, sodaCanPose)
table = world.add_body_from_urdf(tableURDFUri, tablePose)

arm_skeleton = ada.get_arm_skeleton()
positions = arm_skeleton.get_positions()
arm_state_space = ada.get_arm_state_space()


collision_free_constraint = ada.set_up_collision_detection(arm_state_space, arm_skeleton, [can, table])
full_collision_free_constraint = ada.get_full_collision_constraint(arm_state_space, arm_skeleton, collision_free_constraint)

# test original position
safe = full_collision_free_constraint.is_satisfied(arm_state_space, arm_skeleton, positions)
if not safe:
    print("State state is not safe!")

# test self-collision
positions2 = positions.copy()
positions2[1] = 0.4
safe = full_collision_free_constraint.is_satisfied(arm_state_space, arm_skeleton, positions2)
if not safe:
    print("Self-collision!")

# test robot arm collision
positions3 = positions.copy()
positions3[1] -= 0.4
positions3[2] += 0.6
safe = full_collision_free_constraint.is_satisfied(arm_state_space, arm_skeleton, positions3)
if not safe:
    print("Arm body is in collision!")

# test robot hand collision
positions4 = positions.copy()
positions4[1] = 2.1
safe = full_collision_free_constraint.is_satisfied(arm_state_space, arm_skeleton, positions4)
if not safe:
    print("Hand is in collision!")

