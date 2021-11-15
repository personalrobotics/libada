#!/usr/bin/env python

import adapy
import rospy

from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

# import pdb
# pdb.set_trace()

rospy.init_node("adapy_simple_traj")
roscpp_init('adapy_simple_traj', [])
rate = rospy.Rate(10)

print("Initializing ADA")
ada = adapy.Ada(True)
viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")
canURDFUri = "package://pr_assets/data/objects/can.urdf"
sodaCanPose = [0.5, 0.0, 0.73, 0, 0, 0, 1]
tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
tablePose = [0.5, 0.0, 0.0, 0.707107, 0, 0, 0.707107]
world = ada.get_world()
print("Adding Can to world")
can = world.add_body_from_urdf(canURDFUri, sodaCanPose)
print("Adding Table to world")
table = world.add_body_from_urdf(tableURDFUri, tablePose)

positions = ada.get_arm_positions()

world_constraint = ada.get_world_collision_constraint()
self_constraint = ada.get_self_collision_constraint()

print("test original position")
safe_world = world_constraint.is_satisfied(positions)
safe_self = self_constraint.is_satisfied(positions)
assert safe_world, "Detected world collision where none exists"
assert safe_self, "Detected self collision where none exists"
print("PASSED!")

input("Press Enter to Continue")

print("test self-collision")
positions2 = positions.copy()
positions2[2] += 3.14
safe_world = world_constraint.is_satisfied(positions2)
safe_self = self_constraint.is_satisfied(positions2)
assert not safe_world, "Failed to detect world (+ self) collisions"
assert not safe_self, "Failed to detect self collision"
print("PASSED!")
ada.set_arm_positions(positions2)

input("Press Enter to Continue")

print("test robot arm collision")
positions3 = positions.copy()
positions3[1] -= 0.4
positions3[2] += 0.6
safe_world = world_constraint.is_satisfied(positions3)
safe_self = self_constraint.is_satisfied(positions3)
assert not safe_world, "Failed to detect world (+ self) collisions"
assert safe_self, "Detected self collision where none exists"
print("PASSED!")
ada.set_arm_positions(positions3)

input("Press Enter to Continue")

print("test robot hand collision")
positions4 = positions.copy()
positions4[1] = 2.1
safe_world = world_constraint.is_satisfied(positions4)
safe_self = self_constraint.is_satisfied(positions4)
assert not safe_world, "Failed to detect world (+ self) collisions"
assert safe_self, "Detected self collision where none exists"
print("PASSED!")
ada.set_arm_positions(positions4)

input("Press Enter to Continue")
