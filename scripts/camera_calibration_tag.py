#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('camera_calibration_tag')

    # Load the tf listener and broadcaster
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    # Load the static transforms
    # robot_to_tag_trans = rospy.get_param("robot_to_tag_transform/translation")
    # robot_to_tag_rot = rospy.get_param("robot_to_tag_transform/rotation")
    # robot_to_tag_parent = rospy.get_param("robot_to_tag_transform/parent_frame")
    # robot_to_tag_child = rospy.get_param("robot_to_tag_transform/child_frame")
    tag_to_camera_default_trans = rospy.get_param("~tag_to_camera_default_transform/translation")
    tag_to_camera_default_rot = rospy.get_param("~tag_to_camera_default_transform/rotation")
    tag_to_camera_default_parent = rospy.get_param("~tag_to_camera_default_transform/parent_frame")
    tag_to_camera_default_child = rospy.get_param("~tag_to_camera_default_transform/child_frame")
    deteced_tag_frame = rospy.get_param("~apriltag_detector/standalone_tags")[0]["name"]

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            transform = listener.lookupTransform(deteced_tag_frame, tag_to_camera_default_child, rospy.Time(0))
            tag_to_camera_trans, tag_to_camera_rot = transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            tag_to_camera_trans = tag_to_camera_default_trans
            tag_to_camera_rot = tag_to_camera_default_rot

        # Broadcast the 
        broadcaster.sendTransform(tag_to_camera_trans, tag_to_camera_rot, rospy.Time.now(), tag_to_camera_default_child, tag_to_camera_default_parent)
        rate.sleep()