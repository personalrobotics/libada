#!/usr/bin/env python
# This file launches a node that continuously computes the transform from the
# camera to the specified AprilTag, and republishes that transform to attach
# the camera to the robot's TF tree.
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('camera_calibration_tag')

    # Load the tf listener and broadcaster
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    # Load the static transforms
    tag_to_camera_trans = rospy.get_param(
        "~tag_to_camera_default_transform/translation")
    tag_to_camera_rot = rospy.get_param(
        "~tag_to_camera_default_transform/rotation")
    camera_frame = rospy.get_param("~camera_frame")
    fixed_tag_frame = rospy.get_param("~fixed_tag_frame")
    deteced_tag_frame = rospy.get_param(
        "~apriltag_detector/standalone_tags")[0]["name"]

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            # Get the camera's pose in the tag's frame
            transform = listener.lookupTransform(
                deteced_tag_frame, camera_frame, rospy.Time(0))
            tag_to_camera_trans, tag_to_camera_rot = transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # Use the most recent transformation
            pass

        # Broadcast the transform from the tag to the camera, connecting the
        # camera to the robot's TF tree
        broadcaster.sendTransform(tag_to_camera_trans, tag_to_camera_rot,
                                  rospy.Time.now(), camera_frame, fixed_tag_frame)

        rate.sleep()
