#!/usr/bin/env python
# This file listens for the transform between the camera and the detected
# AprilTag for a specified time, and then publishes the transform between
# the specified frme on the robot and the camera. This can then be used as
# a static transform for the camera (and should be valid provided the camera
# doesn't move).
import rospy
import tf
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    # The node must have this name in order to be in the same param nodespace
    # as the online version of camera calibration
    rospy.init_node('camera_calibration_tag')

    # Load the tf listener and broadcaster
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    # Load the static transforms
    calib_duration = rospy.Duration(rospy.get_param("~calib_duration"))
    calib_parent_frame = rospy.get_param("~calib_parent_frame")
    camera_frame = rospy.get_param("~camera_frame")
    fixed_tag_frame = rospy.get_param("~fixed_tag_frame")
    deteced_tag_frame = rospy.get_param(
        "~apriltag_detector/standalone_tags")[0]["name"]

    # Listen to the transform for calib_duration secs
    rate = rospy.Rate(30.0)
    first_msg_recv_time = None
    total_trans = [0.0, 0.0, 0.0]
    total_rot = [0.0, 0.0, 0.0, 0.0]
    num_transforms = 0
    rospy.loginfo("Listening for %d secs" % calib_duration.secs)
    while ((not rospy.is_shutdown()) and
           (first_msg_recv_time is None or
            rospy.Time.now() <= first_msg_recv_time + calib_duration)):
        try:
            # Get the camera's pose in the tag's frame
            transform = listener.lookupTransform(
                deteced_tag_frame, camera_frame, rospy.Time(0))
            tag_to_camera_trans, tag_to_camera_rot = transform

            # Add it to the cumulative transform
            total_trans[0] += tag_to_camera_trans[0]
            total_trans[1] += tag_to_camera_trans[1]
            total_trans[2] += tag_to_camera_trans[2]
            total_rot[0] += tag_to_camera_rot[0]
            total_rot[1] += tag_to_camera_rot[1]
            total_rot[2] += tag_to_camera_rot[2]
            total_rot[3] += tag_to_camera_rot[3]
            num_transforms += 1

            if first_msg_recv_time is None:
                first_msg_recv_time = rospy.Time.now()
                rospy.loginfo("Got first transform")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if first_msg_recv_time is None:
                rospy.logwarn_throttle(
                    15, ("No detected frame %s. " % deteced_tag_frame) +
                    "Are you sure apriltag_ros is running?")

        rate.sleep()

    # Get the average transform from the camera to the detected tag
    avg_trans = [total_trans[i] /
                 num_transforms for i in range(len(total_trans))]
    avg_rot = [total_rot[i]/num_transforms for i in range(len(total_rot))]
    rospy.loginfo("Tag_To_Camera Avg Translation: %s" % avg_trans)
    rospy.loginfo("Tag_To_Camera Avg Rotation: %s" % avg_rot)

    # Compute the final transform
    try:
        cam = PoseStamped()
        cam.header.stamp = rospy.Time.now()
        cam.header.frame_id = fixed_tag_frame
        cam.pose.position.x = avg_trans[0]
        cam.pose.position.y = avg_trans[1]
        cam.pose.position.z = avg_trans[2]
        cam.pose.orientation.x = avg_rot[0]
        cam.pose.orientation.y = avg_rot[1]
        cam.pose.orientation.z = avg_rot[2]
        cam.pose.orientation.w = avg_rot[3]

        cam_in_robot_frame = listener.transformPose(calib_parent_frame, cam)
        rospy.loginfo("Camera in Robot Frame: %s" % cam_in_robot_frame)
        print("Static Transform: %f %f %f %f %f %f %f %s %s" % (
            cam_in_robot_frame.pose.position.x,
            cam_in_robot_frame.pose.position.y,
            cam_in_robot_frame.pose.position.z,
            cam_in_robot_frame.pose.orientation.x,
            cam_in_robot_frame.pose.orientation.y,
            cam_in_robot_frame.pose.orientation.z,
            cam_in_robot_frame.pose.orientation.w,
            calib_parent_frame,
            camera_frame))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("Could not get transform from %s to %s: Terminating" % (
            fixed_tag_frame, calib_parent_frame))
