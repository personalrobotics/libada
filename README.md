# libada [![Build Status](https://github.com/personalrobotics/libada/actions/workflows/build-test.yml/badge.svg?branch=master)](https://github.com/personalrobotics/libada/actions)

C++ library for simulating and running ADA based on DART and AIKIDO

## Launching the robot (default setup)

Simply run `roslaunch libada default.launch`.

Next, you can run a demo from `ada_demos`, that uses the plain Ada robot without any attachments.

## Launching the robot for the feeding demo

The feeding demo requires some more configuration and some additional nodes.
It can be started by running `roslaunch libada default.launch feeding:=true`.
To run it with simulated perception `roslaunch libada default.launch feeding:=true perception:=false`.

You may also need to start
- The camera node, running directly on the camera board. Typically the script is called something like `run_all.sh`
- The forque sensor controller, using `roslaunch forque_sensor_hardware forque.launch`
- The feeding demo parameters, and the feeding demo itself.

## Launching the simulated environment for the feeding demo

Simply run `roslaunch libada simulation.launch`.

This adds mandatory TF trees that the ADA demos are expecting, plus simulated food and face detection.

## Launching the simple perception demo

Run `roslaunch libada simple_perception.launch`.

It takes one argument `adareal`, which should be set to `true` if running on the real robot:
`roslaunch libada simple_perception.launch adareal:=true`

For more information, see the [simple_perception demo in ada_demos](https://github.com/personalrobotics/ada_demos/tree/master/simple_perception).

## Calibrating the Camera
This package offers two techniques for calibrating the camera using AprilTags: (a) real-time; and (b) offline. Both require the same setup.

### Setup
1. Clone the [apriltag](https://github.com/AprilRobotics/apriltag)and [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) repositories into your workspace,and rebuild your workspace.
2. Generate and print AprilTags. There are several ways to do so; we used [this](https://berndpfrommer.github.io/tagslam_web/making_tags/). Tip 1: Upon printing the AprilTag, write down which side is top and bottom on the back of the tag. Tip 2: The process of printing an AprilTag will sometimes change its size, so re-measure the size (in cm) in order to accurately specify it as a parameter in step 5.
3. Affix a printed AprilTag onto the robot. Select a location so that it will be easy to measure a transform from a robot joint to the tag.
4. Measure the transform from a joint on the robot's URDF to the tag, and update the static transform node `st_robot2tag` in `launch/default.launch` and `launch/offline_camera_calibration.launch` accordingly.
5. Specify the tag parameters, referring to `config/gen2_6dof.yaml` as an example. The most important thing to ensure is that the `id` and `size` in `standalone_tags` and the `tag_family` match the tag you affixed to the robot. `tag_to_camera_default_transform` is not required, but is a good fallback; one way to get that transform is by running the `apriltag_ros` node by itself and using [tf_echo](http://wiki.ros.org/tf#tf_echo).

### Real-Time Calibration
Roslaunch `default_launch` with `use_apriltag_calib:=true`

### Offline Calibration
1. Roslaunch `offline_camera_calibration.launch`,where parameter `calib_parent_frame` specifies the frame of the robot you'd like to get the camera transform relative to.
2. Copy and paste the static transform outputted by the launched rosnode into the `st_robot2camera`node of `default.launch`.

