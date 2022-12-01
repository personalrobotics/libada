# libada [![Build Status](https://github.com/personalrobotics/libada/actions/workflows/build-test.yml/badge.svg?branch=master)](https://github.com/personalrobotics/libada/actions)

C++ library for simulating and running the Assistive Dexterous Arm (ADA). Currently used by the Personal Robotics Lab (University of Washington), EmPRISE Lab (Cornell University), and ICAROS (University of Southern California).

Currently, the following base robot platforms are supported:

* [Kinova JACO 2 6DoF](https://assistive.kinovarobotics.com/product/jaco-robotic-arm)
* [Kinova JACO 3 7DoF](https://www.kinovarobotics.com/product/gen3-robots)

In principle, any single robot manipulator should be usable with this software. Each one just needs a new YAML file defining the URDF, RobotHW interface, and ROS controllers.

## Dependencies (First Order)

* [AIKIDO](https://github.com/personalrobotics/aikido): Robot system, connecting OMPL, Ros Controllers, Perception, and the DART collision engine
* [ada_description](https://github.com/personalrobotics/ada_description): JACO 2 and Gen3 URDFs
* [pr_ros_controllers](https://github.com/personalrobotics/pr_ros_controllers): Base joint position, velocity, and effort ROS controllers
* [pr_control_msgs](https://github.com/personalrobotics/pr_control_msgs): Actionlib interface for the ROS controllers
* [jaco_hardware](https://github.com/personalrobotics/jaco_hardware): JACO 2 RobotHW Class
* [kortex_hardware](https://github.com/empriselab/kortex_hardware): Gen3 RobotHW Class

Clone the above into your catkin workspace and build with `catkin build`. Note some dependencies might have other dependencies (e.g. [pr_ros_controllers](https://github.com/personalrobotics/pr_ros_controllers)requires [pr_hardware_interfaces](https://github.com/personalrobotics/pr_hardware_interfaces)). See [pr-rosinstalls](https://github.com/personalrobotics/pr-rosinstalls) for a more complete list.

Most recently tested on ROS Noetic, Ubuntu 20.04 LTS.

## Launching the robot

Basic Setup: `roslaunch libada libada.launch`

**Arguments**:
* `sim` (bool, default `false`): Whether to run the RobotHW node and Ros Controllers
* `version` (int, default `2`) and `dof` (int, default `6`): specify the Kinova arm type

This launch file will include sub-launch files to handle RobotHW and URDF generation, and then it will start the robot_state_publisher, which handles the TF tree and allows the robot to be viewed in RViz as the RobotModel.

## JACO 2 Camera Calibration
This package offers two techniques for calibrating the camera using AprilTags: (a) real-time; and (b) offline. Both require the same setup.

### Setup
1. Clone the [apriltag](https://github.com/AprilRobotics/apriltag)and [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) repositories into your workspace,and rebuild your workspace.
2. Generate and print AprilTags. There are several ways to do so; we used [this](https://berndpfrommer.github.io/tagslam_web/making_tags/). Tip 1: Upon printing the AprilTag, write down which side is top and bottom on the back of the tag. Tip 2: The process of printing an AprilTag will sometimes change its size, so re-measure the size (in cm) in order to accurately specify it as a parameter in step 5.
3. Affix a printed AprilTag onto the robot. Select a location so that it is in the camera view and it is easy to measure a transform from a robot joint to the tag.
4. Measure the transform from a joint on the robot's URDF to the tag, and update the static transform node `st_robot2tag` in `launch/default.launch` and `launch/offline_camera_calibration.launch` accordingly.
5. Specify the tag parameters, referring to `config/gen2_6dof.yaml` as an example. The most important thing to ensure is that the `id` and `size` in `standalone_tags` and the `tag_family` match the tag you affixed to the robot. `tag_to_camera_default_transform` is not required, but is a good fallback; one way to get that transform is by running `apriltag_ros` `continuous_detection.launch` by itself and using [tf_echo](http://wiki.ros.org/tf#tf_echo) where the `target_frame` is `camera_link.`

### Real-Time Calibration
`roslaunch libada libada.launch use_apriltag_calib:=true`

### Offline Calibration
1. Roslaunch `gen2_camera_calib.launch`,where parameter `calib_parent_frame` specifies the frame of the robot you'd like to get the camera transform relative to.
2. Pass the static transform as an argument: `libada.launch use_apriltag_calib:=false camera_transform:="x y z qw qx qy qz frame_from frame_to"`

