# AprilTag Approaching
This package computes the desired target pose relative to the robot’s base_link frame using a predefined geometric relation between each AprilTag and its corresponding alignment or docking target.

It publishes the following topics:
* /target_baselink_pose (geometry_msgs::PoseStamped)
* /alignment_error (geometry_msgs::PoseStamped)

It also publishes one TF frame per tag:

<tag_name> → <tag_name>_target

This package works purely in local coordinates (base_link frame).
No global “world” or “map” frame is needed.

## Dependencies

Install AprilTag and apriltag_ros:

```
  sudo apt install ros-[YOUR_ROS_DISTRO]-apriltag
  sudo apt install ros-[YOUR_ROS_DISTRO]-apriltag-ros
```

## Installation
```
  cd your_ws/src
  git clone https://github.com/MinSungjae/AprilTagApproaching

  cd your_ws
  catkin_make
```
## Configuration

### 3.1 AprilTag (apriltag_ros) configuration

Edit the tag definitions:
```
roscd apriltag_ros/config
gedit tags.yaml
```

Example:
```
  standalone_tags:
  [
  {id: 10, size: 0.161, name: SIGNBOARD01},
  {id: 11, size: 0.161, name: SIGNBOARD02}
  ]
```

### 3.2 Approaching target configuration

Configuration files are located inside:
```
roscd apriltag_approaching/config
```

Example file format:

```
  TAGS:
  name: SIGNBOARD01
  target_pose: [0.50, 0.00, 0.00, 0.0]

  name: SIGNBOARD02
  target_pose: [0.40, 0.10, 0.00, 180.0]
```

Meaning of target_pose:
* x, y, z: desired target position relative to the tag frame
* yaw(deg): additional heading that robot should achieve at target

The final target orientation is computed using two components:
* q_face_alignment : rotates robot front-left-up so it faces the tag
* q_config_heading : rotation from the YAML heading parameter

Final orientation = q_face_alignment * q_config_heading

## TF Requirements and Outputs
#### Required from your robot system:
* base_link → camera_link
* camera_link → camera_optical_frame

#### Published by this package:
* tag_name → tag_name_target (static position + computed orientation)

#### Topics published:
* target_baselink_pose (in base_link frame)
* alignment_error (same as base→target transform)

## Launching the Node
Example launch file:

```
  <node name="apriltag_approaching" pkg="apriltag_approaching" type="apriltag_approaching_node" output="screen"> <param name="tag_config_name" value="my_docking_config" /> <param name="base_frame_name" value="base_link" /> <param name="threshold_dist" value="2.0" /> </node>
```

You must also run apriltag_ros detector:
```
roslaunch apriltag_ros continuous_detection.launch camera_name:=camera image_topic:=color/image_raw
```
## How It Works (Pipeline)

apriltag_ros publishes TF: camera → tag frame

We compute base_link → tag using TF lookup

From config, we read tag → target offset

Combine to compute base_link → target

Publish:

target_baselink_pose

alignment_error

tag → tag_target TF

The robot controller can subscribe to /alignment_error and drive the robot until the error converges to zero.

## Notes

The package does NOT estimate global pose.

Everything is computed in robot local coordinates.

Ideal for docking, approach, or fine alignment tasks.

Requires consistent TF between base_link and camera_link.