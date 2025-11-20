# AprilTag Localization
#### Localize camera frame with respect to world frame with known transformation of tag's w.r.t. world.
It publishes <geometry_msgs::PoseStamped> topic named with exactly same with the "camera_frame_name" argument in launch file


## Dependencies

To install the dependent packages, please follow their instructions. </br>

You have to install both packages. Apriltag_ros depends on Apriltag.

[APRILTAG](https://github.com/AprilRobotics/apriltag)

[APRILTAG_ROS](https://github.com/AprilRobotics/apriltag_ros)

I recommend you to do binary installation.
```
sudo apt install ros-[YOUR_ROS_DISTRO]-apriltag ros-[YOUR_ROS_DISTRO]-apriltag-ros
```

## Installiation

Clone TRUE_RT_TAG package to you_ws/src to install
```
cd your_ws/src
git clone https://github.com/MinSungjae/AprilTagLocalization

cd your_ws/
catkin_make
```

#### Who are suffering OpenCV dependency error while building True RT TAG with AprilTag
Try install opencv3.2 library. You may have broken installation about OpenCV.
```
sudo apt install libopencv3.2
```

## Usage

Before starting, you have to configure the AprilTag ROS package to detect tags. </br>

To configure, go to the Apriltag ROS package directory and open tags.yaml file with a text file editor.

```
roscd apriltag_ros/config
sudo gedit tags.yaml
```
if you install AprilTag package with source(git clone AprilTag), then you can just edit without sudo command.


Add tag information into standalone_tags:
```
standalone_tags:
[
    {id: 256, size: 0.16, name: TAG256},
    {id: 257, size: 0.16, name: TAG257},
    {id: 258, size: 0.16, name: TAG258},
    ... # Define tags 
]
```
If you are finished edit, save, and close the file.

These are for the AprilTag detections.


And let's configure True_RT_Tag.
Go to the True_RT_Tag package directory and edit the configuration file.
```
roscd aprilatag_localization/config
gedit [FILE NAME].yaml
```
And you have to define Tag's information about you in apriltag_ros/config/tags.yaml

```
##### TAG DEFINITION OF ENVIRONMENT #####
# [ID, SIZE(m), X(m), Y(m), Z(m), psi(Deg)]
TAG_TRUE_RT:
  TAGS:
    - [256, 0.16, 1.00, 0.00, 1.000, 0.0]
    - [257, 0.16, 0.25, 0.75, 1.000, 90.0]
    - [258, 0.16, 0.50, 0.75, 1.000, 90.0]
    ... # Define tags
```

Because we assume that Tags will be on the wall, theta and phi are fixed to constant.
You have to define all tags, with size, X-Y-Z position, and psi(heading angle).
Direction angle psi is defined as 0 when facing the X-axis of the world coordinate system and follows right-hand rules.
Default coordinate system in ROS is defined as X(front), Y(left), and Z(up).

You can define your own Tag configuration file to test your robot in any environment.
Just make a new file with the same format and define a config that matches your environment.

## Launch

You can launch the True_RT_Tag package with
```
roslaunch aprilatag_localization aprilatag_localization.launch
```

also, you can modify the launch argument below </br>
```
broadcast_world2cam_tf    // True when only testing standalone.
world_frame_name          //  World frame name
camera_frame_name         // Camera's mount frame
image_frame_name          // Optical frame of image
tag_file_name             // Tag config file name
```
Camera frame have to be broadcasted w.r.t. base_link in your TF managing package.
Also, TF of RealSense's internal transformation, for example camera_mount2camera_color_optical_frame, can be published with URDF of RealSense and robot state publisher. (Hint! Please carefully check True RT Tag launch file.)

If you want to launch with your own environment tag configuration file, you can launch with
```
roslaunch aprilatag_localization aprilatag_localization.launch tag_file_name:=[YOUR_CONFIG_FILE_NAME]
```
(without extension (.yaml))
