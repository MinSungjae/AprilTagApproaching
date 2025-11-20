#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_map>
#include <vector>

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)


class APRILTAG_APPROACHING
{
private:
    ros::NodeHandle* _nh;
    ros::Rate        _rate;

    bool config_loaded = false;

    // Parameters
    std::string package_path;
    std::string tag_config_name;
    std::string world_frame_name;
    std::string base_frame_name;

    double threshold_dist = 1.5;

    // TF components
    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Data loaded from config:
    //   tag_name -> T_tag_target
    std::unordered_map<std::string, tf2::Transform> tag_targets_;

    tf2_ros::TransformBroadcaster dynamic_tf_broadcaster_; 

    // latest requested tag name
    std::string requested_tag_name;

    // Internal functions
    bool loadTagConfig(const std::string& file_name);

    void requestedTagCB(const std_msgs::String& msg);

public:
    // Publishers
    ros::Publisher target_pose_pub;
    ros::Publisher error_pose_pub;

    // Subscribers
    ros::Subscriber tag_request_sub;

    APRILTAG_APPROACHING(ros::NodeHandle* nh, ros::Rate rate);

    // Main processing function
    bool computeAlignment();
};
