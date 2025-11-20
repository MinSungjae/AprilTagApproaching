#include <apriltag_approaching/apriltag_approaching.h>

APRILTAG_APPROACHING::APRILTAG_APPROACHING(ros::NodeHandle* nh, ros::Rate rate)
: _nh(nh), _rate(rate), tf_buffer_(), tf_listener_(tf_buffer_)
{
    package_path = ros::package::getPath("apriltag_approaching");

    // Use full namespaced parameter lookup (same as working APRILTAG_LOCALIZATION)
    _nh->getParam(ros::this_node::getName() + "/tag_config_name", tag_config_name);
    _nh->getParam(ros::this_node::getName() + "/world_frame_name", world_frame_name);
    _nh->getParam(ros::this_node::getName() + "/base_frame_name", base_frame_name);
    _nh->getParam(ros::this_node::getName() + "/threshold_dist", threshold_dist);

    // Subscribers
    tag_request_sub = _nh->subscribe("requested_tag", 1,
                        &APRILTAG_APPROACHING::requestedTagCB, this);

    // Publishers
    target_pose_pub  = _nh->advertise<geometry_msgs::PoseStamped>("target_baselink_pose", 1);
    error_pose_pub   = _nh->advertise<geometry_msgs::PoseStamped>("alignment_error", 1);

    // Load config
    if(loadTagConfig(tag_config_name))
        config_loaded = true;
    else
        ROS_ERROR("Failed to load config...");

    ROS_INFO("Alignment localization is initialized.");
}


void APRILTAG_APPROACHING::requestedTagCB(const std_msgs::String& msg)
{
    requested_tag_name = msg.data;
}



bool APRILTAG_APPROACHING::loadTagConfig(const std::string& file_name)
{
    std::string file_path = package_path + "/config/" + file_name + ".yaml";
    ROS_INFO_STREAM("Loading alignment config: " << file_path);

    YAML::Node node;
    try 
    {
        node = YAML::LoadFile(file_path);
    }
    catch(...) 
    {
        ROS_ERROR("Cannot load config file!");
        return false;
    }

    for(size_t i = 0; i < node["TAGS"].size(); i++)
    {
        std::string tag_name = node["TAGS"][i]["name"].as<std::string>();

        double tx = node["TAGS"][i]["target_pose"][0].as<double>();
        double ty = node["TAGS"][i]["target_pose"][1].as<double>();
        double tz = node["TAGS"][i]["target_pose"][2].as<double>();
        double yaw = node["TAGS"][i]["target_pose"][3].as<double>();

        tf2::Transform T_tag_target;
        T_tag_target.setOrigin(tf2::Vector3(tx, ty, tz));

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw * M_PI/180.0);
        T_tag_target.setRotation(q);

        tag_targets_[tag_name] = T_tag_target;

        ROS_INFO_STREAM("Loaded target for tag " << tag_name);
    }

    return true;
}


// ------------------------------------------------------
// MAIN LOGIC
// ------------------------------------------------------
bool APRILTAG_APPROACHING::computeAlignment()
{
    //------------------------------
    // 0) Requested tag validation
    //------------------------------
    if (requested_tag_name.empty()) 
    {
        ROS_WARN_STREAM_THROTTLE(1.0, "No requested tag name received.");
        return false;
    }

    if (tag_targets_.count(requested_tag_name) == 0)
    {
        ROS_ERROR_STREAM(
            "Requested tag '" << requested_tag_name 
            << "' is not found in config.");
        return false;
    }


    //------------------------------
    // 1) TF: base -> tag
    //------------------------------
    tf2::Transform T_base_tag;
    try 
    {
        auto tf_msg = tf_buffer_.lookupTransform(
            base_frame_name,
            requested_tag_name,
            ros::Time(0)
        );
        tf2::fromMsg(tf_msg.transform, T_base_tag);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_STREAM_THROTTLE(1.0,
            "Tag frame not visible: " << requested_tag_name
            << " (" << ex.what() << ")");
        return false;
    }


    //------------------------------
    // 2) Config 정보: 태그 기준 target(XYZ + 추가 yaw)
    //------------------------------
    tf2::Transform T_tag_target_cfg = tag_targets_[requested_tag_name];
    tf2::Vector3   cfg_pos = T_tag_target_cfg.getOrigin();
    tf2::Quaternion cfg_heading_q = T_tag_target_cfg.getRotation();


    //------------------------------
    // 3) Target orientation 구성
    //
    // Rot_target = Rot_tag * Rot_face * Heading_rot
    //------------------------------

    // (1) tag orientation
    tf2::Quaternion q_tag = T_base_tag.getRotation();

    // (2) face_rot
    tf2::Quaternion q_face;
    q_face.setRPY(-M_PI/2.0, M_PI/2.0, 0);
    q_face.normalize();

    // (3) config heading rotation
    tf2::Quaternion q_cfg_heading = cfg_heading_q; 
    q_cfg_heading.normalize();

    // (4) 최종 orientation
    tf2::Quaternion q_target = q_face * q_cfg_heading; // Target Yaw

    q_target.normalize();

    // tag->target transform 최종 정의 (TAG 좌표계에서)
    tf2::Transform T_tag_target;
    T_tag_target.setOrigin(cfg_pos);
    T_tag_target.setRotation(q_target);


    // 4) base -> target (LOCAL, base_link 기준)
    tf2::Transform T_base_target = T_base_tag * T_tag_target;

    // 5) TAG 기준 target frame TF publish (선택)
    {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp    = ros::Time::now();
        tf_msg.header.frame_id = requested_tag_name;               // parent = TAG
        tf_msg.child_frame_id  = requested_tag_name + "_target";   // child

        tf_msg.transform = tf2::toMsg(T_tag_target);
        dynamic_tf_broadcaster_.sendTransform(tf_msg);
    }

    // 6) base_link 기준 target pose publish
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = base_frame_name;   // LOCAL
        tf2::toMsg(T_base_target, msg.pose);
        target_pose_pub.publish(msg);
    }

    // 7) alignment error = base->target (로컬 에러)
    {
        geometry_msgs::PoseStamped emsg;
        emsg.header.stamp    = ros::Time::now();
        emsg.header.frame_id = base_frame_name;
        tf2::toMsg(T_base_target, emsg.pose);   // 그대로 에러로 사용
        error_pose_pub.publish(emsg);
    }

    return true;
}