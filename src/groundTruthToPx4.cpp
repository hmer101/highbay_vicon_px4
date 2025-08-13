#include <rclcpp/rclcpp.hpp>

#include "highbay_vicon_px4/groundTruthToPx4.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

#include <iostream>
#include <regex>
#include <string>

#include <chrono> // Include for std::chrono
#include <iomanip> // Include for std::put_time

//#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>

GroundTruthToPx4::GroundTruthToPx4() : Node("ground_truth_to_px4", rclcpp::NodeOptions().use_global_arguments(true)) {
    // PARAMETERS
    this->ns_ = this->get_namespace();

    // Get the name and device number from the namespace
    // TODO: Make a more robust way of doing this!
    std::regex pattern(R"(/(\w+)_(\d+))");
    std::smatch match;

    if (std::regex_search(this->ns_, match, pattern)) {
        // Get the word after the '_'
        this->device_role_ = match[1]; 
        
        if (this->device_role_ == "px4"){ // TODO: better work around
            this->device_role_ = "drone";
        }

        this->device_id_ = std::stoi(match[2]); // Get the number
    } else {
        RCLCPP_INFO(this->get_logger(), "Namespace not set; cannot get device params!");
    }
    
    this->declare_parameter<double>("timer_period_mocap_repub", 0.02);
    this->get_parameter("timer_period_mocap_repub", this->timer_period_mocap_repub_);

    this->declare_parameter("print_debug_msgs", true);
    this->get_parameter("print_debug_msgs", this->print_debug_msgs_);

    this->declare_parameter("t_px4_rel_mocap", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("R_px4_rel_mocap_ypr", std::vector<double>{0.0, 0.0, 0.0});
    this->get_parameter("t_px4_rel_mocap", this->t_px4_rel_mocap_);
    this->get_parameter("R_px4_rel_mocap_ypr", this->R_px4_rel_mocap_ypr_); 

    this->declare_parameter<std::string>("gt_ref_name", "ground_truth");
    this->get_parameter("gt_ref_name", this->gt_ref_name_);
    
    // Variables
    //this->msg_pose_latest_.header.frame_id = ""; // Set an empty frame_id to indicate that the latest pose msg has not yet been received
    
    // TFs
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));

    // A static TF to define the mocap's XYZ to PX4's FRD
    this->tf_static_broadcaster_px4_rel_mocap_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->create_static_tfs();
    

    // SUBSCRIBERS
    //rclcpp::QoS qos_profile_mocap = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_profile_fmu = rclcpp::SensorDataQoS();


    // PUBLISHERS
    int pub_timer_period_ms = static_cast<int>(this->timer_period_mocap_repub_ * 1000); 
    this->timer_pub_mocap_ = this->create_wall_timer(std::chrono::milliseconds(pub_timer_period_ms), std::bind(&GroundTruthToPx4::clbk_publoop, this));


    std::string topic_name = this->ns_ + "/fmu/in/vehicle_visual_odometry";
    this->pub_mocap_px4_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        topic_name, qos_profile_fmu);

    // Print info
    RCLCPP_INFO(this->get_logger(), "GROUND TRUTH TO PX4 CONVERSION NODE");
    RCLCPP_INFO(this->get_logger(), "gt_ref_name: %s", gt_ref_name_.c_str());

}

// CALLBACKS
void GroundTruthToPx4::clbk_publoop() {
    // Compose the ground truth frame name
    std::string device_gt_name = this->device_role_ + std::to_string(this->device_id_) + "_gt";

    // 1) Get pose of device in mocap reference frame (gt_ref_name_)
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        // target = reference frame, source = device frame
        transformStamped = this->tf_buffer_->lookupTransform(
            this->gt_ref_name_,   // target frame (reference / mocap world)
            device_gt_name,       // source frame (device body in mocap)
            tf2::TimePointZero
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        return;
    }

    // 2) Build a PoseStamped from that transform (pose in mocap frame, body orientation is FLU)
    geometry_msgs::msg::PoseStamped pose_in_mocap_frame;
    pose_in_mocap_frame.header = transformStamped.header;
    pose_in_mocap_frame.pose.position.x = transformStamped.transform.translation.x;
    pose_in_mocap_frame.pose.position.y = transformStamped.transform.translation.y;
    pose_in_mocap_frame.pose.position.z = transformStamped.transform.translation.z;
    pose_in_mocap_frame.pose.orientation = transformStamped.transform.rotation;

    // 3) Convert body orientation from FLU -> FRD by rotating 180 deg about X
    {
        tf2::Quaternion q_flu;
        tf2::fromMsg(pose_in_mocap_frame.pose.orientation, q_flu);

        // 180° about X; tf2::Quaternion(x, y, z, w)
        tf2::Quaternion q_flu_to_frd(1.0, 0.0, 0.0, 0.0);

        tf2::Quaternion q_frd = q_flu * q_flu_to_frd;
        q_frd.normalize();

        pose_in_mocap_frame.pose.orientation = tf2::toMsg(q_frd);
    }

    // 4) Transform pose from mocap world to PX4 world frame
    geometry_msgs::msg::PoseStamped pose_in_px4_frame;
    try {
        geometry_msgs::msg::TransformStamped mocap_to_px4 =
            this->tf_buffer_->lookupTransform(
                "px4",               // target frame (PX4 world, typically NED)
                this->gt_ref_name_,  // source frame (mocap world)
                tf2::TimePointZero
            );

        tf2::doTransform(pose_in_mocap_frame, pose_in_px4_frame, mocap_to_px4);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform mocap->px4 failed: %s", ex.what());
        return;
    }

    // 5) Publish to PX4 as VehicleOdometry (PX4 expects NED + body FRD; quaternion order w,x,y,z)
    px4_msgs::msg::VehicleOdometry vehicleOdom;
    vehicleOdom.timestamp =
        pose_in_px4_frame.header.stamp.sec * 1000000ULL +
        pose_in_px4_frame.header.stamp.nanosec / 1000ULL;

    vehicleOdom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    vehicleOdom.position[0] = pose_in_px4_frame.pose.position.x;
    vehicleOdom.position[1] = pose_in_px4_frame.pose.position.y;
    vehicleOdom.position[2] = pose_in_px4_frame.pose.position.z;

    vehicleOdom.q[0] = pose_in_px4_frame.pose.orientation.w; // (w, x, y, z)
    vehicleOdom.q[1] = pose_in_px4_frame.pose.orientation.x;
    vehicleOdom.q[2] = pose_in_px4_frame.pose.orientation.y;
    vehicleOdom.q[3] = pose_in_px4_frame.pose.orientation.z;

    this->pub_mocap_px4_->publish(vehicleOdom);
}

// HELPERS 
// Could do this manually without tfs if faster is required
void GroundTruthToPx4::create_static_tfs(){   
    // Create a transform stamped msg to publish
    geometry_msgs::msg::TransformStamped stamped_px4_rel_mocap;
    geometry_msgs::msg::TransformStamped stamped_mocap_rel_gt;

    // MOCAP TO PX4
    stamped_px4_rel_mocap.header.stamp = this->get_clock()->now();  // ros::Time::now();
    stamped_px4_rel_mocap.header.frame_id = "mocap"; // Mocap frame
    stamped_px4_rel_mocap.child_frame_id = "px4";
    stamped_px4_rel_mocap.transform.translation.x = this->t_px4_rel_mocap_[0];
    stamped_px4_rel_mocap.transform.translation.y = this->t_px4_rel_mocap_[1];
    stamped_px4_rel_mocap.transform.translation.z = this->t_px4_rel_mocap_[2];

    // Compute the rotation matrix from Yaw-Pitch-Roll
    tf2::Quaternion q;
    tf2::Matrix3x3 m;
    m.setRPY(this->R_px4_rel_mocap_ypr_[2], this->R_px4_rel_mocap_ypr_[1], this->R_px4_rel_mocap_ypr_[0]);
    m.getRotation(q);

    stamped_px4_rel_mocap.transform.rotation.x = q.x();
    stamped_px4_rel_mocap.transform.rotation.y = q.y();
    stamped_px4_rel_mocap.transform.rotation.z = q.z();
    stamped_px4_rel_mocap.transform.rotation.w = q.w();

    // GROUND TRUTH TO MOCAP
    stamped_mocap_rel_gt.header.stamp = this->get_clock()->now();  // ros::Time::now();
    stamped_mocap_rel_gt.header.frame_id = this->gt_ref_name_;
    stamped_mocap_rel_gt.child_frame_id = "mocap";
    stamped_mocap_rel_gt.transform.translation.x = 0.0;
    stamped_mocap_rel_gt.transform.translation.y = 0.0;
    stamped_mocap_rel_gt.transform.translation.z = 0.0;
    stamped_mocap_rel_gt.transform.rotation.x = 0.0;
    stamped_mocap_rel_gt.transform.rotation.y = 0.0;
    stamped_mocap_rel_gt.transform.rotation.z = 0.0;
    stamped_mocap_rel_gt.transform.rotation.w = 1.0;

    // Broadcast the transforms
    this->tf_static_broadcaster_px4_rel_mocap_->sendTransform(stamped_px4_rel_mocap);
    this->tf_static_broadcaster_px4_rel_mocap_->sendTransform(stamped_mocap_rel_gt);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundTruthToPx4>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}