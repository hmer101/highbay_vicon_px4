#include <rclcpp/rclcpp.hpp>

#include "highbay_vicon_px4/highbayToPx4.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

#include <iostream>
#include <regex>
#include <string>

#include <chrono> // Include for std::chrono
#include <iomanip> // Include for std::put_time

//#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>

HighbayToPx4::HighbayToPx4() : Node("mocap_to_px4", rclcpp::NodeOptions().use_global_arguments(true)) {
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

    // this->declare_parameter<std::string>("device_role", "drone");
    // this->get_parameter("device_role", this->device_role_);

    // this->declare_parameter<int>("device_id", 1);
    // this->get_parameter("device_id", this->device_id_);

    this->declare_parameter<double>("timer_period_mocap_repub", 0.02);
    this->get_parameter("timer_period_mocap_repub", this->timer_period_mocap_repub_);

    this->declare_parameter("t_px4_rel_mocap", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("R_px4_rel_mocap_ypr", std::vector<double>{0.0, 0.0, 0.0});
    this->get_parameter("t_px4_rel_mocap", this->t_px4_rel_mocap_);
    this->get_parameter("R_px4_rel_mocap_ypr", this->R_px4_rel_mocap_ypr_); 

    std::string topic_mocap;
    this->declare_parameter<std::string>("topic_mocap","");
    this->get_parameter("topic_mocap", topic_mocap);
    topic_mocap = "/" + topic_mocap + "_" + this->device_role_ + std::to_string(this->device_id_) + "/world";  // Add modifiers for this device  

    // Variables
    this->msg_pose_latest_.header.frame_id = ""; // Set an empty frame_id to indicate that the latest pose msg has not yet been received
    
    // TFs
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));

    // A static TF to define the mocap's XYZ to PX4's FRD
    this->tf_static_broadcaster_px4_rel_mocap_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->create_static_tfs();
    

    // SUBSCRIBERS
    rclcpp::QoS qos_profile_mocap = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_profile_fmu = rclcpp::SensorDataQoS();

    this->sub_mocap_device_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_mocap, qos_profile_mocap, std::bind(&HighbayToPx4::clbk_mocap_received, this, std::placeholders::_1));


    // PUBLISHERS
    int pub_timer_period_ms = static_cast<int>(this->timer_period_mocap_repub_ * 1000); 
    this->timer_pub_mocap_ = this->create_wall_timer(std::chrono::milliseconds(pub_timer_period_ms), std::bind(&HighbayToPx4::clbk_publoop, this));


    std::string topic_name = this->ns_ + "/fmu/in/vehicle_visual_odometry";
    this->pub_mocap_px4_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        topic_name, qos_profile_fmu);

    // Print info
    RCLCPP_INFO(this->get_logger(), "HIGHBAY VICON TO PX4 CONVERSION NODE");
    RCLCPP_INFO(this->get_logger(), "topic_mocap: %s", topic_mocap.c_str());
    
}

// CALLBACKS
void HighbayToPx4::clbk_mocap_received(const geometry_msgs::msg::PoseStamped msg){
    // Store the mocap received msg
    this->msg_pose_latest_ = msg;
}   

void HighbayToPx4::clbk_publoop(){    
    // Check if mocap data has been received yet
    geometry_msgs::msg::PoseStamped pose_latest_FRD;

    if(this->msg_pose_latest_.header.frame_id == ""){
        RCLCPP_WARN(this->get_logger(), "Mocap pose data not yet received!");
        return;
    }else{
        // Convert FLU frame direction to FRD
        tf2::Quaternion q_FLU(
            this->msg_pose_latest_.pose.orientation.x,
            this->msg_pose_latest_.pose.orientation.y,
            this->msg_pose_latest_.pose.orientation.z,
            this->msg_pose_latest_.pose.orientation.w
        );

        tf2::Quaternion q_FLU_FRD(1.0, 0.0, 0.0, 0.0);
        tf2::Quaternion q_FRD = q_FLU * q_FLU_FRD;

        pose_latest_FRD = this->msg_pose_latest_; // Does this deep copy?
        pose_latest_FRD.pose.orientation.x = q_FRD.x();
        pose_latest_FRD.pose.orientation.y = q_FRD.y();
        pose_latest_FRD.pose.orientation.z = q_FRD.z();
        pose_latest_FRD.pose.orientation.w = q_FRD.w();

    }

    // Convert the latest msg to FRD coordinates
    geometry_msgs::msg::PoseStamped poseInPx4Frame;
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        // Lookup transform from mocap frame to px4 frame
        // target frame, source frame
        transformStamped = this->tf_buffer_->lookupTransform("px4", "mocap", tf2::TimePointZero);

        // Create a new PoseStamped for the transformed pose
        tf2::doTransform(pose_latest_FRD, poseInPx4Frame, transformStamped);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }

    // Create px4 message
    px4_msgs::msg::VehicleOdometry vehicleOdom;

    // Set timestamp
    // TODO: This comes out at 0 (apparently OK as pixhawk should fill this in?)
    vehicleOdom.timestamp = transformStamped.header.stamp.sec*1000000 + transformStamped.header.stamp.nanosec / 1000;  // Convert to microseconds

    // Position
    vehicleOdom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED; // POSE_FRAME_FRD;
    vehicleOdom.position[0] = poseInPx4Frame.pose.position.x;
    vehicleOdom.position[1] = poseInPx4Frame.pose.position.y;
    vehicleOdom.position[2] = poseInPx4Frame.pose.position.z;

    // Orientation
    vehicleOdom.q[0] = poseInPx4Frame.pose.orientation.w;  // Note quaternion order: q(w, x, y, z)
    vehicleOdom.q[1] = poseInPx4Frame.pose.orientation.x;  // This order has been tested and is correct
    vehicleOdom.q[2] = poseInPx4Frame.pose.orientation.y;
    vehicleOdom.q[3] = poseInPx4Frame.pose.orientation.z;

    // Velocity
    //vehicle_odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    //vehicle_odometry.velocity[0]
    //vehicle_odometry.angular_velocity[0]

    // DEBUG: Print the quaternion values to see the orientation
    // RCLCPP_INFO(this->get_logger(), "Orientation in NED (w, x, y, z): (%f, %f, %f, %f)",
    //             vehicleOdom.q[0], vehicleOdom.q[1], vehicleOdom.q[2], vehicleOdom.q[3]);

    // Publish to PX4
    this->pub_mocap_px4_->publish(vehicleOdom);
}

// HELPERS 
// Could do this manually without tfs if faster is required
void HighbayToPx4::create_static_tfs(){   
    // Create a transform stamped msg to publish
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();  // ros::Time::now();
    transformStamped.header.frame_id = "mocap";
    transformStamped.child_frame_id = "px4";
    transformStamped.transform.translation.x = this->t_px4_rel_mocap_[0];
    transformStamped.transform.translation.y = this->t_px4_rel_mocap_[1];
    transformStamped.transform.translation.z = this->t_px4_rel_mocap_[2];

    // Compute the rotation matrix from Yaw-Pitch-Roll
    tf2::Quaternion q;
    tf2::Matrix3x3 m;
    m.setRPY(this->R_px4_rel_mocap_ypr_[2], this->R_px4_rel_mocap_ypr_[1], this->R_px4_rel_mocap_ypr_[0]);
    m.getRotation(q);

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // Broadcast the transform
    this->tf_static_broadcaster_px4_rel_mocap_->sendTransform(transformStamped);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HighbayToPx4>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}