#include <rclcpp/rclcpp.hpp>

#include "highbay_vicon_px4/highbayToPx4.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

#include <chrono> // Include for std::chrono
#include <iomanip> // Include for std::put_time

HighbayToPx4::HighbayToPx4() : Node("mocap_to_px4", rclcpp::NodeOptions().use_global_arguments(true)) {
    // PARAMETERS
    // this->declare_parameter<std::string>("env", "phys");
    // this->get_parameter("env", this->env_);

    // this->declare_parameter<int>("num_drones", 3);
    // this->get_parameter("num_drones", this->num_drones_);

    // this->declare_parameter<int>("first_drone_num_", 1);
    // this->get_parameter("first_drone_num_", this->first_drone_num_);

    this->declare_parameter<int>("drone_id", 1);
    this->get_parameter("drone_id", this->drone_id_);

    // float estimation_timer_period;
    // this->declare_parameter<double>("timer_estimation", 0.1);
    // this->get_parameter("timer_estimation", estimation_timer_period);

    // this->declare_parameter<double>("est_threshold_ang_dist_", 10.0);
    // this->get_parameter("est_threshold_ang_dist_", this->est_threshold_ang_dist_);

    // this->declare_parameter<double>("est_threshold_time_", 3.0);
    // this->get_parameter("est_threshold_time_", this->est_threshold_time_);

    // // Get the current time
    // auto now = std::chrono::system_clock::now();
    // auto init_time = std::chrono::system_clock::to_time_t(now);

    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&init_time), "%Y_%m_%d_%H_%M_%S_"); // Format the time
 
    // // VARIABLES
    // this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));

    // this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // int est_timer_period_ms = static_cast<int>(estimation_timer_period * 1000); 
    // this->estimation_timer_ = this->create_wall_timer(
    //                             std::chrono::milliseconds(est_timer_period_ms),
    //                             std::bind(&SlungPoseEstimationOnline::clbk_estimation, this));

    // this->marker_pose_measurements_.resize(this->num_drones_);
    // this->state_current_estimate_ = droneState::State("world", droneState::CS_type::ENU);

    // // SETUP
    // this->start_time_ = this->get_clock()->now();

    // Print info
    RCLCPP_INFO(this->get_logger(), "HIGHBAY VICON TO PX4 CONVERSION NODE");
    
}

// void SlungPoseEstimationOnline::clbk_estimation(){

// }   


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HighbayToPx4>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}