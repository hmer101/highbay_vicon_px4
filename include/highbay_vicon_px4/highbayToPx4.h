#ifndef HIGHBAY_TO_PX4_H
#define HIGHBAY_TO_PX4_H

#include <string>

#include <rclcpp/rclcpp.hpp>
// #include <tf2_ros/transform_listener.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include "std_msgs/msg/string.hpp"

// #include "multi_drone_slung_load_interfaces/msg/phase.hpp"
// #include "slung_pose_estimation/State.h"
// #include "slung_pose_estimation/utils.h"


class HighbayToPx4 : public rclcpp::Node {
public:
    HighbayToPx4();

private:
    // PARAMETERS
    int drone_id_;

    // int num_drones_;
    // int first_drone_num_;

    // std::string env_;
    // rclcpp::Time start_time_;

    // float est_threshold_ang_dist_;
    // float est_threshold_time_;

    // // VARIABLES
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // std::vector<std::optional<geometry_msgs::msg::TransformStamped>> marker_pose_measurements_;
    
    // droneState::State state_current_estimate_;

    // rclcpp::TimerBase::SharedPtr estimation_timer_;
    
    // // Flags 
    // bool state_estimate_set_ = false;

    // // CALLBACKS
    // void clbk_estimation();

};

#endif // HIGHBAY_TO_PX4_H