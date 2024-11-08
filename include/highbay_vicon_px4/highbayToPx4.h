#ifndef HIGHBAY_TO_PX4_H
#define HIGHBAY_TO_PX4_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "px4_msgs/msg/vehicle_odometry.hpp"
// #include "std_msgs/msg/string.hpp"

class HighbayToPx4 : public rclcpp::Node {
public:
    HighbayToPx4();

private:
    // PARAMETERS
    std::string ns_;
    std::string device_role_;
    int device_id_;

    double timer_period_mocap_repub_;

    std::vector<double> t_px4_rel_mocap_;
    std::vector<double> R_px4_rel_mocap_ypr_;

    // VARIABLES
    geometry_msgs::msg::PoseStamped msg_pose_latest_;

    // TFs
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_px4_rel_mocap_;

    // SUBSCRIBERS
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_mocap_device_;

    // PUBLISHERS
    rclcpp::TimerBase::SharedPtr timer_pub_mocap_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_mocap_px4_;
    

    // CALLBACKS
    void clbk_mocap_received(const geometry_msgs::msg::PoseStamped msg);
    void clbk_publoop();

    // HELPERS
    void create_static_tfs();
};

#endif // HIGHBAY_TO_PX4_H