// Include header (Read description in header)
#include "../include/learning_trial.hpp"

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/logging.hpp> //For "get_logger"

// Explicit default constructor (Blame ROS2)
learning_trial::~learning_trial()
{
}

// Constructor
learning_trial::learning_trial(/* args */) {
    // This constructor will not be used. Print that something went wrong
    RCLCPP_ERROR(rclcpp::get_logger("dnf_pubsub"), "learning_trial default constructor called. This should not happen");
}
learning_trial::learning_trial(std::vector<int> _keywords, int _target_color, int _correct_action) {
    keywords = _keywords;
    target_color = _target_color;
    correct_action = _correct_action;
    RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "learning_trial constructor called - Happy learning!");
}

// Getters
int learning_trial::get_target() {
    return target_color;
}

int learning_trial::get_action() {
    return correct_action;
}

std::vector<int> learning_trial::get_keywords() {
    return keywords;
}
