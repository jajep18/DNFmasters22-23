// Include header (Read description in header)
#include "../include/dnf_1d.hpp"

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/logging.hpp> //For "get_logger"
#include <iostream>


// Constructor
DNF_1D::DNF_1D(int _dimensions, bool debug) {
    // Set dimensions
    m_dimensions = _dimensions;

    // Check if dimensions are valid
    if (m_dimensions < m_minimum_dims) {
        RCLCPP_ERROR(rclcpp::get_logger("dnf_pubsub"), "Dimensions are too small. Minimum dimensions is %d", m_minimum_dims);
        return;
    }

    // Initialize tensors
    m_activation = torch::zeros({m_dimensions});
    m_output = torch::zeros({m_dimensions});
    m_input = torch::zeros({m_dimensions});

    // Initialize interaction kernel
    m_interaction_kernel = torch::zeros({m_dimensions, m_dimensions});
    m_interaction_kernel[0][0] = 1;
    m_interaction_kernel[0][1] = 1;
    m_interaction_kernel[1][0] = 1;
    m_interaction_kernel[1][1] = 1;
    m_interaction_kernel[1][2] = 1;
    m_interaction_kernel[2][1] = 1;
    m_interaction_kernel[2][2] = 1;

    // Debug
    if (debug) {
        RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "Initialized DNF with %d dimensions", m_dimensions);
    }
}

// Destructor
DNF_1D::~DNF_1D() {
    RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "DNF_1D destructor called");
}

// Process step
void DNF_1D::step(torch::Tensor input, float dt) {
    // Update input
    m_input = input;

    // Update activation
    m_activation = (1.0f - dt) * m_activation + dt * (m_input - m_activation + m_interaction_kernel.matmul(m_activation));

    // Update output
    m_output = 1.0f / (1.0f + torch::exp(-m_activation));
}

// Getters
torch::Tensor DNF_1D::get_activation() {
    return m_activation;
}
torch::Tensor DNF_1D::get_output() {
    return m_output;
}
torch::Tensor DNF_1D::get_input() {
    return m_input;
}
