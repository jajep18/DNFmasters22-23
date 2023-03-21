// Include header (Read description in header)
#include "../include/dnf_2d.hpp"

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/logging.hpp> //For "get_logger"
#include <iostream>

// Explicit default constructor (Blame ROS2)
DNF_2D::DNF_2D() {
    RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "DNF_2D default constructor called");
    m_dimensions1 = -1; //Debug value
    m_dimensions2 = -1; //Debug value
}


// Constructor
DNF_2D::DNF_2D(int _dimensions1, int _dimensions2, bool debug) {
    // Set dimensions
    m_dimensions1 = _dimensions1;
    m_dimensions2 = _dimensions2;

    // Check if dimensions are valid
    if ((m_dimensions1 < m_minimum_dims) || (m_dimensions2 < m_minimum_dims)) {
        RCLCPP_ERROR(rclcpp::get_logger("dnf_pubsub"), "Dimensions are too small. Minimum dimensions is %d", m_minimum_dims);
        return;
    }

    // Initialize tensors
    m_activation = torch::zeros({m_dimensions1, m_dimensions2});
    m_output = torch::zeros({m_dimensions1, m_dimensions2});
    m_input1 = torch::zeros({m_dimensions1});
    m_input2 = torch::zeros({m_dimensions2});

    // Initialize interaction kernel
    m_interaction_kernel = torch::zeros({3, 3}); // TODO: Get sizes for the kernels from the DNF constructor
    m_interaction_kernel[0][0] = 1;
    m_interaction_kernel[0][1] = 1;
    m_interaction_kernel[1][0] = 1;
    m_interaction_kernel[1][1] = 1;
    m_interaction_kernel[1][2] = 1;
    m_interaction_kernel[2][1] = 1;
    m_interaction_kernel[2][2] = 1;

    // Debug
    if (debug) {
        RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "Initialized DNF with dimensions: %d %d", m_dimensions1, m_dimensions2);
    }
}

// Destructor
DNF_2D::~DNF_2D() {
    RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "DNF_2D destructor called");
}

// Process step
void DNF_2D::step(torch::Tensor input1, torch::Tensor input2, float dt) {
    // Update input
    m_input1 = input1;
    m_input2 = input2;

    // Update activation
    // m_activation1 = (1.0f - dt) * m_activation1 + dt * (m_input1 - m_activation1 + m_interaction_kernel.matmul(m_activation1));
    // m_activation2 = (1.0f - dt) * m_activation2 + dt * (m_input2 - m_activation2 + m_interaction_kernel.matmul(m_activation2));

    // Test the unsqueeze thingamajig
    auto input1_unsqueezed = m_input1.unsqueeze(1);
    auto input2_unsqueezed = m_input2.unsqueeze(0);

    RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "input1_unsqueezed dimensions: %d %d", input1_unsqueezed.sizes()[0], input1_unsqueezed.sizes()[1]);
    RCLCPP_INFO(rclcpp::get_logger("dnf_pubsub"), "input2_unsqueezed dimensions: %d %d", input2_unsqueezed.sizes()[0], input2_unsqueezed.sizes()[1]);

    // Update m_activation
    m_activation = (1.0f - dt) * m_activation + dt * (m_input1.unsqueeze(1).matmul(m_input2.unsqueeze(0)) - m_activation);
    // m_activation = (1.0f - dt) * m_activation + dt * (m_input1.unsqueeze(1).matmul(m_input2.unsqueeze(0)) - m_activation + m_interaction_kernel.matmul(m_activation));

    // Update output
    // TODO: Check if this is correct
    //m_output = m_activation1.unsqueeze(1).matmul(m_activation2.unsqueeze(0)); Only 1 2d actiovation tensor now
    //m_output = 1.0f / (1.0f + torch::exp(-m_activation)); // This is in 1D
    m_output = m_activation;
}

// Getters
torch::Tensor DNF_2D::get_activation() {
    // if (index_activation == 0){
    //     return m_activation1;
    // } else if (index_activation == 1){
    //     return m_activation2;
    // } else {
    //     RCLCPP_ERROR(rclcpp::get_logger("dnf_pubsub"), "Invalid index_activation %d", index_activation);
    //     return m_activation1;
    // }
    return m_activation;
}
torch::Tensor DNF_2D::get_output() {
    return m_output;
}
torch::Tensor DNF_2D::get_input(int index_input) {
    if (index_input == 0){
        return m_input1;
    } else if (index_input == 1){
        return m_input2;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("dnf_pubsub"), "Invalid index_input %d", index_input);
        return m_input1;
    }
}

// Setters
void DNF_2D::set_activation(torch::Tensor activation) {
    m_activation = activation;
}
void DNF_2D::set_output(torch::Tensor output) {
    m_output = output;
}
void DNF_2D::set_input(torch::Tensor input1, torch::Tensor input2) {
    m_input1 = input1;
    m_input2 = input2;
}
void DNF_2D::set_input_element(int index_input, int index_element, float value) {
    if (index_input == 0){
        m_input1[index_element] = value;
    } else if (index_input == 1){
        m_input2[index_element] = value;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("dnf_pubsub"), "Invalid index_input %d", index_input);
    }
}
void DNF_2D::reset_input() {
    m_input1 = torch::zeros({m_dimensions1});
    m_input2 = torch::zeros({m_dimensions2});
}
