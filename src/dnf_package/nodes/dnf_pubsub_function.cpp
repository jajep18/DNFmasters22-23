#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/logging.hpp" //For "get_logger"
#include <iostream>
#include "../src/dnf_lib.cpp"

#include <torch/torch.h>//

// Custom messages
#include "custom_msgs/msg/circle_info.hpp"
#include "custom_msgs/msg/circle_info_arr.hpp"

#include "../src/actions.cpp"
#include "../include/dnf_1d.hpp"

#include "std_msgs/msg/int8_multi_array.hpp"
#include "custom_msgs/msg/triangulated_circle_info_arr.hpp"
#include "../srv/decision.hpp"

using std::placeholders::_1;

class DNFNode : public rclcpp::Node
{
public:
  DNFNode() : Node("dnf_pubsub"){
    // Create a subscriber to the detected circles
    // circle_subscription_ = this->create_subscription<custom_msgs::msg::TriangulatedCircleInfoArr>(
    //   "triangulated_circle_topic", 1, std::bind(&DNFNode::circle_callback, this, _1));

    // keyword_subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
    //   "keywords", 1, std::bind(&DNFNode::keyword_callback, this, _1));
    
    //DNFinit();

    // // Create service client for the movement command center to recieve decisions on actions and targets
    // decision_client_ = this->create_client<dnf_package::srv::Decision>("decision_service");


  }

private:

  void keyword_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg ){
    if ( msg->data.size() > 0){
      for( int i = 0; i < msg->data.size(); i++){
        //keywords_dnf.set(msg->data[i], 69); // Set the places of the keywords to 69 for debugging purposes
      }


    }
    return;
  }


  void circle_callback(const custom_msgs::msg::CircleInfoArr::SharedPtr msg){
    if (msg->circles.size() > 0){
        // std::string circle_log = "";
        // for(size_t i = 0; i<msg->circles.size(); i++){
        //     circle_log += "\n Circle " + std::to_string(i) + ":";
        //     circle_log += " x=" + std::to_string(msg->circles[i].x);
        //     circle_log += " y=" + std::to_string(msg->circles[i].y);
        //     circle_log += " r=" + std::to_string(msg->circles[i].r);
        //     circle_log += " b/g/r means = " + std::to_string(msg->circles[i].bgr_mean[0]) + "/"
        //                                     + std::to_string(msg->circles[i].bgr_mean[1]) + "/"
        //                                     + std::to_string(msg->circles[i].bgr_mean[2]);
        //     circle_log += " b/g/r var = "   + std::to_string(msg->circles[i].bgr_var [0]) + "/"
        //                                     + std::to_string(msg->circles[i].bgr_var [1]) + "/"
        //                                     + std::to_string(msg->circles[i].bgr_var [2]);
        // }
        // RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles!" << circle_log);
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles!");
    } else {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles.");
    }
    
  }
  // Member variables
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArr>::SharedPtr circle_subscription_;
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArr>::SharedPtr keyword_subscription_;


  // // DNFs
  // DNF_1D keywords_dnf(20, true);      //Contains the input keywords in the command. This is the (true) input of the network
  // DNF_1D color_circles_dnf(3, true);  //Contains the color of the circles. This is an input
  // //TODO: Create 2D DNFs for the position of the circles
  // DNF_1D pos_x_circle_dnf(3, true);   //Contains the x position of the circles. This is an input
  // DNF_1D pos_y_circle_dnf(3, true);   //Contains the y position of the circles. This is an input
  // DNF_1D action_dnf(5, true);         //Contains the actions, this is the output of the network


};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DNFNode>());

  rclcpp::shutdown();
  return 0;
}