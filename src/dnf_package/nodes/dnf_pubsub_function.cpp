#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/logging.hpp" //For "get_logger"
#include <iostream>
//#include "../src/dnf_lib.cpp"
#include <torch/torch.h>//

// Custom messages
#include "custom_msgs/msg/circle_info.hpp"
#include "custom_msgs/msg/circle_info_arr.hpp"

using std::placeholders::_1;

class DNFNode : public rclcpp::Node
{
public:
  DNFNode() : Node("dnf_pubsub"){
    subscription_ = this->create_subscription<custom_msgs::msg::CircleInfoArr>(
      "cam_circle_topic", 1, std::bind(&DNFNode::topic_callback, this, _1));
    //DNFinit();
  }

private:
  void topic_callback(const custom_msgs::msg::CircleInfoArr::SharedPtr msg){
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
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArr>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DNFNode>());

  rclcpp::shutdown();
  return 0;
}