#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <iostream>
#include <chrono> //500ms

#include "../src/vision.cpp"

using namespace std::chrono_literals; //Ugly, yet necessarry
// Create publisher (For hough circle data)
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

void imageCallbackCircleDetect(const sensor_msgs::msg::Image::ConstSharedPtr & msg) //Image::ConstSharedPtr & msg //Image::SharedPtr  msg
{

  std::vector<cv::Vec3f> circles;
  cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
  try {

    circles = detect_circles(src);


    cv::putText(src, //target image
          "Detected " + std::to_string(circles.size() ) + " circles" , //text
          cv::Point(10, src.rows / 10), //top-left position
          cv::FONT_HERSHEY_DUPLEX,
          0.6,
          CV_RGB(225, 255, 255), //font color
          2);

    cv::imshow("view", src);

    // cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) //Image::ConstSharedPtr & msg //Image::SharedPtr  msg
{

  cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
  try {
    cv::imshow("view", src);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

void timer_callback(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_)
{
  size_t count_ = 0;
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  auto logger = rclcpp::get_logger("my_publisher");
  RCLCPP_INFO(logger, "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("cam_subscriber", options);

    // Create CV Window    
    cv::namedWindow("view");
    cv::startWindowThread();
    
    // Image transport
    image_transport::ImageTransport it(node);//it(nh);
    image_transport::Subscriber sub = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, imageCallbackCircleDetect);

    // Create publisher (For hough circle data)
    rclcpp::TimerBase::SharedPtr timer_;
    publisher_ = node->create_publisher<std_msgs::msg::String>("cam_circle_topic", 1); //normally 1
    timer_ = node->create_wall_timer(500ms, std::bind(&timer_callback, node));
    //timer_ = node->create_wall_timer(500ms, []() -> void { timer_callback(publisher_); });

    //Start node
    rclcpp::spin(node);

    //Close windows again
    cv::destroyWindow("view");
    rclcpp::shutdown();

    return 0;
}