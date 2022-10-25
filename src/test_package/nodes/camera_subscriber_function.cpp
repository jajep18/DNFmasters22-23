#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1; // MIght not be necessary

// class camSubscriber : public rclcpp::Node
// {
// public:
//   camSubscriber()
//   : Node("cam_subscriber")
//   {
  
//     // Subscribe to image topics - This is the problem!
    
//     subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//        "sensor_msgs/msg/Image", 10, std::bind(&camSubscriber::imageCallback, this, _1));


//     // subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//     //             "Image",, camSubscriber::imageCallback );
    
  
//   }

// void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) //Image::ConstSharedPtr & msg //Image::SharedPtr  msg
// {
//   try {
//     cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//     cv::waitKey(10);
//   } catch (const cv_bridge::Exception & e) {
//     auto logger = rclcpp::get_logger("my_subscriber");
//     RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   }
// }

// private:

//   void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
//   {
//     RCLCPP_INFO(this->get_logger(), "I See: " +  msg->encoding);
//   }



//   // --------------------------------------------------------------------------------
//   // Subscriber instance variable
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

// };

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) //Image::ConstSharedPtr & msg //Image::SharedPtr  msg
{
  try {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    //ros::NodeHandle nh;
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("cam_subscriber", options);

    // Create CV Window    
    cv::namedWindow("view");
    cv::startWindowThread();
    
    // Image transport
    image_transport::ImageTransport it(node);//it(nh);
    image_transport::Subscriber sub = it.subscribe("diff_drive_robot/custom_rgb/image_raw", 1, imageCallback);
    // /diff_drive_robot/custom_rgb/image_raw //camera/image

    //rclcpp::spin(std::make_shared<camSubscriber>());
    rclcpp::spin(node);

    //Close window again
    cv::destroyWindow("view");

    rclcpp::shutdown();
    return 0;
}