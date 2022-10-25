#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"
#include<vector>

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

// void circle_pub_callback(std::vector<cv::Vec3f> circles){
//   for (size_t i = 0; i < circles.size(); i++)
//   {
//     std::cout << "Circle [x, y, r] " << i <<" data: ";
//     for (size_t j = 0; j < circles[i].rows ; j++)
//     {
//       std::cout << circles[i][j];
//     }
//     std::cout << "." <<std::endl;
//   }
  
// }

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) //Image::ConstSharedPtr & msg //Image::SharedPtr  msg
{
  cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
  try {
    cv::imshow("view", src);
    // cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  // Hough Transform

    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 1, 30 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    //Draw center and outline of all detected circles 
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        cv::circle( src, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        cv::circle( src, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
  //Show detected circles
  circle_pub_callback(circles);
  cv::imshow("view", src);


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
    image_transport::Subscriber sub = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, imageCallback);

    //Start node
    rclcpp::spin(node);

    //Close windows again
    cv::destroyWindow("view");
    rclcpp::shutdown();
    return 0;
}