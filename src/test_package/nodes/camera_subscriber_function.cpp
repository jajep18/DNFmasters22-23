#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <vector>
#include <iostream>



using std::placeholders::_1; // MIght not be necessary

std::vector<cv::Vec3f> detect_circles(cv::Mat &image){

    std::vector<cv::Vec3f> circles;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 25,
                  1, 30 // (min_radius & max_radius) to detect larger circles
    );
 
    //Draw center and outline of all detected circles 
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);

        // circle center
        cv::circle( image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        
        // circle outline
        int radius = c[2];
        cv::circle( image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }
  //Return detected circles

  return circles;

}


void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) //Image::ConstSharedPtr & msg //Image::SharedPtr  msg
{

  std::vector<cv::Vec3f> circles;
  cv::Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
  try {

    circles = detect_circles(src);
    cv::imshow("view", src);
    // cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  cv::putText(src, //target image
            "Detected " + std::to_string(circles.size() ) + " circles" , //text
            cv::Point(10, src.rows / 10), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            0.6,
            CV_RGB(225, 255, 255), //font color
            2);

  cv::imshow("view", src);


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
    image_transport::Subscriber sub = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, imageCallback);

    //Start node
    rclcpp::spin(node);

    //Close windows again
    cv::destroyWindow("view");
    rclcpp::shutdown();

    return 0;
}