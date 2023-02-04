#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include "image_transport/image_transport.hpp"
// #include "opencv2/highgui.hpp"
// #include <opencv2/opencv.hpp>
// #include "rclcpp/logging.hpp"
// #include "sensor_msgs/msg/image.hpp"

#include "custom_msgs/msg/circle_info.hpp" //Custom message type for circles
#include "custom_msgs/msg/circle_info_arr.hpp" //Custom message type for array of circles
#include "custom_msgs/msg/circle_info_arr_stereo.hpp" //Custom message type for array of circles for stereo sets

using std::placeholders::_1;

class CircleSubscriber : public rclcpp::Node
{
public:
  CircleSubscriber() : Node("circle_subscriber"){
    subscription_ = this->create_subscription<custom_msgs::msg::CircleInfoArrStereo>(
      "cam_circle_topic", 1, std::bind(&CircleSubscriber::topic_callback, this, _1));

    subscriptionSorted_ = this->create_subscription<custom_msgs::msg::CircleInfoArrStereo>(
      "cam_circle_topic", 1, std::bind(&CircleSubscriber::sort_detections, this, _1));

    
  }

private:
  void topic_callback(const custom_msgs::msg::CircleInfoArrStereo::SharedPtr msg){


    if (msg->left.circles.size() > 0){
        std::string circle_log = "";
        for(size_t i = 0; i<msg->left.circles.size(); i++){
            circle_log += "\n Circle " + std::to_string(i) + ":";
            circle_log += " x=" + std::to_string(msg->left.circles[i].x);
            circle_log += " y=" + std::to_string(msg->left.circles[i].y);
            circle_log += " r=" + std::to_string(msg->left.circles[i].r);
            circle_log += " b/g/r means = " + std::to_string(msg->left.circles[i].bgr_mean[0]) + "/"
                                            + std::to_string(msg->left.circles[i].bgr_mean[1]) + "/"
                                            + std::to_string(msg->left.circles[i].bgr_mean[2]);
            circle_log += " b/g/r var = "   + std::to_string(msg->left.circles[i].bgr_var [0]) + "/"
                                            + std::to_string(msg->left.circles[i].bgr_var [1]) + "/"
                                            + std::to_string(msg->left.circles[i].bgr_var [2]);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->left.circles.size() << " circles!" << circle_log);
    } else {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->left.circles.size() << " circles.");
    }

    if (msg->right.circles.size() > 0){
              std::string circle_log = "";
        for(size_t i = 0; i<msg->right.circles.size(); i++){
            circle_log += "\n Circle " + std::to_string(i) + ":";
            circle_log += " x=" + std::to_string(msg->right.circles[i].x);
            circle_log += " y=" + std::to_string(msg->right.circles[i].y);
            circle_log += " r=" + std::to_string(msg->right.circles[i].r);
            circle_log += " b/g/r means = " + std::to_string(msg->right.circles[i].bgr_mean[0]) + "/"
                                            + std::to_string(msg->right.circles[i].bgr_mean[1]) + "/"
                                            + std::to_string(msg->right.circles[i].bgr_mean[2]);
            circle_log += " b/g/r var = "   + std::to_string(msg->right.circles[i].bgr_var [0]) + "/"
                                            + std::to_string(msg->right.circles[i].bgr_var [1]) + "/"
                                            + std::to_string(msg->right.circles[i].bgr_var [2]);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->right.circles.size() << " circles!" << circle_log);
    } else {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->right.circles.size() << " circles.");
    }

      
      

    // if (msg->circles.size() > 0){
    //     std::string circle_log = "";
    //     for(size_t i = 0; i<msg->circles.size(); i++){
    //         circle_log += "\n Circle " + std::to_string(i) + ":";
    //         circle_log += " x=" + std::to_string(msg->circles[i].x);
    //         circle_log += " y=" + std::to_string(msg->circles[i].y);
    //         circle_log += " r=" + std::to_string(msg->circles[i].r);
    //         circle_log += " b/g/r means = " + std::to_string(msg->circles[i].bgr_mean[0]) + "/"
    //                                         + std::to_string(msg->circles[i].bgr_mean[1]) + "/"
    //                                         + std::to_string(msg->circles[i].bgr_mean[2]);
    //         circle_log += " b/g/r var = "   + std::to_string(msg->circles[i].bgr_var [0]) + "/"
    //                                         + std::to_string(msg->circles[i].bgr_var [1]) + "/"
    //                                         + std::to_string(msg->circles[i].bgr_var [2]);
    //     }
    //     RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles!" << circle_log);
    // } else {
    //     //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    //     RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles.");
    // }
    
  }

  void triangulate(const custom_msgs::msg::CircleInfoArrStereo::SharedPtr msg){

    // Triangulate circles
    // Use one circle for reference and find matching circle in other image 
    // by finding the circle with the same color as the reference circle


    // Find matching circles
    std::vector<custom_msgs::msg::CircleInfo> left_circles = msg->left.circles;
    std::vector<custom_msgs::msg::CircleInfo> right_circles = msg->right.circles;

    // Find reference circle 
    custom_msgs::msg::CircleInfo ref_circle = left_circles[0];

    // Find matching circle in right image
    custom_msgs::msg::CircleInfo match_circle;
    double min_dist = 1000000;
    for(size_t i = 0; i<right_circles.size(); i++){
        double dist = std::sqrt(std::pow(ref_circle.bgr_mean[0] - right_circles[i].bgr_mean[0], 2) + 
                                std::pow(ref_circle.bgr_mean[1] - right_circles[i].bgr_mean[1], 2) + 
                                std::pow(ref_circle.bgr_mean[2] - right_circles[i].bgr_mean[2], 2));
        if (dist < min_dist){
            min_dist = dist;
            match_circle = right_circles[i];
        }
    }

    // Create points for triangulation
    cv::Point ref_point(ref_circle.x, ref_circle.y);
    cv::Point match_point(match_circle.x, match_circle.y);

    // Create matrix for triangulation from points
    cv::Mat qs = (cv::Mat_<double>(4,1) << ref_point.x, ref_point.y, match_point.x, match_point.y);

    // Triangulate 
    // Follow this: https://answers.opencv.org/question/117141/triangulate-3d-points-from-a-stereo-camera-and-chessboard/
    // cv::triangulatePoints(projMat1, projMat2, undistCoords1, undistCoords2, triangCoords4D);
    cv::TriangulatePoints(qs, 



    

    
    




    

    

  }

  rclcpp::Subscription<custom_msgs::msg::CircleInfoArrStereo>::SharedPtr subscription_;
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArrStereo>::SharedPtr subscriptionSorted_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleSubscriber>());
  


  rclcpp::shutdown();
  return 0;
}
