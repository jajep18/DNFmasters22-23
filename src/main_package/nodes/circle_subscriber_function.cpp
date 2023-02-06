#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
// #include "opencv2/highgui.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>

#include <math.h>

#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "custom_msgs/msg/circle_info.hpp" //Custom message type for circles
#include "custom_msgs/msg/circle_info_arr.hpp" //Custom message type for array of circles
#include "custom_msgs/msg/circle_info_arr_stereo.hpp" //Custom message type for array of circles for stereo sets
#include "custom_msgs/msg/triangulated_circle_info.hpp" //Custom message type for triangulated of circles for stereo sets
#include "custom_msgs/msg/triangulated_circle_info_arr.hpp" //Custom message type for array of triangulated circles for stereo sets

using std::placeholders::_1;

class CircleSubscriber : public rclcpp::Node
{
public:
  CircleSubscriber() : Node("circle_subscriber"){

    // Camera info subscribers
    camera_info_subscriber_left_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/custom_ns/custom_camera/left/custom_camera_info", 1, std::bind(&CircleSubscriber::camera_info_callback_left, this, _1));

    camera_info_subscriber_right_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/custom_ns/custom_camera/right/custom_camera_info", 1, std::bind(&CircleSubscriber::camera_info_callback_right, this, _1));

    // Circle info subscribers
    subscription_ = this->create_subscription<custom_msgs::msg::CircleInfoArrStereo>(
      "cam_circle_topic", 1, std::bind(&CircleSubscriber::topic_callback, this, _1));

    subscriptionTriangulate_ = this->create_subscription<custom_msgs::msg::CircleInfoArrStereo>(
      "cam_circle_topic", 1, std::bind(&CircleSubscriber::triangulate, this, _1));

    publisher_    = this->create_publisher<custom_msgs::msg::TriangulatedCircleInfoArr>("triangulated_circle_topic", 1);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&CircleSubscriber::publish, this));

    
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
  }

  void camera_info_callback_left(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
    // Get camera info  
    // RCLCPP_INFO_STREAM(this->get_logger(), "Got left camera info.");
    camera_info_left = msg;
  }

  void camera_info_callback_right(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
    // Get camera info  
    // RCLCPP_INFO_STREAM(this->get_logger(), "Got right camera info.");
    camera_info_right = msg;
  }

  // int findIdx(std::vector<auto> input_vec, auto query  ){ // Move later
  //   // Find index of query in input_vec
  //   // Returns -1 if not found
  //   auto it = std::find(input_vec.begin(), input_vec.end(), query);
  //   if (it != input_vec.end()){
  //       return std::distance(input_vec.begin(), it);
  //   }
  //   else{
  //       return -1;
  //   }
  // }

  void triangulate(const custom_msgs::msg::CircleInfoArrStereo::SharedPtr msg){

    // Triangulate circles
    // Use one circle for reference and find matching circle in other image 
    // by finding the circle with the same color as the reference circle

    if(msg->left.circles.size() == 0 || msg->right.circles.size() == 0){
        RCLCPP_INFO_STREAM(this->get_logger(), "No circles found.");
        return;
    }
    else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Triangulating circles.");
    }

  
    // Find matching circles
    std::vector<custom_msgs::msg::CircleInfo> left_circles = msg->left.circles;
    std::vector<custom_msgs::msg::CircleInfo> right_circles = msg->right.circles;


    // Check for lowest number of detected circles
    int min_num_circles = std::min(left_circles.size(), right_circles.size());

    // Loop through all circles
    std::vector<cv::Mat> triangulated_circles_points3d; // 3D points from triangulation
    std::vector<int> triangulated_circles_idx; // Index of circle in right image
    std::vector<std::string> triangulated_circles_color; // Color of circle

    for (int i = 0; i < min_num_circles; i++)
    {
    
      // Find reference circle 
      custom_msgs::msg::CircleInfo ref_circle = left_circles[i];

      // Find matching circle in right image
      int match_idx = -1;
      custom_msgs::msg::CircleInfo match_circle;
      double min_dist = 1000000;
      for(size_t j = 0; j<right_circles.size(); j++){
          double dist = std::sqrt(std::pow(ref_circle.bgr_mean[0] - right_circles[j].bgr_mean[0], 2) + 
                                  std::pow(ref_circle.bgr_mean[1] - right_circles[j].bgr_mean[1], 2) + 
                                  std::pow(ref_circle.bgr_mean[2] - right_circles[j].bgr_mean[2], 2));
          if (dist < min_dist){
              min_dist = dist;
              match_circle = right_circles[j];
              match_idx = j;
          }
      }

      std::vector<int> mean_color;
      mean_color.push_back( round( (ref_circle.bgr_mean[0] + match_circle.bgr_mean[0])/2 ) );
      mean_color.push_back( round( (ref_circle.bgr_mean[1] + match_circle.bgr_mean[1])/2 ) );
      mean_color.push_back( round( (ref_circle.bgr_mean[2] + match_circle.bgr_mean[2])/2 ) );
      
      
      //std::string color = "unknown";
      // if (mean_color[2] > mean_color[1] ){ // Check blue against red and green
      //   if (mean_color[2] > mean_color[2]){ //Check blue against red
      //     color = "red";
      //   }
      //   else{
      //     color = "blue";
      //   }
      // }
      // else if (mean_color[1] > mean_color[0]){ // Check green against blue
      //   color = "green";
      // }
      // else{
      //   color = "blue";
      // }

      // Determine color of circles
      // int color = std::max( begin(mean_color), end(mean_color) );

      auto color_idx = max_element(std::begin(mean_color), std::end(mean_color));

      // int color_idx = mean_color.find(begin(mean_color), end(mean_color), color);
      // int color_idx = std::find(mean_color, color); //mean_color[0], mean_color[1], mean_color[2]);
      // int color_idx = std::where(color == std::max(mean_color[0], mean_color[1], mean_color[2]));
      // int color_idx = findIdx(mean_color, color);
      
      // Create points for triangulation
      cv::Point ref_point(ref_circle.x, ref_circle.y);
      cv::Point match_point(match_circle.x, match_circle.y);

      // Create matrix for triangulation from points
      cv::Mat qs = (cv::Mat_<int>(4,1) << ref_point.x, ref_point.y, match_point.x, match_point.y);

      // Define rotation matrix and translation vector
      std::vector<cv::Mat> rotVec;
      std::vector<cv::Mat> transVec;
      cv::Mat TMat_left(3, 1, CV_64F), TMat_right(3, 1, CV_64F);
      rotVec.push_back(cv::Mat::eye(3, 3, CV_64F));
      TMat_left = (cv::Mat_<double>(3,1) << 0, 0, 2.3);
      TMat_right = (cv::Mat_<double>(3,1) << 0.2, 0, 2.3);

      transVec.push_back(TMat_left);
      transVec.push_back(TMat_right);

      // Define camera matrix from camera info
      cv::Mat camMat_left = (cv::Mat_<double>(3,3) << camera_info_left->k[0], camera_info_left->k[1], camera_info_left->k[2],
                                                  camera_info_left->k[3], camera_info_left->k[4], camera_info_left->k[5],
                                                  camera_info_left->k[6], camera_info_left->k[7], camera_info_left->k[8]);

      cv::Mat camMat_right = (cv::Mat_<double>(3,3) << camera_info_right->k[0], camera_info_right->k[1], camera_info_right->k[2],
                                                  camera_info_right->k[3], camera_info_right->k[4], camera_info_right->k[5],
                                                  camera_info_right->k[6], camera_info_right->k[7], camera_info_right->k[8]);
      
      // Compute projection matrices
      // Translation matrix is known for the simulated cameras
      // If real camera is used remeeber to change the translation matrix to the correct value from calibration
      cv::Mat proj_mat_left = camMat_left * (cv::Mat_<double>(3,4) << 
                                                  1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 2.3
                                                  );
      cv::Mat proj_mat_right = camMat_right * (cv::Mat_<double>(3,4) << 
                                                  1, 0, 0, 0.2,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 2.3
                                                  );      
      

      // RCLCPP_INFO_STREAM(this->get_logger(), "Read left camera info: " << proj_mat_left);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Read right camera info: " << proj_mat_right);

      // RCLCPP_INFO_STREAM(this->get_logger(), "Read left camera info: " << projMat1);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Read right camera info: " << camera_info_right->header.frame_id);

      // Triangulate 
      // Follow this: https://answers.opencv.org/question/117141/triangulate-3d-points-from-a-stereo-camera-and-chessboard/

      cv::Mat left_point_mat(2, 1, CV_64F);
      cv::Mat right_point_mat(2 ,1 , CV_64F);

      left_point_mat.at<double>(0,0) = ref_point.x;
      left_point_mat.at<double>(1,0) = ref_point.y;

      right_point_mat.at<double>(0,0) = match_point.x;
      right_point_mat.at<double>(1,0) = match_point.y;

      // RCLCPP_INFO_STREAM(this->get_logger(), "left_point_mat: " << left_point_mat);
      // RCLCPP_INFO_STREAM(this->get_logger(), "right_point_mat: " << right_point_mat);

      //Create 4d matrix for triangulation
      cv::Mat triangCoords4D(4, 1, CV_64F);

      cv::triangulatePoints(proj_mat_left, proj_mat_right, left_point_mat, right_point_mat, triangCoords4D);


      
      // Unhomogenize coordinates
      cv::Mat triangCoords3D = triangCoords4D.rowRange(0,3) / triangCoords4D.row(3);

      triangulated_circles_points3d.push_back(triangCoords3D);
      triangulated_circles_idx.push_back(match_idx);      
      triangulated_circles_color.push_back(color_dict[(color_idx - mean_color.begin())]);

      
    }



    


    // // Print triangulated points
    // for (size_t j = 0; j < triangulated_circles_points3d.size(); j++)
    // {
    //   RCLCPP_INFO_STREAM(this->get_logger(), "Triangulated circle " << "of color: " << triangulated_circles_color[j] << "(Index: " << j 
    //                     << ")" << " to "<< triangulated_circles_idx[j] << ": " << triangulated_circles_points3d[j]);
    // }

    // Publish triangulated points
    for (size_t i = 0; i < triangulated_circles_points3d.size() ; i++)
    {
      custom_msgs::msg::TriangulatedCircleInfo triangulated_circles_msg;
      triangulated_circles_msg.color = triangulated_circles_color[i];
      triangulated_circles_msg.x = triangulated_circles_points3d[i].at<double>(0,0);
      triangulated_circles_msg.y = triangulated_circles_points3d[i].at<double>(1,0);

      triangulated_circles_arr_msg.circles.push_back(triangulated_circles_msg);
      
    }

        

  }

  void publish()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing triangulated circles");
    publisher_->publish(triangulated_circles_arr_msg);
  }
  // Subscribers
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArrStereo>::SharedPtr subscription_;
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArrStereo>::SharedPtr subscriptionTriangulate_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_left_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_right_;

  // ************ Publishers ************
  rclcpp::Publisher<custom_msgs::msg::TriangulatedCircleInfoArr>::SharedPtr publisher_;
  //Publisher
  rclcpp::TimerBase::SharedPtr timer_;

  // ************ Messages ************
  custom_msgs::msg::TriangulatedCircleInfoArr triangulated_circles_arr_msg;  

  // Camera info
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_left;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_right;

  std::vector<std::string> color_dict = {"blue", "green", "red", "unknown"};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleSubscriber>());
  


  rclcpp::shutdown();
  return 0;
}
