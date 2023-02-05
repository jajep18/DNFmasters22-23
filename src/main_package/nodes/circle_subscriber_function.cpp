#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
// #include "opencv2/highgui.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>



#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "custom_msgs/msg/circle_info.hpp" //Custom message type for circles
#include "custom_msgs/msg/circle_info_arr.hpp" //Custom message type for array of circles
#include "custom_msgs/msg/circle_info_arr_stereo.hpp" //Custom message type for array of circles for stereo sets

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


  // cv::Mat computeProjMat(cv::Mat camMat, std::vector<cv::Mat> rotVec, std::vector<cv::Mat> transVec)
  // {
  //   cv::Mat rotMat(3, 3, CV_64F), RTMat(3, 4, CV_64F);
  //   //1. Convert rotation vector into rotation matrix 
  //   cv::Rodrigues(rotVec.at(0), rotMat);
  //   //2. Append translation vector to rotation matrix
  //   cv::hconcat(rotMat, transVec.at(0), RTMat);
  //   //3. Compute projection matrix by multiplying intrinsic parameter 
  //   //matrix (A) with 3 x 4 rotation and translation pose matrix (RT).
  //   return (camMat * RTMat);
  // }


  void triangulate(const custom_msgs::msg::CircleInfoArrStereo::SharedPtr msg){ // Bug somewhere here

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

    // Find reference circle 
    custom_msgs::msg::CircleInfo ref_circle = left_circles[1];

    // Find matching circle in right image
    int match_idx = -1;
    custom_msgs::msg::CircleInfo match_circle;
    double min_dist = 1000000;
    for(size_t i = 0; i<right_circles.size(); i++){
        double dist = std::sqrt(std::pow(ref_circle.bgr_mean[0] - right_circles[i].bgr_mean[0], 2) + 
                                std::pow(ref_circle.bgr_mean[1] - right_circles[i].bgr_mean[1], 2) + 
                                std::pow(ref_circle.bgr_mean[2] - right_circles[i].bgr_mean[2], 2));
        if (dist < min_dist){
            min_dist = dist;
            match_circle = right_circles[i];
            match_idx = i;
        }
    }

    // RCLCPP_INFO_STREAM(this->get_logger(), "Found matching circle with index " << match_idx << " and distance " << min_dist);

    // Create points for triangulation
    cv::Point ref_point(ref_circle.x, ref_circle.y);
    cv::Point match_point(match_circle.x, match_circle.y);

    // Create matrix for triangulation from points
    cv::Mat qs = (cv::Mat_<int>(4,1) << ref_point.x, ref_point.y, match_point.x, match_point.y);

    // Read camera matrix from camera info left
    // cv::Mat proj_mat_left = (cv::Mat_<double>(3,4) << camera_info_left->p[0], camera_info_left->p[1], camera_info_left->p[2], camera_info_left->p[3],
    //                                             camera_info_left->p[4], camera_info_left->p[5], camera_info_left->p[6], camera_info_left->p[7],
    //                                             camera_info_left->p[8], camera_info_left->p[9], camera_info_left->p[10], camera_info_left->p[11]);

    // // Read camera matrix from camera info right
    // cv::Mat proj_mat_right = (cv::Mat_<double>(3,4) << camera_info_right->p[0], camera_info_right->p[1], camera_info_right->p[2], camera_info_right->p[3],
    //                                             camera_info_right->p[4], camera_info_right->p[5], camera_info_right->p[6], camera_info_right->p[7],
    //                                             camera_info_right->p[8], camera_info_right->p[9], camera_info_right->p[10], camera_info_right->p[11]);

    // Define rotation matrix and translation vector
    std::vector<cv::Mat> rotVec;
    std::vector<cv::Mat> transVec;
    cv::Mat TMat_left(3, 1, CV_64F), TMat_right(3, 1, CV_64F);
    rotVec.push_back(cv::Mat::eye(3, 3, CV_64F));
    TMat_left = (cv::Mat_<double>(3,1) << 0, 0, 2.3);
    TMat_right = (cv::Mat_<double>(3,1) << 0.2, 0, 2.3);

    transVec.push_back(TMat_left);
    transVec.push_back(TMat_right);

    // Debug print
    // RCLCPP_INFO_STREAM(this->get_logger(), "Debug - post transVec");

    // Define camera matrix from camera info
    cv::Mat camMat_left = (cv::Mat_<double>(3,3) << camera_info_left->k[0], camera_info_left->k[1], camera_info_left->k[2],
                                                camera_info_left->k[3], camera_info_left->k[4], camera_info_left->k[5],
                                                camera_info_left->k[6], camera_info_left->k[7], camera_info_left->k[8]);

    cv::Mat camMat_right = (cv::Mat_<double>(3,3) << camera_info_right->k[0], camera_info_right->k[1], camera_info_right->k[2],
                                                camera_info_right->k[3], camera_info_right->k[4], camera_info_right->k[5],
                                                camera_info_right->k[6], camera_info_right->k[7], camera_info_right->k[8]);

    
      // Debug print
    // RCLCPP_INFO_STREAM(this->get_logger(), "Debug - post camMat");

    
    // Compute projection matrices
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
    



    // RCLCPP_INFO_STREAM(this->get_logger(), "Debug - post projMat");

    // Debug print


    RCLCPP_INFO_STREAM(this->get_logger(), "Read left camera info: " << proj_mat_left);
    RCLCPP_INFO_STREAM(this->get_logger(), "Read right camera info: " << proj_mat_right);

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

    RCLCPP_INFO_STREAM(this->get_logger(), "left_point_mat: " << left_point_mat);
    RCLCPP_INFO_STREAM(this->get_logger(), "right_point_mat: " << right_point_mat);

    

    // RCLCPP_INFO_STREAM(this->get_logger(), "Triangulating points.");

    //Triangulate
    // cv::triangulatePoints(proj_mat_left, proj_mat_right, left_points, right_points, triangCoords4D);

    //Create 4d matrix for triangulation
    cv::Mat triangCoords4D(4, 1, CV_64F);

    cv::triangulatePoints(proj_mat_left, proj_mat_right, left_point_mat, right_point_mat, triangCoords4D);


    
    // Unhomogenize coordinates
    cv::Mat triangCoords3D = triangCoords4D.rowRange(0,3) / triangCoords4D.row(3);

    // Print coordinates
    RCLCPP_INFO_STREAM(this->get_logger(), "Triangulated coordinates for left[0] and right " << match_idx << " is " << triangCoords3D);
       

  }
  // Subscribers
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArrStereo>::SharedPtr subscription_;
  rclcpp::Subscription<custom_msgs::msg::CircleInfoArrStereo>::SharedPtr subscriptionTriangulate_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_left_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_right_;

  

  // Camera info
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_left;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_right;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleSubscriber>());
  


  rclcpp::shutdown();
  return 0;
}
