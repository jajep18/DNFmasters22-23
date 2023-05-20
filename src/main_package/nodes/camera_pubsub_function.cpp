//Always needed
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/logging.hpp" //For "get_logger"
//Includes for subscribing 
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
//Includes for publishing
#include <chrono> //for 500ms
#include "custom_msgs/msg/circle_info.hpp" //Custom message type for circles
#include "custom_msgs/msg/circle_info_arr.hpp" //Custom message type for array of circles
#include "custom_msgs/msg/circle_info_arr_stereo.hpp" //Custom message type for array of circles for stereo sets

#include <string>


//Include our own source libraries
#include "../src/vision.cpp"
using namespace std::chrono_literals; //Ugly, yet necessarry


class Camera_PubSub : public rclcpp::Node
{    
public:
    Camera_PubSub() : Node("camera_pubsub"){
        //publisher_    = this->create_publisher<std_msgs::msg::String>("cam_circle_topic", 1); //normally 1, queue size qued est 1
        publisher_    = this->create_publisher<custom_msgs::msg::CircleInfoArrStereo>("cam_circle_topic", 1); //normally 1, queue size qued est 1
        timer_        = this->create_wall_timer(8000ms, std::bind(&Camera_PubSub::timer_callback, this));
        subscriptionRightCam_ = this->create_subscription<sensor_msgs::msg::Image>("custom_ns/custom_camera/right/custom_image", 
                                                                            1, 
                                                                            std::bind(&Camera_PubSub::topic_callback_right_cam, 
                                                                            this, 
                                                                            std::placeholders::_1));
        subscriptionLeftCam_ = this->create_subscription<sensor_msgs::msg::Image>("custom_ns/custom_camera/left/custom_image", 
                                                                            1, 
                                                                            std::bind(&Camera_PubSub::topic_callback_left_cam, 
                                                                            this, 
                                                                            std::placeholders::_1));

        // stereo_cam_callback();



    }

private:
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionLeftCam_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionRightCam_;
    
    //std::vector<cv::Vec6f> circles; // Detected circles [ x y r B G R]
    custom_msgs::msg::CircleInfoArr m_circleInfoArr_left;
    custom_msgs::msg::CircleInfoArr m_circleInfoArr_right;



    void topic_callback_right_cam(const sensor_msgs::msg::Image::SharedPtr msg_image)
    {
        cv::Mat src = cv_bridge::toCvShare(msg_image, "bgr8")->image;
        try {
            // src.copyTo(right);
            m_circleInfoArr_right = detect_circles(src);
            cv::imshow("Right cam", src);
        } catch (const cv_bridge::Exception & e) {
            auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not convert from " << msg_image->encoding.c_str() << " to 'bgr8'.");
        }
    }

    void topic_callback_left_cam(const sensor_msgs::msg::Image::SharedPtr msg_image)
    {
        cv::Mat src = cv_bridge::toCvShare(msg_image, "bgr8")->image;
        
        try {
            src.copyTo(src);
            m_circleInfoArr_left = detect_circles(src);
            cv::imshow("Left cam", src);
        } catch (const cv_bridge::Exception & e) {
            auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not convert from " << msg_image->encoding.c_str() << " to 'bgr8'.");
        }
    }

    // void stereo_cam_callback()
    // {
    //     // cv::Mat left = cv_bridge::toCvShare(left_cam, "bgr8")->image;
    //     // cv::Mat right = cv_bridge::toCvShare(right_cam, "bgr8")->image;
        
    //     cv::namedWindow("Left view");
    //     cv::namedWindow("Right view");

    //     // m_circleInfoArr = detect_circles(src);
    //     if(!left.empty())
    //         cv::imshow("Left cam", left);
    //     if(!right.empty())
    //         cv::imshow("Right cam", right);
   

    // }

    // void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg_camera_info)
    // {
    //     std::string K_matrix = msg_camera_info->K.data();
        
    //     try {
    //         RCLPP_INFO(this->get_logger(), "K: '%s'", K_matrix);
    //     } catch (const cv_bridge::Exception & e) {
    //         auto logger = rclcpp::get_logger("my_subscriber");
    //         RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find K.");
    //     }
    // }

    //Publisher
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::CircleInfoArrStereo>::SharedPtr publisher_;
    size_t count_ = 0;

    
    void timer_callback(){

        auto stereoPoints = custom_msgs::msg::CircleInfoArrStereo();
        stereoPoints.left = m_circleInfoArr_left;
        stereoPoints.right = m_circleInfoArr_right;



        // auto message = m_circleInfoArr;


        //auto message = custom_msgs::msg::CircleInfoArr();
        // for(size_t i = 0; i < circles.size(); i++){
        //     auto circle = custom_msgs::msg::CircleInfo();
        //     circle.x        = circles[i][0];
        //     circle.y        = circles[i][1];
        //     circle.r        = circles[i][2];
        //     // circle.bgr[0]   = circles[i][3];
        //     // circle.bgr[1]   = circles[i][4];
        //     // circle.bgr[2]   = circles[i][5];
        //     message.circles.push_back(circle);
        // }
        //message.data = "Detected " + std::to_string(circles.size() ) + " circles!";
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // RCLCPP_INFO_STREAM(this->get_logger(), "hello" << " world" << i);
        publisher_->publish(stereoPoints);
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Create CV Window    
    // cv::namedWindow("view");
    cv::startWindowThread();
    
    //rclcpp::spin(node);
    rclcpp::spin(std::make_shared<Camera_PubSub>());
    
    // cv::destroyWindow("view");
    rclcpp::shutdown();
    return 0;
}