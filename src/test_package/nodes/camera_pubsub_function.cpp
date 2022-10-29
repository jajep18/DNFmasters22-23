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
using namespace std::chrono_literals; //Ugly, yet necessarry
#include <string>


//Include our own source libraries
#include "../src/vision.cpp"


class Camera_PubSub : public rclcpp::Node
{    
public:
    Camera_PubSub() : Node("camera_pubsub"){
        publisher_    = this->create_publisher<std_msgs::msg::String>("cam_circle_topic", 1); //normally 1, queue size qued est 1
        timer_        = this->create_wall_timer(5000ms, std::bind(&Camera_PubSub::timer_callback, this));
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("top_down_cam/custom_rgb/image_raw", 
                                                                            1, 
                                                                            std::bind(&Camera_PubSub::topic_callback, 
                                                                            this, 
                                                                            std::placeholders::_1));
    }

private:
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_image)
    {
        cv::Mat src = cv_bridge::toCvShare(msg_image, "bgr8")->image;
        try {
            circles = detect_circles(src);
            cv::putText(src, //target image
                "Detected " + std::to_string(circles.size() ) + " circles" ,    // Text
                cv::Point(10, src.rows / 10),   // Top-left position
                cv::FONT_HERSHEY_DUPLEX,
                0.6,
                CV_RGB(225, 255, 255),  // Font color
                2);
            cv::imshow("view", src);
        } catch (const cv_bridge::Exception & e) {
            auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not convert from " << msg_image->encoding.c_str() << " to 'bgr8'.");
        }
    }

    std::vector<cv::Vec3f> circles; // Detected circles

    //Publisher
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;
    std::string circleStr;
    void timer_callback(){
        auto message = std_msgs::msg::String();
        message.data = "Detected " + std::to_string(circles.size() ) + " circles!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Create CV Window    
    cv::namedWindow("view");
    cv::startWindowThread();
    
    //rclcpp::spin(node);
    rclcpp::spin(std::make_shared<Camera_PubSub>());
    
    cv::destroyWindow("view");
    rclcpp::shutdown();
    return 0;
}