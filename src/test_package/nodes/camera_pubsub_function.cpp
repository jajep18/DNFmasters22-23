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

//Include our own source libraries
#include "../src/vision.cpp"

class Camera_PubSub : public rclcpp::Node
{    
public:
    Camera_PubSub() : Node("camera_pubsub"){
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_cool_ass_topic", 1); //normally 1, queue size qued est 1
        timer_ = this->create_wall_timer(500ms, std::bind(&Camera_PubSub::timer_callback, this));
    }

private:
    // Subscriber
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
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
        } catch (const cv_bridge::Exception & e) {
            auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    //Publisher
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    void timer_callback(){
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

  
}

};

camera_pubsub::camera_pubsub(/* args */) : Node("cam_pubsub")
{   
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("cam_pubsub", options);
    it(node);
    sub = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, imageCallbackCircleDetect);
}


camera_pubsub::~camera_pubsub()
{
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Camera_PubSub>());

    rclcpp::shutdown();
    return 0;
}