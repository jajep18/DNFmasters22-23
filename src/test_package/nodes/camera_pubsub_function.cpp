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


//Include our own source libraries
#include "../src/vision.cpp"


class Camera_PubSub : public rclcpp::Node
{    
public:
    Camera_PubSub() : Node("camera_pubsub"){
    //Camera_PubSub(image_transport::ImageTransport &it) : Node("camera_pubsub"){
    //Camera_PubSub(rclcpp::NodeHandle &nh, image_transport::ImageTransport &it){
        publisher_ = this->create_publisher<std_msgs::msg::String>("cam_circle_topic", 1); //normally 1, queue size qued est 1
        timer_ = this->create_wall_timer(5000ms, std::bind(&Camera_PubSub::timer_callback, this));

        //subscription_ = this->create_subscription<sensor_msgs::msg::Image>("cam_circle_topic", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        // it(this);
        //sub = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, &Camera_PubSub::imageCallback);
        //image_transport::ImageTransport it(this);
        //image_transport::Subscriber sub = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, imageCallback);
        //sub = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, imageCallback);
    }

    void set_subscription(image_transport::ImageTransport &it){
        subscriber_ = it.subscribe("top_down_cam/custom_rgb/image_raw", 1, &Camera_PubSub::imageCallback, this);
        // Subscriber image_transport::ImageTransport::subscribe 	( 	const std::string &  	base_topic,
		// uint32_t  	queue_size,
		// void(T::*)(const sensor_msgs::ImageConstPtr &)  	fp,
		// const boost::shared_ptr< T > &  	obj,
		// const TransportHints &  	transport_hints = TransportHints() 
	// )
    }

private:
    // Subscriber
    image_transport::Subscriber subscriber_;
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
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

    std::vector<cv::Vec3f> circles; // Detected circles

    //Publisher
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;
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

    std::shared_ptr<Camera_PubSub> node = std::make_shared<Camera_PubSub>();
    
    image_transport::ImageTransport it(node);
    node->set_subscription(it);
    
    rclcpp::spin(node);
    //rclcpp::spin(std::make_shared<Camera_PubSub>());
    
    cv::destroyWindow("view");
    rclcpp::shutdown();
    return 0;
}