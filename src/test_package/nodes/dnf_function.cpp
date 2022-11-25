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

using std::placeholders::_1;