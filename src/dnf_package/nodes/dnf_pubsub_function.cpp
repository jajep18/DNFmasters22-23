/* 
 * Description:     Node handling the decision making and DNF architecture
 *
 * Author:			    Erik Lindby
 *                  Jacob Floe Jeppesen
 *					        University of Southern Denmark
 * Creation date:   01-11-2022
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/logging.hpp" //For "get_logger"
#include <iostream>
#include "../src/dnf_lib.cpp"
#include <torch/torch.h>

#include <ament_index_cpp/get_package_share_directory.hpp> //For getting the package path
// may throw ament_index_cpp::PackageNotFoundError exception
#include <filesystem> //For filesystem get_path current purposes

// Custom messages
#include "std_msgs/msg/int8_multi_array.hpp"
#include "custom_msgs/msg/circle_info.hpp"
#include "custom_msgs/msg/circle_info_arr.hpp"
#include "custom_msgs/msg/triangulated_circle_info_arr.hpp"
#include "custom_msgs/srv/decision.hpp"

// Local includes
#include "../src/actions.cpp"
#include "../include/dnf_1d.hpp"
#include "../include/dnf_2d.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DNFNode : public rclcpp::Node
{
public:
  DNFNode() : Node("dnf_pubsub"),
    // Member initialization list
    keywords_dnf(20,true),    //Contains the input keywords in the command. This is the (true) input of the network
    color_circles_dnf(3,true),//Contains the color of the circles. This is an input
    pos_x_circle_dnf(3,true), //Contains the x position of the circles. This is an input
    pos_y_circle_dnf(3,true), //Contains the y position of the circles. This is an input
    action_dnf(5,true),       //Contains the actions, this is the output of the network
    keywords_color_dnf(20,3,true)//Combined DNF Keywords+Color½int index_input, int index_element,
  {
    // Create a subscriber to the detected circles
    circle_subscription_ = this->create_subscription<custom_msgs::msg::TriangulatedCircleInfoArr>(
      "triangulated_circle_topic", 1, std::bind(&DNFNode::circle_callback, this, _1));

    keyword_subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
      "keywords", 1, std::bind(&DNFNode::keyword_callback, this, _1));

    // Create service client for the movement command center to recieve decisions on actions and targets
    // decision_service_ = this->create_service<custom_msgs::srv::Decision>("decision_service", &DNFNode::decision_service_callback);
    decision_service_ = this->create_service<custom_msgs::srv::Decision>("/DNF/decision", std::bind(&DNFNode::decision_service_callback, this, _1, _2));

    // Initialize DNFs
    //DNFinit();
    get_log_path();

    //write2DTensorCSV(keywords_color_dnf.get_output(), log_path, "keywords_color_dnf_output.csv");
    // torch::Tensor keywords_color_dnf_output = read2DTensorCSV(log_path, "keywords_color_dnf_output.csv");
    // keywords_color_dnf.set_output(keywords_color_dnf_output);
    // //Write it back to a file to check if it is the same
    // //printTensor(keywords_color_dnf.get_output());
    // write2DTensorCSV(keywords_color_dnf.get_output(), log_path, "test.csv");

    // - - - - - - - - - - - - - - - - - - - Test the DNF - - - - - - - - - - - - - - - - - - - 
    //Set input of keywords
    keywords_dnf.set_input_element(4, 6.9f);
    write2DTensorCSV(keywords_dnf.get_input(), log_path, "keywords_dnf_input.csv");

    //Set input of color
    color_circles_dnf.set_input_element(0, 6.9f);
    write2DTensorCSV(color_circles_dnf.get_input(), log_path, "color_circles_dnf_input.csv");

    //Cross the Keyword and Color DNFS
    keywords_color_dnf.set_input(keywords_dnf.get_input(), color_circles_dnf.get_input());
    keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 1.0f);
    write2DTensorCSV(keywords_color_dnf.get_activation(), log_path, "keywords_color_dnf_activation.csv");

    //Attempt to extract the keywords back from the Keyword/Color 2D DNF
    torch::Tensor keywords_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    write2DTensorCSV(keywords_response, log_path, "color_response_KWxC.csv");

    // - - - - - - - - - - - - - - - - - - - Keyword x Color test 2 - - - - - - - - - - - - - - - - - - - 
    // Learning / Memorization phase - - - - - - - - 
    // Learning red
    keywords_dnf.reset_input();
    color_circles_dnf.reset_input();
    keywords_dnf.set_input_element(4, 6.9f); //"Red"
    color_circles_dnf.set_input_element(0, 6.9f); // The red color
    keywords_color_dnf.set_input(keywords_dnf.get_input(), color_circles_dnf.get_input()); //Set input for the 2D DNF
    keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 0.5f); //Get it into memory

    // Learning green
    keywords_dnf.reset_input();
    color_circles_dnf.reset_input();
    keywords_dnf.set_input_element(5, 6.9f);
    color_circles_dnf.set_input_element(1, 6.9f); // The green color
    keywords_color_dnf.set_input(keywords_dnf.get_input(), color_circles_dnf.get_input());
    keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 0.5f); //Get it into memory

    // Learning blue
    keywords_dnf.reset_input();
    color_circles_dnf.reset_input();
    keywords_dnf.set_input_element(6, 6.9f);
    color_circles_dnf.set_input_element(2, 6.9f); // The blue color
    keywords_color_dnf.set_input(keywords_dnf.get_input(), color_circles_dnf.get_input());
    keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 0.5f); //Get it into memory

    // Save KWxC DNF after learning
    write2DTensorCSV(keywords_color_dnf.get_activation(), log_path, "keywords_color_dnf_activation_after_learning.csv");

    // Memory extraction / Remembering phase - - - - - - - -
    // Remembering red
    keywords_dnf.reset_input();
    keywords_dnf.set_input_element(4, 6.9f); //"Red"
    torch::Tensor red_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    write2DTensorCSV(red_response, log_path, "red_response.csv");

    // Remembering green
    keywords_dnf.reset_input();
    keywords_dnf.set_input_element(5, 6.9f); //"Green"
    torch::Tensor green_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    write2DTensorCSV(green_response, log_path, "green_response.csv");

    // Remembering blue
    keywords_dnf.reset_input();
    keywords_dnf.set_input_element(6, 6.9f); //"Blue"
    torch::Tensor blue_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    write2DTensorCSV(blue_response, log_path, "blue_response.csv");

    // Remember / Extract for a word that has not been learned
    keywords_dnf.reset_input();
    keywords_dnf.set_input_element(17, 6.9f); //"Away"
    torch::Tensor dummy_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    write2DTensorCSV(dummy_response, log_path, "dummy_response.csv");

    // Remember / Extract for several words that have been learned
    keywords_dnf.reset_input();
    keywords_dnf.set_input_element(4, 6.9f); //"Red"
    keywords_dnf.set_input_element(5, 6.9f); //"Green"
    torch::Tensor red_green_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    write2DTensorCSV(red_green_response, log_path, "red_green_response.csv");

    // Remember / Extract for a phrase containing one learned word
    keywords_dnf.reset_input();
    keywords_dnf.set_input_element(0, 6.9f); //"Move"
    keywords_dnf.set_input_element(1, 6.9f); //"The"
    keywords_dnf.set_input_element(4, 6.9f); //"Red"
    keywords_dnf.set_input_element(2, 6.9f); //"Ball"
    keywords_dnf.set_input_element(8, 6.9f); //"Right"
    torch::Tensor red_phrase_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    write2DTensorCSV(red_phrase_response, log_path, "red_phrase_response.csv");
  }

private:

  void decision_service_callback(const std::shared_ptr<custom_msgs::srv::Decision::Request> request,
                                 std::shared_ptr<custom_msgs::srv::Decision::Response> response ){
    RCLCPP_INFO(this->get_logger(), "Received request with requestid: [%d]", request->requestid);
    //int requestid = request->requestid;
    response->actions = 1 + request->requestid;
    response->targets = 0;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Sending back response: [%d] [%d]", response->actions, response->targets);
  }

  void keyword_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg ){
    if ( msg->data.size() > 0){
      keywords_dnf.reset_input();
      // Set the keywords
      for( size_t i = 0; i < msg->data.size(); i++){
        keywords_dnf.set_input_element(msg->data[i], 69); // Set the places of the keywords to 69 for debugging purposes
      }

      // Print the keywords
      RCLCPP_INFO(this->get_logger(), "Recieved keywords: %d", msg->data.size());
      RCLCPP_INFO_STREAM(this->get_logger(), "Keywords: " << '\n' << keywords_dnf.get_input());

    }
    return;
  }


  void circle_callback(const custom_msgs::msg::TriangulatedCircleInfoArr::SharedPtr msg){
    if (msg->circles.size() > 0){

        for(size_t i = 0; i<msg->circles.size(); i++){
          detected_circles.push_back(msg->circles[i]);

        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles!");
    } else {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles.");
    }
    
  }

  void get_log_path(){    
    //This function gets the path to the log directory "dnf_logs"
    //This should get the path without depending on the projects local directory hardcoded
    //Done in a very roundabout manner, but nothing else would compile because of torch linking issues >:(
    std::string dnf_log_path = ament_index_cpp::get_package_share_directory("dnf_package");
    std::string::size_type i = dnf_log_path.find("install/dnf_package/share/dnf_package");
    if (i != std::string::npos)
        dnf_log_path.erase(i, dnf_log_path.size());
    dnf_log_path.append("src/dnf_package/dnf_logs/");
    RCLCPP_INFO(this->get_logger(), "Logs will be written to: %s", dnf_log_path.c_str());
    log_path = dnf_log_path;

    //TODO: Combine this path with the appended path to the log directory, to get the full path
    // Cleaner way than what we have know
    //RCLCPP_INFO(this->get_logger(), "Filesystem: Current path: %s", std::filesystem::current_path().c_str());
    //std::filesystem::current_path() 
  }

  

  // Member variables -------------------
  std::string log_path;
  // Subscriptions
  rclcpp::Subscription<custom_msgs::msg::TriangulatedCircleInfoArr>::SharedPtr circle_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr keyword_subscription_;
  // Service
  rclcpp::Service<custom_msgs::srv::Decision>::SharedPtr decision_service_;

  // Circles
  std::vector<custom_msgs::msg::TriangulatedCircleInfo> detected_circles; //Contains the detected circles {x, y, color}

  // DNFs
  DNF_1D keywords_dnf;      //Contains the input keywords in the command. This is the (true) input of the network
  DNF_1D color_circles_dnf;  //Contains the color of the circles. This is an input
  //TODO: Create 2D DNFs for the position of the circles
  DNF_1D pos_x_circle_dnf;   //Contains the x position of the circles. This is an input
  DNF_1D pos_y_circle_dnf;   //Contains the y position of the circles. This is an input
  DNF_1D action_dnf;         //Contains the actions, this is the output of the network

  DNF_2D keywords_color_dnf; //Contains the keywords and the color of the circles. This is a combined (produced) DNF


};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DNFNode>());

  rclcpp::shutdown();
  return 0;
}