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
#include "../src/hsvrgb.cpp"
#include "../src/learning_trial.cpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define VOCAB_SIZE 25 //The size of the vocabulary

class DNFNode : public rclcpp::Node
{
public:
  DNFNode() : Node("dnf_pubsub"),
    // Member initialization list
    keywords_dnf(VOCAB_SIZE,true),    //Contains the input keywords in the command. This is the (true) input of the network
    color_circles_dnf(3,true),        //Contains the color of the circles. This is an input
    pos_x_circle_dnf(3,true),         //Contains the x position of the circles. This is an input
    pos_y_circle_dnf(3,true),         //Contains the y position of the circles. This is an input
    action_dnf(5,true),               //Contains the actions, this is the output of the network
    keywords_color_dnf(VOCAB_SIZE,3,true),     //Combined DNF Keywords+Color½int index_input, int index_element,
    keywords_action_dnf(VOCAB_SIZE, 5, true),   //Combined DNF Keywords+Action
    color_expanded_dnf(256, true) //Test of "complex" dnf, color with 256 neurons
  {
    // Create a subscriber to the detected circles
    circle_subscription_ = this->create_subscription<custom_msgs::msg::TriangulatedCircleInfoArr>(
      "triangulated_circle_topic", 1, std::bind(&DNFNode::circle_callback, this, _1));

    keyword_subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
      "keywords", 1, std::bind(&DNFNode::keyword_callback, this, _1));

    // Create service client for the movement command center to recieve decisions on actions and targets
    // decision_service_ = this->create_service<custom_msgs::srv::Decision>("decision_service", &DNFNode::decision_service_callback);
    decision_service_ = this->create_service<custom_msgs::srv::Decision>("/DNF/decision", std::bind(&DNFNode::decision_service_callback, this, _1, _2));

    // Get the path to the dnf logs folder
    get_log_path();

    // Test the HSV-RGB conversion
    // Example of circle BGR values 0.000000 210.000000 0.000000
    // Should HSV: 120, 100, 82
    // float b = 0.0f, g = 210.0f, r = 0.0f;
    // RCLCPP_INFO(this->get_logger(), "Color(BGR) of test circle %f %f %f", b, g, r);
    // float h, s, v;
    // RGBtoHSV(b, g, r, h, s, v);
    // RCLCPP_INFO(this->get_logger(), "HSV of test circle %f %f %f", h, s, v);

    // - - - - - - - - - - - - - - - - - - - Large (256 neurons) DNF test- - - - - - - - - - - - - - - - - - - 
    // color_expanded_dnf.set_input_element(0, 6.9f); // Needs an alternative, "Stimulate input neurons at ()" function

    // Perform the (test) learning
    test_dnf_setup();
    dnf_learning();
  }

private:

  void decision_service_callback(const std::shared_ptr<custom_msgs::srv::Decision::Request> request,
                                 std::shared_ptr<custom_msgs::srv::Decision::Response> response ){
    if (keywords_recieved == false)
    {
      RCLCPP_INFO(this->get_logger(), "Recieved request, but Keywords not recieved yet. Returning...");
      response->actions = 0;
      response->targets = 0;
      response->success = false;
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received request with requestid: [%d]", request->requestid);

    // Extract targets from the KW/Target(color) DNF
    torch::Tensor targets = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    RCLCPP_INFO(this->get_logger(), "Targets extracted: ");
    printTensor(targets);
    int best_target = targets.argmax().item().toInt();
    RCLCPP_INFO(this->get_logger(), "Best target: %d", best_target);

    // Extract actions from the KW/Action DNF
    torch::Tensor actions = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    RCLCPP_INFO(this->get_logger(), "Actions extracted: ");
    printTensor(actions);
    int best_action = actions.argmax().item().toInt();
    RCLCPP_INFO(this->get_logger(), "Best action: %d", best_action);

    // COnvert at::Tensor to int for  response
    response->actions = (int) best_action;
    response->targets = (int) best_target;
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
      //RCLCPP_INFO_STREAM(this->get_logger(), "Keywords: " << '\n' << keywords_dnf.get_input());

      // Set keywords_recieved flag
      keywords_recieved = true;
    }
    return;
  }


  void circle_callback(const custom_msgs::msg::TriangulatedCircleInfoArr::SharedPtr msg){
    if (msg->circles.size() > 0){

        for(size_t i = 0; i<msg->circles.size(); i++){
          detected_circles.push_back(msg->circles[i]);

        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles!");
        RCLCPP_INFO(this->get_logger(), "Color(BGR) of circle 0: %f %f %f", msg->circles[0].bgr_mean[0], msg->circles[0].bgr_mean[1], msg->circles[0].bgr_mean[2]);

        // //Save msg->circles[0].bgr_mean to a vector<float> with RGB values
        // std::vector<float> hsv_local = HSVFromRGB(msg->circles[0].bgr_mean[2], msg->circles[0].bgr_mean[1], msg->circles[0].bgr_mean[0]);
        // RCLCPP_INFO(this->get_logger(), "(Local) HSV of circle 0: %f %f %f", hsv_local[0], hsv_local[1], hsv_local[2]);

        //Convert to HSV
        float h = 0, s = 0, v = 0;
        HSVtoRGB(msg->circles[0].bgr_mean[2], msg->circles[0].bgr_mean[1], msg->circles[0].bgr_mean[0], h, s, v);
        
        //Print new values
        RCLCPP_INFO(this->get_logger(), "(hsvrgb.cpp) HSV of circle 0: %f %f %f", h, s, v);
    } else {
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


  void test_dnf_setup(){
    // Test: Setup the DNFs with predefined values
    
    // - - - - - - - - - - - - - - - - - - - Keyword x Action test- - - - - - - - - - - - - - - - - - - 
    // Learning / Memorization phase - - - - - - - - 
    // Learning "Move"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(0, 6.9f); // The keyword "Move"
    action_dnf.set_input_element(MOVE, 6.9f); // The action "Move"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory

    // Learning "Release"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(21, 6.9f); // The keyword "Release"
    action_dnf.set_input_element(RELEASE, 6.9f); // The action "Release"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory

    // Learning "Grasp"
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(22, 6.9f); // The keyword "Grasp"
    action_dnf.set_input_element(GRASP, 6.9f); // The action "Grasp"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory

    // Learning "Pick" (Up)
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(23, 6.9f); // The keyword "Pick"
    action_dnf.set_input_element(PICK_UP, 6.9f); // The action "Pick Up"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF
    keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input(), 0.5f); //Get it into memory
    
    // Learning "Place" (Down)
    keywords_dnf.reset_input();
    action_dnf.reset_input();
    keywords_dnf.set_input_element(24, 6.9f); // The keyword "Place"
    action_dnf.set_input_element(PLACE_DOWN, 6.9f); // The action "Place Down"
    keywords_action_dnf.set_input(keywords_dnf.get_input(), action_dnf.get_input()); // Set the input of the combined DNF

    // Save KWxA DNF after learning
    write2DTensorCSV(keywords_action_dnf.get_activation(), log_path, "keywords_action_dnf_activation_after_learning.csv");

    // // Memory extraction / Remembering phase - - - - - - - -
    // // Remembering move
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // torch::Tensor move_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(move_response, log_path, "move_response.csv");

    // // Remembering release
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(21, 6.9f); //"Release"
    // torch::Tensor release_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(release_response, log_path, "release_response.csv");

    // // Remembering GRASP
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(22, 6.9f); //"grasp"
    // torch::Tensor grasp_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(grasp_response, log_path, "grasp_response.csv");

    // // Remembering "Pick"
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(23, 6.9f); //"Pick"
    // torch::Tensor pick_reponse = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(pick_reponse, log_path, "pick_response.csv");

    // // Remembering "Place"
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(24, 6.9f); //"Place"
    // torch::Tensor place_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(place_response, log_path, "place_response.csv");

    // // Remember / Extract for a word that has not been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(17, 6.9f); //"Away"
    // torch::Tensor dummy_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(dummy_response, log_path, "dummy_response.csv");

    // // Remember / Extract for several words that have been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // keywords_dnf.set_input_element(21, 6.9f); //"Release"
    // torch::Tensor red_green_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_green_response, log_path, "move_release_response.csv");

    // // Remember / Extract for a phrase containing one learned word
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // keywords_dnf.set_input_element(1, 6.9f); //"The"
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // keywords_dnf.set_input_element(2, 6.9f); //"Ball"
    // keywords_dnf.set_input_element(8, 6.9f); //"Right"
    // torch::Tensor red_phrase_response = keywords_action_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_phrase_response, log_path, "move_phrase_response.csv");

    // - - - - - - - - - - - - - - - - - - - Keyword x Color test 2 - - - - - - - - - - - - - - - - - - - 
    // // Learning / Memorization phase - - - - - - - - 
    // // Learning red
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

    // // Memory extraction / Remembering phase - - - - - - - -
    // // Remembering red
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // torch::Tensor red_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_response, log_path, "red_response.csv");

    // // Remembering green
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(5, 6.9f); //"Green"
    // torch::Tensor green_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(green_response, log_path, "green_response.csv");

    // // Remembering blue
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(6, 6.9f); //"Blue"
    // torch::Tensor blue_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(blue_response, log_path, "blue_response.csv");

    // // Remember / Extract for a word that has not been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(17, 6.9f); //"Away"
    // torch::Tensor dummy_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(dummy_response, log_path, "dummy_response.csv");

    // // Remember / Extract for several words that have been learned
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // keywords_dnf.set_input_element(5, 6.9f); //"Green"
    // torch::Tensor red_green_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_green_response, log_path, "red_green_response.csv");

    // // Remember / Extract for a phrase containing one learned word
    // keywords_dnf.reset_input();
    // keywords_dnf.set_input_element(0, 6.9f); //"Move"
    // keywords_dnf.set_input_element(1, 6.9f); //"The"
    // keywords_dnf.set_input_element(4, 6.9f); //"Red"
    // keywords_dnf.set_input_element(2, 6.9f); //"Ball"
    // keywords_dnf.set_input_element(8, 6.9f); //"Right"
    // torch::Tensor red_phrase_response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
    // write2DTensorCSV(red_phrase_response, log_path, "red_phrase_response.csv");
  }

  void dnf_learning(int learning_iterations = 1){
    // Create learning trials list
    std::vector<learning_trial> learning_trials;

    // Trial: Move the red ball left
    std::vector<int> trial_keywords = {0, 1, 4, 2, 7}; // "Move", "The", "Red", "Ball", "Left"
    learning_trial trial_mrbl(trial_keywords, /*_target_color =*/ 0, /*_correct_action =*/ 2);
    learning_trials.push_back(trial_mrbl);
    
    // Trial: Move the red ball right
    trial_keywords = {0, 1, 4, 2, 8}; // "Move", "The", "Red", "Ball", "Right"
    learning_trial trial_mrbr(trial_keywords, /*_target_color =*/ 0, /*_correct_action =*/ 2);
    learning_trials.push_back(trial_mrbr);

    // Trial: Move the green ball left
    trial_keywords = {0, 1, 5, 2, 7}; // "Move", "The", "Green", "Ball", "Left"
    learning_trial trial_mgbl(trial_keywords, /*_target_color =*/ 1, /*_correct_action =*/ 2);
    learning_trials.push_back(trial_mgbl);

    // Trial: Move the green ball right
    trial_keywords = {0, 1, 5, 2, 8}; // "Move", "The", "Green", "Ball", "Right"
    learning_trial trial_mgbr(trial_keywords, /*_target_color =*/ 1, /*_correct_action =*/ 2);
    learning_trials.push_back(trial_mgbr);

    // Trial: Move the blue ball left
    trial_keywords = {0, 1, 6, 2, 7}; // "Move", "The", "Blue", "Ball", "Left"
    learning_trial trial_mlbl(trial_keywords, /*_target_color =*/ 2, /*_correct_action =*/ 2);
    learning_trials.push_back(trial_mlbl);

    // Trial: Move the blue ball right
    trial_keywords = {0, 1, 6, 2, 8}; // "Move", "The", "Blue", "Ball", "Right"
    learning_trial trial_mlbr(trial_keywords, /*_target_color =*/ 2, /*_correct_action =*/ 2);
    learning_trials.push_back(trial_mlbr);

    // Perform the learning trials
    for (int i = 0; i < learning_iterations; i++){
      // Shuffle the learning trials in each epoch
      std::random_shuffle(learning_trials.begin(), learning_trials.end());

      for (size_t j = 0; j < learning_trials.size(); j++){
        // Set the input keywords
        keywords_dnf.reset_input();
        std::vector<int> keywords = learning_trials[j].get_keywords();
        for (size_t k = 0; k < keywords.size(); k++){
          keywords_dnf.set_input_element(keywords[k], 6.9f);
        }

        // Set the color
        color_circles_dnf.reset_input();
        color_circles_dnf.set_input_element(learning_trials[j].get_target(), 6.9f);

        // Step the DNF
        keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input(), 0.5f);
        // Get the response
        //torch::Tensor response = keywords_color_dnf.extract_response_DNF(keywords_dnf.get_input());
        // Get the error
        //torch::Tensor error = response - learning_trials[j].correct_action;
        // Backpropagate the error
        //keywords_color_dnf.backpropagate_DNF(error);
      }
    }
  }

  

  // Member variables -------------------
  std::string log_path;
  // Subscriptions
  rclcpp::Subscription<custom_msgs::msg::TriangulatedCircleInfoArr>::SharedPtr circle_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr keyword_subscription_;
  bool keywords_recieved = false;
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
  DNF_2D keywords_action_dnf; //Contains the keywords and the actions. This is a combined (produced) DNF;

  DNF_1D color_expanded_dnf; //Contains the color of the circles as a scalar. This is an input


};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DNFNode>());

  rclcpp::shutdown();
  return 0;
}