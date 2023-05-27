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
#include <boost/shared_ptr.hpp>

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
#include "../src/evaluator.cpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define VOCAB_SIZE 17 //The size of the keyword vocabulary
#define ACTION_AMOUNT 8 //The size of the action vocabulary

class DNFNode : public rclcpp::Node
{
public:
  DNFNode() : Node("dnf_pubsub"),
    // Member initialization list
    keywords_dnf(VOCAB_SIZE,true),    //Contains the input keywords in the command. This is the (true) input of the network
    color_circles_dnf(3,true),        //Contains the color of the circles. This is an input
    pos_x_circle_dnf(3,true),         //Contains the x position of the circles. This is an input
    pos_y_circle_dnf(3,true),         //Contains the y position of the circles. This is an input
    action_dnf(ACTION_AMOUNT, true),                        //Contains the actions, this is the output of the network
    keywords_color_dnf(VOCAB_SIZE, 3, true, HEBBIAN, COLNORM, NO_SUPP, 0.5f),      //Combined DNF Keywords+Color
    keywords_action_dnf(VOCAB_SIZE, ACTION_AMOUNT, true,  HEBBIAN, COLNORM, NO_SUPP, 0.5f),   //Combined DNF Keywords+Action HEBBIAN, NORM, SUPP, 0.5f
    // keywords_color_dnf(VOCAB_SIZE,3,true),                  //Combined DNF Keywords+Color
    // keywords_action_dnf(VOCAB_SIZE, ACTION_AMOUNT, true),   //Combined DNF Keywords+Action
    color_expanded_dnf(256, true), //Test of "complex" dnf, color with 256 neurons
    evaluator(true) //Evaluator for evaluating the performance of the DNF
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

    // Create evaluator - Class that can evaluate the performance of a learned correlation matrix using 24 examples
    //Evaluator evaluator; Done in initialization list instead

    // --------------- Create lists of DNFs with different params for comparison. learning rule, norm, suppr, etc. ---------------
    RCLCPP_INFO(this->get_logger(), "Setting up list of Correlation matrices to be trained in tandem for comparison");
    // // Classical ML learning rule, no normalization, no suppression
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, NO_SUPP, 0.1f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, NO_SUPP, 0.2f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, NO_SUPP, 0.3f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, NO_SUPP, 0.4f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, NO_SUPP, 0.5f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, NO_SUPP, 0.6f));
    // // Classical ML learning rule, reg. normalization, no suppression
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, NO_SUPP, 0.1f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, NO_SUPP, 0.2f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, NO_SUPP, 0.3f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, NO_SUPP, 0.4f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, NO_SUPP, 0.5f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, NO_SUPP, 0.6f));
    // // Classical ML learning rule, no normalization, with suppression
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, SUPP, 0.1f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, SUPP, 0.2f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, SUPP, 0.3f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, SUPP, 0.4f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, SUPP, 0.5f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NONE, SUPP, 0.6f));
    // // Classical ML learning rule, reg. normalization, with suppression
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, SUPP, 0.1f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, SUPP, 0.2f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, SUPP, 0.3f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, SUPP, 0.4f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, SUPP, 0.5f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, ML, NORM, SUPP, 0.6f));
    // // Hebbian learning rule, no normalization, no suppression
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, NO_SUPP, 0.25f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, NO_SUPP, 0.5f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, NO_SUPP, 0.75f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, NO_SUPP, 1.0f));
    // // Hebbian learning rule, reg. normalization, no suppression
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, NO_SUPP, 0.25f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, NO_SUPP, 0.5f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, NO_SUPP, 0.75f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, NO_SUPP, 1.0f));
    // // Hebbian learning rule, column normalization, no suppression
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, NO_SUPP, 0.25f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, NO_SUPP, 0.5f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, NO_SUPP, 0.75f));
    // KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, NO_SUPP, 1.0f));

    // Hebbian learning rule, no normalization, with suppression
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, SUPP, 0.25f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, SUPP, 0.5f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, SUPP, 0.75f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NONE, SUPP, 1.0f));
    // Hebbian learning rule, reg. normalization, with suppression
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, SUPP, 0.25f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, SUPP, 0.5f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, SUPP, 0.75f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, NORM, SUPP, 1.0f));
    // Hebbian learning rule, column normalization, with suppression
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, SUPP, 0.25f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, SUPP, 0.5f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, SUPP, 0.75f));
    KW_ACT_DNF_list.push_back(new DNF_2D(VOCAB_SIZE, ACTION_AMOUNT, true, HEBBIAN, COLNORM, SUPP, 1.0f));

    // Perform the learning set up
    dnf_learning(50);
    RCLCPP_INFO(this->get_logger(), "dnf-learning() setup complete");

    // Print the comparison log to csv
    // Write tensor to file
    std::ofstream comp_file;
    comp_file.open((comp_log_path + "Eval_v1.csv").c_str());
    if(comp_file.fail()){RCLCPP_ERROR(this->get_logger(), "Failed to open comparison log file");}
    for (size_t i = 0; i < comp_log.size(); i++){
      for (size_t j = 0; j < comp_log[i].size(); j++){
        comp_file << comp_log[i][j] << ",";
      }
      comp_file << "\n";
    }
    comp_file.close();
    RCLCPP_INFO(this->get_logger(), "Comp log complete");
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

    // Convert at::Tensor to int for  response
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
        //RCLCPP_INFO_STREAM(this->get_logger(), "Found " << msg->circles.size() << " circles!");
        //RCLCPP_INFO(this->get_logger(), "Color(BGR) of circle 0: %f %f %f", msg->circles[0].bgr_mean[0], msg->circles[0].bgr_mean[1], msg->circles[0].bgr_mean[2]);

        // //Save msg->circles[0].bgr_mean to a vector<float> with RGB values
        // std::vector<float> hsv_local = HSVFromRGB(msg->circles[0].bgr_mean[2], msg->circles[0].bgr_mean[1], msg->circles[0].bgr_mean[0]);
        // RCLCPP_INFO(this->get_logger(), "(Local) HSV of circle 0: %f %f %f", hsv_local[0], hsv_local[1], hsv_local[2]);

        //Convert to HSV
        float h = 0, s = 0, v = 0;
        HSVtoRGB(msg->circles[0].bgr_mean[2], msg->circles[0].bgr_mean[1], msg->circles[0].bgr_mean[0], h, s, v);
        
        //Print new values
        //RCLCPP_INFO(this->get_logger(), "(hsvrgb.cpp) HSV of circle 0: %f %f %f", h, s, v);
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
    comp_log_path = dnf_log_path.append("comparison/");

    //TODO: Combine this path with the appended path to the log directory, to get the full path
    // Cleaner way than what we have now
    //RCLCPP_INFO(this->get_logger(), "Filesystem: Current path: %s", std::filesystem::current_path().c_str());
    //std::filesystem::current_path() 
  }




  void dnf_learning(int learning_iterations = 1){
    // Create learning trials list
    std::vector<learning_trial> learning_trials;

    // Loop through the different target colors, and create every type of learning trial for each color
    // Keywords:    2: red, 3: green, 4: blue
    // Colors:      0: red, 1: green, 2: blue
    for (int i = 2; i <= 4; i++){
      int target_color = i-2; // Set the target color for the learning trials

      // Trial: Move the X ball to the left
      std::vector<int> trial_keywords = {0, i, 1, 5}; // "Move", TARGET COLOR, "Ball", "Left"
      learning_trial trial_mxbl(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ MOVE_LEFT);
      learning_trials.push_back(trial_mxbl);
      
      // Trial: Move the X ball to the right
      trial_keywords = {0, i, 1, 6}; // "Move", TARGET COLOR, "Ball", "Right"
      learning_trial trial_mxbr(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ MOVE_RIGHT);
      learning_trials.push_back(trial_mxbr);

      // Trial: Move the X ball away
      trial_keywords = {0, i, 1, 10}; // "Move", TARGET COLOR, "Ball", "Away"
      learning_trial trial_mxba(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ MOVE_BACK);
      learning_trials.push_back(trial_mxba);

      // Trial: Move the X ball closer / towards the arm
      trial_keywords = {0, i, 1, 9}; // "Move", TARGET COLOR, "Ball", "Closer"
      learning_trial trial_mxbc(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ MOVE_FORWARD);
      learning_trials.push_back(trial_mxbc);

      // Trial: Pick up the X ball
      trial_keywords = {14, 7, i, 1}; // "Pick", "Up", TARGET COLOR, "Ball"
      learning_trial trial_pux(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ PICK_UP);
      learning_trials.push_back(trial_pux);

      // Trial: Place down the X ball
      trial_keywords = {15, 8, i, 1}; // "Place", "Down", TARGET COLOR, "Ball"
      learning_trial trial_plx(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ PLACE_DOWN);
      learning_trials.push_back(trial_plx);

      // Trial: Grasp the X ball
      trial_keywords = {13, i, 1}; // "Grasp", TARGET COLOR, "Ball"
      learning_trial trial_grx(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ GRASP);
      learning_trials.push_back(trial_grx);

      // Trial: Release the X ball
      trial_keywords = {16, i, 1}; // "Release", TARGET COLOR, "Ball"
      learning_trial trial_rlx(trial_keywords, /*_target_color =*/ target_color, /*_correct_action =*/ RELEASE);
      learning_trials.push_back(trial_rlx);
    }
    
    RCLCPP_INFO(this->get_logger(), "DNF learning trials created");

    // Perform the learning trials
    for (int i = 0; i < learning_iterations; i++){
      // // Shuffle the learning trials in each epoch
      // std::random_shuffle(learning_trials.begin(), learning_trials.end());

      for (size_t j = 0; j < learning_trials.size(); j++){
        // Set the input keywords
        keywords_dnf.reset_input();
        std::vector<int> keywords = learning_trials[j].get_keywords();
        for (size_t k = 0; k < keywords.size(); k++){
          keywords_dnf.set_input_element(keywords[k], 1.0f);
        }

        // Set the target color
        color_circles_dnf.reset_input();
        color_circles_dnf.set_input_element(learning_trials[j].get_target(), 1.0f);

        // Set the correct action
        action_dnf.reset_input();
        action_dnf.set_input_element(learning_trials[j].get_action(), 1.0f); // The action in the learning trial

        // Step the Keyword x Color DNF
        keywords_color_dnf.step(keywords_dnf.get_input(), color_circles_dnf.get_input());

        // Step the Keyword x Action DNF
        keywords_action_dnf.step(keywords_dnf.get_input(), action_dnf.get_input());

        // Step all the KW_ACT_DNFs with the given inputs - These are for comparing the different learning parameters
        step_all_test_KW_ACT_dnfs(keywords_dnf.get_input(), action_dnf.get_input());

        // If this is the first learning trial in the first epoch, save the initial activations
        if (i == 0 && j == 0){
          // Save KWxA DNF before learning
          write2DTensorCSV(keywords_action_dnf.get_activation(), log_path, "keywords_action_dnf_1_trial.csv");

          // Save KWxC DNF before learning
          write2DTensorCSV(keywords_color_dnf.get_activation(), log_path, "keywords_color_dnf_1_trial.csv");
        }
      }

      // If this is the first epoch, save the activations after the first epoch
      if (i == 0){
        // Save KWxA DNF after the first epoch
        write2DTensorCSV(keywords_action_dnf.get_activation(), log_path, "keywords_action_dnf_1_epoch.csv");

        // Save KWxC DNF after the first epoch
        write2DTensorCSV(keywords_color_dnf.get_activation(), log_path, "keywords_color_dnf_1_epoch.csv");
      }

      // Shuffle the learning trials in each epoch
      std::random_shuffle(learning_trials.begin(), learning_trials.end());
      
      RCLCPP_INFO(this->get_logger(), "DNF learning epoch {%d}/{%d} complete", i+1, learning_iterations);
    }

    RCLCPP_INFO(this->get_logger(), "DNF learning complete!");

    // Save KWxA DNF after learning
    write2DTensorCSV(keywords_action_dnf.get_activation(), log_path, "keywords_action_dnf_50_epoch.csv");

    // Save KWxC DNF after learning
    write2DTensorCSV(keywords_color_dnf.get_activation(), log_path, "keywords_color_dnf_50_epoch.csv");
    RCLCPP_INFO(this->get_logger(), "DNFs saved to csv");

    // Test evaluate 
    int fails = 0;
    double eval_res = evaluator.evaluate_KWxA(keywords_action_dnf.get_activation(), fails);
    // Print fails
    RCLCPP_INFO(this->get_logger(), "Evaluator fails: %d", fails);
    RCLCPP_INFO(this->get_logger(), "Evaluation softmax mean result: %f", eval_res);
  }



  void step_all_test_KW_ACT_dnfs(torch::Tensor input1, torch::Tensor input2){
    // Step all the KW_ACT_DNFs with the given inputs
    for (size_t i = 0; i < KW_ACT_DNF_list.size(); i++){
      KW_ACT_DNF_list[i]->step(input1, input2);
    }

    // Save the specific activation at (14,0) for each KW_ACT_DNF into comp_log
    std::vector<float> comp_log_row;
    for (size_t i = 0; i < KW_ACT_DNF_list.size(); i++){
      int failed_evals = 0;
      float eval_res = evaluator.evaluate_KWxA(KW_ACT_DNF_list[i]->get_activation(), failed_evals);

      comp_log_row.push_back(float(failed_evals));
      comp_log_row.push_back(eval_res);
    }

    // // Save the specific activation at (14,0) for each KW_ACT_DNF into comp_log
    // std::vector<float> comp_log_row;
    // for (size_t i = 0; i < KW_ACT_DNF_list.size(); i++){
    //   comp_log_row.push_back(KW_ACT_DNF_list[i]->get_activation_at(14,0));
    // }
    comp_log.push_back(comp_log_row);
  }

  

  // Member variables -------------------
  std::string log_path;
  std::string comp_log_path;
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

  // DNFs for learning - List of keyword x action DNFs, multiple so that learning comparisons can be made for the rapport
  std::vector<DNF_2D*> KW_ACT_DNF_list;
  // Create a storage variable for the element in the correlation matrix being compared:
  std::vector<std::vector<float>> comp_log;

  // Evaluator object
  Evaluator evaluator;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DNFNode>());

  rclcpp::shutdown();
  return 0;
}