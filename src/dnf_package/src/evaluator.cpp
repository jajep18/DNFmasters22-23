// Include header (Read description in header)
#include "../include/evaluator.hpp"
#include "dnf_lib.cpp"

// Include ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/logging.hpp> //For "get_logger"
#include <iostream>


// Constructor
Evaluator::Evaluator(bool debug) { //Debug defaults to true
    if (debug) {
        RCLCPP_INFO(rclcpp::get_logger("dnf_evaluator"), "Evaluator constructor called");
    }

    // Initialize evaluation trials
    float data_trial1[17]  = {1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Red, Left
    float data_trial2[17]  = {1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Red, Right
    float data_trial3[17]  = {1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Red, Closer
    float data_trial4[17]  = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}; // Move, Ball, Red, Away
    float data_trial5[17]  = {0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0}; // Pick, Up, Red, Ball
    float data_trial6[17]  = {0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0}; // Place, Down, Red, Ball
    float data_trial7[17]  = {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}; // Grasp, Red, Ball
    float data_trial8[17]  = {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}; // Release, Red, Ball

    float data_trial9[17]  = {1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Left, Green
    float data_trial10[17] = {1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Right, Green
    float data_trial11[17] = {1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Closer, Green
    float data_trial12[17] = {1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}; // Move, Ball, Away, Green
    float data_trial13[17] = {0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0}; // Pick, Up, Ball, Green
    float data_trial14[17] = {0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0}; // Place, Down, Ball, Green
    float data_trial15[17] = {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}; // Grasp, Ball, Green
    float data_trial16[17] = {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}; // Release, Ball, Green

    float data_trial17[17] = {1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Left, Blue
    float data_trial18[17] = {1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Right, Blue
    float data_trial19[17] = {1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}; // Move, Ball, Closer, Blue
    float data_trial20[17] = {1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0}; // Move, Ball, Away, Blue
    float data_trial21[17] = {0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0}; // Pick, Up, Ball, Blue
    float data_trial22[17] = {0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0}; // Place, Down, Ball, Blue
    float data_trial23[17] = {0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}; // Grasp, Ball, Blue
    float data_trial24[17] = {0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}; // Release, Ball, Blue

    torch::Tensor trial1    = torch::from_blob(data_trial1, {17}, torch::kFloat32).clone();
    torch::Tensor trial2    = torch::from_blob(data_trial2, {17}, torch::kFloat32).clone();
    torch::Tensor trial3    = torch::from_blob(data_trial3, {17}, torch::kFloat32).clone();
    torch::Tensor trial4    = torch::from_blob(data_trial4, {17}, torch::kFloat32).clone();
    torch::Tensor trial5    = torch::from_blob(data_trial5, {17}, torch::kFloat32).clone();
    torch::Tensor trial6    = torch::from_blob(data_trial6, {17}, torch::kFloat32).clone();
    torch::Tensor trial7    = torch::from_blob(data_trial7, {17}, torch::kFloat32).clone();
    torch::Tensor trial8    = torch::from_blob(data_trial8, {17}, torch::kFloat32).clone();
    torch::Tensor trial9    = torch::from_blob(data_trial9, {17}, torch::kFloat32).clone();
    torch::Tensor trial10   = torch::from_blob(data_trial10, {17}, torch::kFloat32).clone();
    torch::Tensor trial11   = torch::from_blob(data_trial11, {17}, torch::kFloat32).clone();
    torch::Tensor trial12   = torch::from_blob(data_trial12, {17}, torch::kFloat32).clone();
    torch::Tensor trial13   = torch::from_blob(data_trial13, {17}, torch::kFloat32).clone();
    torch::Tensor trial14   = torch::from_blob(data_trial14, {17}, torch::kFloat32).clone();
    torch::Tensor trial15   = torch::from_blob(data_trial15, {17}, torch::kFloat32).clone();
    torch::Tensor trial16   = torch::from_blob(data_trial16, {17}, torch::kFloat32).clone();
    torch::Tensor trial17   = torch::from_blob(data_trial17, {17}, torch::kFloat32).clone();
    torch::Tensor trial18   = torch::from_blob(data_trial18, {17}, torch::kFloat32).clone();
    torch::Tensor trial19   = torch::from_blob(data_trial19, {17}, torch::kFloat32).clone();
    torch::Tensor trial20   = torch::from_blob(data_trial20, {17}, torch::kFloat32).clone();
    torch::Tensor trial21   = torch::from_blob(data_trial21, {17}, torch::kFloat32).clone();
    torch::Tensor trial22   = torch::from_blob(data_trial22, {17}, torch::kFloat32).clone();
    torch::Tensor trial23   = torch::from_blob(data_trial23, {17}, torch::kFloat32).clone();
    torch::Tensor trial24   = torch::from_blob(data_trial24, {17}, torch::kFloat32).clone();
    
    // Push evaluation trials to vector
    evaluation_trials.push_back(trial1);
    evaluation_trials.push_back(trial2);
    evaluation_trials.push_back(trial3);
    evaluation_trials.push_back(trial4);
    evaluation_trials.push_back(trial5);
    evaluation_trials.push_back(trial6);
    evaluation_trials.push_back(trial7);
    evaluation_trials.push_back(trial8);
    evaluation_trials.push_back(trial9);
    evaluation_trials.push_back(trial10);
    evaluation_trials.push_back(trial11);
    evaluation_trials.push_back(trial12);
    evaluation_trials.push_back(trial13);
    evaluation_trials.push_back(trial14);
    evaluation_trials.push_back(trial15);
    evaluation_trials.push_back(trial16);
    evaluation_trials.push_back(trial17);
    evaluation_trials.push_back(trial18);
    evaluation_trials.push_back(trial19);
    evaluation_trials.push_back(trial20);
    evaluation_trials.push_back(trial21);
    evaluation_trials.push_back(trial22);
    evaluation_trials.push_back(trial23);
    evaluation_trials.push_back(trial24);

    // Initialize evaluation trial targets (colors)
    evaluation_targets = {  0, 0, 0, 0, 0, 0, 0, 0, 
                            1, 1, 1, 1, 1, 1, 1, 1,
                            2, 2, 2, 2, 2, 2, 2, 2 };

    // Initialize evaluation trial actions (actions)
    evaluation_actions = {  2, 3, 5, 4, 0, 1, 6, 7, 
                            2, 3, 5, 4, 0, 1, 6, 7, 
                            2, 3, 5, 4, 0, 1, 6, 7};

    // Create trial 1 from a known list of numbers [1 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0]
    //torch::Tensor trial1 = torch.tensor({1, 1, 1, 0, 0, 1,0, 0, 0, 0, 0, 0, 0, 0,0, 0}, torch::kFloat32);
    

    if (debug) {
        RCLCPP_INFO(rclcpp::get_logger("dnf_evaluator"), "Initialized evaluator with %d evaluation trials, %d evaluation targets, and %d evaluation actions", 
                                                            evaluation_trials.size(), evaluation_targets.size(), evaluation_actions.size());
        // print the first trial
        RCLCPP_INFO(rclcpp::get_logger("dnf_evaluator"), "First trial: %s", trial1.toString());
        std::cout << trial1 << std::endl;
    }
}

// Destructor
Evaluator::~Evaluator() {
    RCLCPP_INFO(rclcpp::get_logger("dnf_evaluator"), "Evaluator destructor called");
}

// Evaluate the given DNF
double Evaluator::evaluate_KWxA(torch::Tensor correlation_matrix, int & _failed_trials){
    // Sanity check that evaluation_trials is of size 24
    if (evaluation_trials.size() != 24) {
        RCLCPP_ERROR(rclcpp::get_logger("dnf_evaluator"), "Evaluation trials is not of size 24");
        //return -1;
    }

    // Go through all the trials (evaluation_trials)
    int failed_trials = 0;
    std::vector<double> softmax_results;
    
    for (size_t i = 0; i < evaluation_trials.size(); i++)
    {
        // Get the trial
        torch::Tensor eval_trial = evaluation_trials[i];
        int correct_action = evaluation_actions[i];

        // Get trial response
        // torch::Tensor trial_response = eval_trial.matmul(correlation_matrix);
        torch::Tensor trial_response = torch::matmul(eval_trial.unsqueeze(0), correlation_matrix);
        // std::cout << "trial response: " << trial_response << std::endl;
        
        // Run argmax on the evaluation trial
        int argmax_result = torch::argmax(trial_response).item<int>();
        // RCLCPP_INFO(rclcpp::get_logger("dnf_evaluator"), "Argmax result: %d", argmax_result);

        // Check if the argmax result is equal to the correct action index
        if (argmax_result != correct_action) {
            failed_trials++;
        }

        // Get softmax of the evaluation trial
        torch::Tensor softmax_vec = torch::softmax(trial_response, 1);
        // std::cout << "softmax_vec: " << softmax_vec << std::endl;

        // Get softmax value of the correct action:
        softmax_results.push_back(softmax_vec[0][correct_action].item<double>());
        // std::cout << "softmax_result:" << softmax_vec[0][correct_action].item<double>() << std::endl;
    }

    // Get mean of softmax results
    double mean_softmax = std::accumulate(softmax_results.begin(), softmax_results.end(), 0.0) / double(softmax_results.size());
    
    _failed_trials = failed_trials;
    return mean_softmax;    
}

double Evaluator::evaluate_KWxT(torch::Tensor correlation_matrix, int & _failed_trials){
        // Sanity check that evaluation_trials is of size 24
    if (evaluation_trials.size() != 24) {
        RCLCPP_ERROR(rclcpp::get_logger("dnf_evaluator"), "Evaluation trials is not of size 24");
        //return -1;
    }

    // Go through all the trials (evaluation_trials)
    int failed_trials = 0;
    std::vector<double> softmax_results;
    
    for (size_t i = 0; i < evaluation_trials.size(); i++)
    {
        // Get the trial
        torch::Tensor eval_trial = evaluation_trials[i];
        int correct_target = evaluation_targets[i];

        // Get trial response
        torch::Tensor trial_response = torch::matmul(eval_trial.unsqueeze(0), correlation_matrix);

        // Print trial response
        // std::cout << "trial response: " << trial_response << std::endl;
        
        // Run argmax on the evaluation trial
        int argmax_result = torch::argmax(trial_response).item<int>();

        // Check if the argmax result is equal to the correct action index
        if (argmax_result != correct_target) {
            failed_trials++;
        }
        // Get softmax of the evaluation trial
        torch::Tensor softmax_vec = torch::softmax(trial_response, 1); 
        // Get softmax value of the correct action:
        softmax_results.push_back(softmax_vec[0][correct_target].item<double>());
    }
    // Get mean of softmax results
    double mean_softmax = std::accumulate(softmax_results.begin(), softmax_results.end(), 0.0) / double(softmax_results.size());
    _failed_trials = failed_trials;
    return mean_softmax;  
}