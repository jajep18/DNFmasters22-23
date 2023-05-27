#ifndef EVALUATOR
#define EVALUATOR
/* 
 * Description:     Class that stores information about 24 different learning trials.
                    Used to evaluate the performance of the DNF.
 *
 * Author:			Erik Lindby
 *                  Jacob Floe Jeppesen
 *					University of Southern Denmark
 * Creation date:   01-04-2023
 */

// Includes
#include <torch/torch.h>
#include <vector>

class Evaluator
{
private:
    // Evaluation trials
    std::vector<torch::Tensor> evaluation_trials; // 24 torch tensors containing the input keywords for each trial
    std::vector<int> evaluation_targets; // 24 integers containing the target color for each trial
    std::vector<int> evaluation_actions; // 24 integers containing the correct action for each trial
    
public:
    // Constructors / Destructors
    Evaluator(bool debug = true);
    ~Evaluator();

    // Evaluation function (to be used on keyword-action correlation matrices)
    double evaluate_KWxA(torch::Tensor correlation_matrix, int & _failed_trials);

    // Evaluation function (to be used on keyword-target color correlation matrices)
    double evaluate_KWxT(torch::Tensor correlation_matrix, int & _failed_trials);

};

#endif // EVALUATOR