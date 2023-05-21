#ifndef LEARNING_TRIAL
#define LEARNING_TRIAL
/* 
 * Description:     Class that stores information about a single learning trial
 *                  Keywords, Correct target (color), correct action 
 *
 * Author:			Erik Lindby
 *                  Jacob Floe Jeppesen
 *					University of Southern Denmark
 * Creation date:   01-04-2023
 */

// Includes
#include <torch/torch.h>
#include <vector>

class learning_trial
{
private:
    std::vector<int> keywords; // Keywords in numbers (vocab)
    int target_color;
    int correct_action;
    
public:
    learning_trial(/* args */); // Default constroctur (This will not be used)
    learning_trial(std::vector<int> _keywords, int _target_color, int _correct_action); // Constructor (This will be used
    ~learning_trial(); // Default destructor (Blame ROS2)

    // Getters
    int get_target();
    int get_action();
    std::vector<int> get_keywords();
};

#endif // LEARNING_TRIAL