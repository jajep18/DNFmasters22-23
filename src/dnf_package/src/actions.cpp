#include <vector>

/* This file contains the set of actions that the robot can perform.
 * These are the possible outcomes of the decision making process.
 * This file is defined twice, here in cpp for the decision making process, and in the main package for the pythonic movement_command_center
 */

enum action{
    PICK_UP      = 0,
    PLACE_DOWN   = 1,
    MOVE_LEFT    = 2,
    MOVE_RIGHT   = 3,
    MOVE_BACK    = 4,
    MOVE_FORWARD = 5,
    GRASP        = 6,
    RELEASE      = 7
};


// Legacy code: The below were the set of actions used during preliminary tests of the decision making process

// enum action{
//     PICK_UP     = 0,
//     PLACE_DOWN  = 1,
//     MOVE        = 2,
//     GRASP       = 3,
//     RELEASE     = 4
// };
