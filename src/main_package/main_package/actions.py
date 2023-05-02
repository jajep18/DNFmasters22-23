'''
This file contains the set of actions that the robot can perform.
 * These are the possible outcomes of the decision making process.
 * This file is defined twice. In  the dnf package as cpp for the decision making process, and in here in the main package for the python movement_command_center
'''
from enum import Enum
import numpy as np

class Action(np.int8, Enum):
    PICK_UP      = 0,
    PLACE_DOWN   = 1,
    MOVE_LEFT    = 2,
    MOVE_RIGHT   = 3,
    MOVE_BACK    = 4,
    MOVE_FORWARD = 5,
    GRASP        = 6,
    RELEASE      = 7

    def __int__(self):
        return self.value
