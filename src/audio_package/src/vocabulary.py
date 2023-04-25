from enum import Enum, EnumMeta

#import int8
import numpy as np

# Get the coordinates of the ball from the color passed
# coordinates = self._ball_coordinates[Color[color.upper()].value]

# class MetaEnum(EnumMeta):
#     def __contains__(cls, item):
#         try:
#             cls(item)
#         except ValueError:
#             return False
#         return True    

# class BaseEnum(Enum, metaclass=MetaEnum):
#     pass

# Keywords - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class Keywords(np.int8, Enum):
    MOVE    = 0
    BALL    = 1
    RED     = 2
    GREEN   = 3
    BLUE    = 4
    LEFT    = 5
    RIGHT   = 6
    UP      = 7
    DOWN    = 8
    CLOSER  = 9
    AWAY    = 10
    BACK    = 11
    SPHERE  = 12
    GRASP   = 13
    PICK    = 14
    PLACE   = 15
    ERROR   = 16 # Should not be used, non-valid keywords for checks return this


    #FORWARD = 11



# All words (Dictionary for pocketsphinx) - - - - - - - - - - - - - -
class Words(np.int8, Enum):
    MOVE    = 0
    THE     = 1
    BALL    = 2
    TO      = 3
    RED     = 4
    GREEN   = 5
    BLUE    = 6
    LEFT    = 7
    RIGHT   = 8
    UP      = 9
    DOWN    = 10
    ARM     = 11
    CLOSER  = 12
    FROM    = 13
    BACK    = 14
    SPHERE  = 15
    AWAY    = 16
    RELEASE = 17
    GRASP   = 18
    PICK    = 19
    PLACE   = 20
    A       = 21
    ERROR   = 22 # Should not be used, non-valid words for checks return this

    # Dont use "forward" - Pocketsphinx detects it instead of ball
    
    
    # YOUR    = 18
    # ERROR   = 19
    # NO      = 20
    # CUP     = 12
    # BLOCK   = 13
    # BOX     = 14
    # CUBE    = 15
    # MOM     = 69
    
