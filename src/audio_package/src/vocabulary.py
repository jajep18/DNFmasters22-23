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
    CUP     = 12
    BLOCK   = 13
    BOX     = 14
    CUBE    = 15
    SPHERE  = 16
    AWAY    = 17
    YOUR    = 18
    ERROR   = 19
    NO      = 20
    RELEASE = 21
    GRASP   = 22
    PICK    = 23
    PLACE   = 24

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

   
    #MOM     = 69
    
