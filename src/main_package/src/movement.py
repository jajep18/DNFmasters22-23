import numpy as np

def extrapolate_path(joints_current, joints_desired):
    '''
    This function extrapolates a path for the robot based on the current and desired joint positions
    input: Current joint positions, desired joint positions (Only the first 3)
    output: Extrapolated path
    '''
    # Verify that the current joint positions are valid (lol)
    #Verify(joints_current)

    # Verify that the desired joint positions are valid
    #Verify(joints_desired)

    # Calculate the distance between the current and desired positions
    distance = np.linalg.norm(joints_current - joints_desired)

    # Calculate the number of steps needed to reach the desired position
    steps = int(distance / 0.01)

    # Calculate the step size
    step_size = distance / steps

    # Calculate the direction vector
    direction_vector = (joints_desired - joints_current) / distance

    # Calculate the path
    path = np.zeros((steps, 3))
    for i in range(steps):
        path[i] = joints_current + (i * step_size * direction_vector)
    
    return path

