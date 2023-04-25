#!/usr/bin/env python3
import numpy as np
import math
import csv


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
        
    

def parabolic_blend(start_pos, end_pos, max_vel, sample_rate):
        """
        This function returns the parabolic blend of the start and end positions
        @param: start_pos - The starting position of the blend (x, y, z)
        @param: end_pos - The ending position of the blend (x, y, z)
        @param: max_vel - The maximum velocity of the blend (m/s)
        @param: sample_rate - The sample rate of the blend (Hz) 
        @return: positions - The list of positions in the blend
        """
        # Calculate the distance between the start and end points
        distance = math.sqrt(sum([(end_pos[i]-start_pos[i])**2 for i in range(len(start_pos))]))
        
        # Calculate the duration of the blend
        blend_duration = distance / max_vel
        
        # Calculate the acceleration and deceleration time
        accel_time = blend_duration / 2.0
        decel_time = blend_duration / 2.0
        
        # Calculate the distance covered during acceleration and deceleration
        accel_distance = 0.5 * max_vel * accel_time
        decel_distance = 0.5 * max_vel * decel_time
        
        # Calculate the constant velocity distance
        constant_distance = distance - accel_distance - decel_distance
        
        # Initialize the output list of positions
        positions = []
        
        # Generate the acceleration trajectory
        for t in range(int(accel_time * sample_rate)):
            time = t / float(sample_rate)
            pos = []
            for i in range(len(start_pos)):
                pos.append(start_pos[i] + 0.5 * max_vel * time**2 * (end_pos[i]-start_pos[i]) / distance)
            positions.append(pos)
        
        # Generate the constant velocity trajectory
        for t in range(int(constant_distance / max_vel * sample_rate)):
            time = t / float(sample_rate)
            pos = []
            for i in range(len(start_pos)):
                pos.append(start_pos[i] + accel_distance * (end_pos[i]-start_pos[i]) / distance + max_vel * time * (end_pos[i]-start_pos[i]) / distance)
            positions.append(pos)
        
        # Generate the deceleration trajectory
        for t in range(int(decel_time * sample_rate)):
            time = t / float(sample_rate)
            pos = []
            for i in range(len(start_pos)):
                pos.append(end_pos[i] - 0.5 * max_vel * (decel_time-time)**2 * (end_pos[i]-start_pos[i]) / distance)
            positions.append(pos)
        
        # Add the end point to the trajectory
        positions.append(end_pos)
        
        return positions

def create_path(current_pos, pickup_target, place_target, height_over_target):
    '''
    @param: current_pos - The current position of the robot
    @param: pickup_target - The target position of the pickup
    @param: place_target - The target position of the place
    @param: height_over_target - The height above the target to move to
    @return: paths - The list of paths to follow to perform pick and place
    '''
    # Define the points
    # Starting position
    start_pos = current_pos
    # Above pickup position
    pickup = np.copy(pickup_target)
    above_pickup = pickup_target
    above_pickup[2] = above_pickup[2] + height_over_target
    # Above place position
    place = np.copy(place_target)
    above_place = place_target
    above_place[2] = above_place[2] + height_over_target
    end_pos = current_pos
    
    # Initialize the list of positions for parabolic blend
    positions = []
    positions.append(start_pos)
    positions.append(above_pickup)
    positions.append(pickup)
    positions.append(above_pickup)
    positions.append(above_place)
    positions.append(place)
    positions.append(above_place)
    positions.append(end_pos)
    
    print("Starting position: ", start_pos)
    print("Above pickup position: ", above_pickup)
    print("Pickup position: ", pickup)
    print("Above place position: ", above_place)
    print("Place position: ", place)
    print("Ending position: ", end_pos)

    
    # Create paths with parabolic blend
    paths = []
    for i in range(len(positions)-1):
        path = parabolic_blend(positions[i], positions[i+1], 0.5, 10 )
        paths.append(path)
    
    return paths

def write_path_to_csv(tcp_path, filename):
    '''
    This function writes a path to a csv file
    @param: tcp_path - The (tcp) path to write to the csv file
    @param: filename - The name of the csv file, should end in .csv
    '''

    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for row in tcp_path:
            for i in range(len(row)):
                writer.writerow(row[i])
        writer.writerow("\n")
    


def main():
    print("Testing movement.py")

    start_pos = np.array([0.0, 0.0, 1.0])
    pickup_target = np.array([1.0, 1.0, 1.0])
    place_target = np.array([2.0, 2.0, 1.0])
    height_offset = 0.3

    test_path = create_path(start_pos, pickup_target, place_target, height_offset)
    
    write_path_to_csv(test_path, "test_path.csv")

    #print(test_path)
        
if __name__ == "__main__":
    main()
