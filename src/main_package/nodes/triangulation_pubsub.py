#!/usr/bin/env python3
import math

from geometry_msgs.msg import Twist, TransformStamped, Transform

## Ros includes
import rclpy
from rclpy.node import Node

## Library includes
import cv2
import numpy as np
#import opencv2 as cv2

## Interface (msgs/srvs) includes
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from custom_msgs.msg import CircleInfo                  # Custom message type for circles
from custom_msgs.msg import CircleInfoArr               # Custom message type for array of circles
from custom_msgs.msg import CircleInfoArrStereo         # Custom message type for array of circles for stereo sets
from custom_msgs.msg import TriangulatedCircleInfo      # Custom message type for triangulated of circles for stereo sets
from custom_msgs.msg import TriangulatedCircleInfoArr   # Custom message type for array of triangulated circles for stereo sets


# This node is responsible for triangulation of circles (balls) from the stereo cameras
# This node is rewritten from a CPP equivalent, as openCV had some difficulties we were unable to solve
class TriangulationNode(Node):
    def __init__(self):
        super().__init__('triangulation_node')

        ## Create subscriptions
        self._subscriber_cam_info_left  = self.create_subscription(CameraInfo, '/custom_ns/custom_camera/left/custom_camera_info', self.cam_info_left_callback, 1)
        self._subscriber_cam_info_right = self.create_subscription(CameraInfo, '/custom_ns/custom_camera/right/custom_camera_info', self.cam_info_right_callback, 1)
        self._subscriber_circle_stereo  = self.create_subscription(CircleInfoArrStereo, 'cam_circle_topic', self.triangulation_callback, 1)
        
        ## Create publishers
        self._publisher_triangulated_circles = self.create_publisher(TriangulatedCircleInfoArr, 'triangulated_circles', 1)
        timer_period = 1 # Seconds
        self._timer_publisher = self.create_timer(timer_period, self.publisher_callback)
        self._triangulated_circles = TriangulatedCircleInfoArr() # The variable that will be published
        
        # Define camera matrices (3x3) - These will be set by the cam info callbacks
        self.camera_matrix_left = np.array([])
        self.camera_matrix_right = np.array([])
        self.distortion_coeff_left = np.array([])
        self.distortion_coeff_right = np.array([])

        # Define camera transformation matrices (Translation & Orientation)
        self.transformation_matrix_left = np.array([[1, 0, 0, 0.1], 
                                                    [0, 75.09758, 0, -0.18], 
                                                    [0.0, 0.0, -90.0, 0.7]])
        self.transformation_matrix_right = np.array([[1, 0, 0, 0],
                                                    [0, 75.09758, 0, -0.22],
                                                    [0, 0, -90, 0.7]])

    def cam_info_left_callback(self, msg):
        """
        Topic Callback for Camera Info (Left)
        """
        # If the camera matrix is already set, return
        if self.camera_matrix_left.size == 0:
        # if self.camera_matrix_left == []:
            # Set the camera matrix
            self.camera_matrix_left = np.array(msg.k).reshape(3,3)
            # # Pad the matrix with 0s to be 3x4
            # self.camera_matrix_left = np.pad(self.camera_matrix_left, ((0,0),(0,1)), 'constant', constant_values=(0))

            # Set the distortion coefficients
            self.distortion_coeff_left = np.array(msg.d).reshape(1,5)


    def cam_info_right_callback(self, msg):
        """
        Topic Callback for Camera Info (Right)
        """
        # If the camera matrix is already set, return
        if self.camera_matrix_right.size == 0:
        # if self.camera_matrix_right == []:
            # Set the camera matrix
            self.camera_matrix_right = np.array(msg.k).reshape(3,3)
            # # Pad the matrix with 0s to be 3x4
            # self.camera_matrix_right = np.pad(self.camera_matrix_right, ((0,0),(0,1)), 'constant', constant_values=(0))

            # Set the distortion coefficients
            self.distortion_coeff_right = np.array(msg.d).reshape(1,5)
        

    def triangulation_callback(self, msg):
        """
        Topic Callback for 'cam_circle_topic'. 
        Triangulates the circles detected in the Stereo cameras
        """

        if self.camera_matrix_left.size == 0 or self.camera_matrix_right.size == 0:
            self.get_logger().info('Cannot start triangulation: Camera matrices not set')
            return
        if not msg.left.circles or not msg.right.circles:
            self.get_logger().info('No circles detected')
            return
        self.get_logger().info('Triangulating circles')
        left_circles = msg.left.circles
        right_circles = msg.right.circles

        min_num_circles = min(len(left_circles), len(right_circles))

        # Define projection matrices ()
        # TODO: camMatrix * R * T
        projection_matrix_left = self.camera_matrix_left.dot( self.transformation_matrix_left )
        projection_matrix_right = self.camera_matrix_right.dot( self.transformation_matrix_right )

            
        # Define vectors for storing the triangulated circles
        mean_color_vec = []
        var_color_vec = []
        color_idx = -1

        left_points = []
        right_points = []
        color_idx_vec = []
        
        for i in range(min_num_circles):
            # Use left circle as reference
            left_circle = left_circles[i]
            right_circle = right_circles[i]

            # Find matching circle in right image
            matching_idx = -1
            matching_circle = right_circle
            min_dist = 1000000
            # FInd distance in color space
            for j in range(min_num_circles):
                right_circle = right_circles[j]
                dist = np.linalg.norm(left_circle.bgr_mean - right_circle.bgr_mean)
                if dist < min_dist:
                    min_dist = dist
                    matching_idx = j
                    matching_circle = right_circle
            
            # If no matching circle found, skip
            if matching_idx == -1:
                return
            
            # If matching circle is too far away, skip
            if min_dist > 100:
                return
            
            # Find average color of the two circles
            mean_color = []
            mean_color.append(round( (left_circle.bgr_mean[0] + matching_circle.bgr_mean[0]) / 2 ))
            mean_color.append(round( (left_circle.bgr_mean[1] + matching_circle.bgr_mean[1]) / 2 ))
            mean_color.append(round( (left_circle.bgr_mean[2] + matching_circle.bgr_mean[2]) / 2 ))

            # Find average variance of the two circles
            var_color = []
            var_color.append(round( (left_circle.bgr_var[0] + matching_circle.bgr_var[0]) / 2 ))
            var_color.append(round( (left_circle.bgr_var[1] + matching_circle.bgr_var[1]) / 2 ))
            var_color.append(round( (left_circle.bgr_var[2] + matching_circle.bgr_var[2]) / 2 ))
            
            # Find color channel with max value of circle in mean_color
            max_value = max(mean_color)
            color_idx = mean_color.index(max_value)

            # Create points for triangulation
            left_point = np.array([left_circle.x, left_circle.y])
            right_point = np.array([matching_circle.x, matching_circle.y]) 
            left_points.append(left_point)
            right_points.append(right_point)
            color_idx_vec.append(color_idx)
            
        # Undistort points
        for i in range(len(left_points)):
            left_points[i] = cv2.undistortPoints(left_points[i], self.camera_matrix_left, self.distortion_coeff_left)
            right_points[i] = cv2.undistortPoints(right_points[i], self.camera_matrix_right, self.distortion_coeff_right)
        # CorrectPoints
        [left_points, right_points] = cv2.correctMatches(projection_matrix_left, left_points, right_points)
        
        # Triangulate points
        triangulated_points = cv2.triangulatePoints(projection_matrix_left, projection_matrix_right, left_points, right_points)

        # Convert to homogenous coordinates
        triangulated_points = triangulated_points / triangulated_points[3]
        if triangulated_points[2] < 0:
            self.get_logger().info('Triangulated point is behind camera')
            return
        
        self.get_logger().info('Triangulation has reached the end. There are %d circles', triangulated_points.size)

        # Create TriangulatedCircleInfoArr message
        self._triangulated_circles = TriangulatedCircleInfoArr() # "Clear" the message
        for i in range(triangulated_points.size()):
            triangulated_circle = TriangulatedCircleInfo()
            triangulated_circle.x = triangulated_points[0][i]
            triangulated_circle.y = triangulated_points[1][i]
            triangulated_circle.color_idx = color_idx_vec[i]
            triangulated_circle.mean_color = mean_color_vec[i]
            triangulated_circle.var_color = var_color_vec[i]
            self._triangulated_circles.circles.append(triangulated_circle)
        
        
            

    def publisher_callback(self):
        """
        Publisher callback: 
        Publishes the circles triangulated by triangulation_callback
        """
        # Check if there are any circles to publish
        if not self._triangulated_circles.circles:
            return
        
        # Publish the triangulated circles
        self._triangulated_circles_pub.publish(self._triangulated_circles)



def main(args = None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the node
    node = TriangulationNode()
    
    # # Spin the node
    rclpy.spin(node)
    
    # Destroy the node explicitly (optional)
    #node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()