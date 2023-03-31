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
        
        # Define camera matrices (3x3) - K & d - These will be set by the cam info callbacks
        self.camera_matrix_left = np.array([])
        self.camera_matrix_right = np.array([])
        self.distortion_coeff_left = np.array([])
        self.distortion_coeff_right = np.array([])

        # # Define camera transformation matrices (Translation & Orientation)
        # self.transformation_matrix_left = np.array([[1,     0,          0,      0.1], 
        #                                             [0,     75.09758,   0,      -0.18], 
        #                                             [0.0,   0.0,        -90.0,  0.7]])
        
        # self.transformation_matrix_right = np.array([[1,    0,          0,      -0.1],
        #                                             [0,     75.09758,   0,      -0.22],
        #                                             [0,     0,          -90,    0.7]])
        
        # Define camera transformation matrices (Translation & Orientation)
        self.transformation_matrix_left = np.array([[-0.45728419,     0.6828136,    0.56978659,  0.10], 
                                                    [-0.6828136 ,     0.14092347,  -0.71687248, -0.18], 
                                                    [-0.56978659,    -0.71687248,   0.40179234,  0.70]])
        
        self.transformation_matrix_right = np.array([[-0.45728419,     0.6828136,    0.56978659, -0.10], 
                                                     [-0.6828136 ,     0.14092347,  -0.71687248, -0.22], 
                                                     [-0.56978659,    -0.71687248,   0.40179234,  0.70]])

        # Define color dictionary - used for discrete interpretations of colors
        self.color_dict = ["blue", "green", "red", "unknown"]
        

    def cam_info_left_callback(self, msg):
        """
        Topic Callback for Camera Info (Left)
        """
        # If the camera matrix is already set, return
        if self.camera_matrix_left.size == 0:
        # if self.camera_matrix_left == []:
            # Set the camera matrix K
            self.camera_matrix_left = np.array(msg.k).reshape(3,3)

            # Set the distortion coefficients d
            self.distortion_coeff_left = np.array(msg.d).reshape(1,5)


    def cam_info_right_callback(self, msg):
        """
        Topic Callback for Camera Info (Right)
        """
        # If the camera matrix is already set, return
        if self.camera_matrix_right.size == 0:
        # if self.camera_matrix_right == []:
            # Set the camera matrix K
            self.camera_matrix_right = np.array(msg.k).reshape(3,3)


            # Set the distortion coefficients d
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
        # proj = camMatrix * ( R * T )
        projection_matrix_left = self.camera_matrix_left.dot( self.transformation_matrix_left )
        projection_matrix_right = self.camera_matrix_right.dot( self.transformation_matrix_right )
        
        # The below code does exactly the same as the above 2 lines. exact same projection matrices.
        # print('Projection Matrix Left shape:')
        # print(projection_matrix_left.shape)
        # print(projection_matrix_left)

        # KA_left = np.concatenate((self.camera_matrix_left, np.zeros((3, 1))), axis=1)
        # KA_right = np.concatenate((self.camera_matrix_right, np.zeros((3, 1))), axis=1)
        # T_left = np.eye(4, dtype=np.float64)
        # T_left[:3, :3]  = self.transformation_matrix_left[:3, :3]
        # T_left[:3, 3]   = self.transformation_matrix_left[:3, 3]
        # T_right = np.eye(4, dtype=np.float64)
        # T_right[:3, :3] = self.transformation_matrix_right[:3, :3]
        # T_right[:3, 3]  = self.transformation_matrix_right[:3, 3]

        # print('Camera Matrix Left shape:')
        # print(KA_left.shape)
        # print(KA_left)
        # print('Transformation Matrix Left shape:')
        # print(T_left.shape)
        # print(T_left)
        
        # projection_matrix_left = np.matmul(KA_left, T_left)
        # projection_matrix_right = np.matmul(KA_right, T_right)

        # print('Projection Matrix Left shape:')
        # print(projection_matrix_left.shape)
        # print(projection_matrix_left)

        # Test the projection matrix by projecting a point in the left camera frame
        point = np.array([0.1, -0.2, 0.1, 1])
        point_proj = projection_matrix_left.dot(point)
        point_proj = point_proj / point_proj[2]
        print('Projected point in left camera frame:')
        print(point_proj)
        
        # Get translation and rotation matrices for the left and right cameras
        t_left = self.transformation_matrix_left[0:3,3]
        r_left = self.transformation_matrix_left[0:3,0:3]
        t_right = self.transformation_matrix_right[0:3,3]
        r_right = self.transformation_matrix_right[0:3,0:3]
        
        # Compute rotation matrix from left to right camera
        r_left_to_right = r_right.dot(np.linalg.inv(r_left))
        t_left_to_right = t_right - r_left_to_right.dot(t_left)
        
        # print('Rotation matrix from left to right camera:')
        # print(r_left_to_right)
        # print('Translation vector from left to right camera:')
        # print(t_left_to_right)

        # Compute essential matrix
        #cv2.fundamentalFromProjections(projection_matrix_left, projection_matrix_right)
        S=np.mat([[0,-t_left_to_right[2],t_left_to_right[1]],[t_left_to_right[2],0,-t_left_to_right[1]],[-t_left_to_right[1],t_left_to_right[0],0]]) # Skew symmetric matrix
        print('S:')
        print(S)
        essential_matrix=np.mat(r_left_to_right)*S

        # print('Essential matrix:')
        # print(essential_matrix)

        # Compute fundamental matrix
        fundamental_matrix = np.linalg.inv(self.camera_matrix_right).T.dot(essential_matrix).dot(np.linalg.inv(self.camera_matrix_left))

        print('Fundamental matrix:')
        print(fundamental_matrix)
            
        # Define vectors for storing the triangulated circles
        mean_color_vec = []
        var_color_vec = []
        color_idx = -1

        left_points = np.array([])
        right_points = np.array([])
        left_points.shape = (0,2)  # Make it 2D so it concatenates properly
        right_points.shape = (0,2) # Make it 2D so it concatenates properly
        color_idx_vec = []
        
        self.get_logger().info('Minimum circles: %d' % min_num_circles)
        
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
            
            print("Matches circle %d with circle %d" % (i, matching_idx))
            
            # If no matching circle found, skip
            if matching_idx == -1:
                return
            
            # If matching circle is too far away, skip
            if min_dist > 100:
                return
            
            # Find average color of the two circles
            mean_color = []
            mean_color.append( (left_circle.bgr_mean[0] + matching_circle.bgr_mean[0]) / 2.0 )
            mean_color.append( (left_circle.bgr_mean[1] + matching_circle.bgr_mean[1]) / 2.0 )
            mean_color.append( (left_circle.bgr_mean[2] + matching_circle.bgr_mean[2]) / 2.0 )
            mean_color_vec.append(mean_color)

            # Find average variance of the two circles
            var_color = []
            var_color.append( (left_circle.bgr_var[0] + matching_circle.bgr_var[0]) / 2.0 )
            var_color.append( (left_circle.bgr_var[1] + matching_circle.bgr_var[1]) / 2.0 )
            var_color.append( (left_circle.bgr_var[2] + matching_circle.bgr_var[2]) / 2.0 )
            var_color_vec.append(var_color)
            
            # Find color channel with max value of circle in mean_color
            max_value = max(mean_color)
            color_idx = mean_color.index(max_value)

            # Create points for triangulation
            left_point = np.array([[left_circle.x, left_circle.y]])
            right_point = np.array([[matching_circle.x, matching_circle.y]]) 
            #np.append(right_points, right_point) # right_points.append(right_point)
            #np.append(left_points, left_point) # left_points.append(left_point)
            left_points = np.concatenate((left_points, left_point))
            right_points = np.concatenate((right_points, right_point))
            color_idx_vec.append(color_idx)

            # Printout for debugging
            # self.get_logger().info('Left point: %s' % left_point)
            # print(left_points)
            # self.get_logger().info('Right point: %s' % right_point)
            # print(right_points)

        # These operations can only be performed if there are at least 2 sets of points (circles matched)
        # We use the point matches to undistort the points and correct matches
        if len(left_points) >= 2:
            # Undistort points
            left_points = np.expand_dims(left_points, axis=1)
            right_points = np.expand_dims(right_points, axis=1)
            self.get_logger().info("Left points shape:")
            print(left_points.shape)
            print(left_points)
            self.get_logger().info("Right points shape:")
            print(right_points.shape)
            print(right_points)
        
            #left_points  = cv2.undistortPoints(left_points, self.camera_matrix_left, self.distortion_coeff_left)
            #right_points = cv2.undistortPoints(right_points, self.camera_matrix_right, self.distortion_coeff_right)

            # Remove extra dimension
            left_points = np.squeeze(left_points)
            right_points = np.squeeze(right_points)

            # Add more dimensions? (for cv2.correctMatches)
            # https://answers.opencv.org/question/341/python-correctmatches/
            left_points = np.reshape(left_points, (1, len(left_points), 2))
            right_points = np.reshape(right_points, (1, len(right_points), 2))

            self.get_logger().info("Undistorted Left points shape:")
            print(left_points.shape)
            print(left_points)
            self.get_logger().info("Undistorted Right points shape:")
            print(right_points.shape)
            print(right_points)
            
            # CorrectPoints
            #[left_points, right_points] = cv2.correctMatches(projection_matrix_left, left_points, right_points)
            #[left_points, right_points] = cv2.correctMatches(fundamental_matrix, left_points, right_points)

            # # Remove extra dimension
            # left_points = np.squeeze(left_points)
            # right_points = np.squeeze(right_points)

            self.get_logger().info("Correctmatched Left points shape:")
            print(left_points.shape)
            print(left_points)
            self.get_logger().info("Correctmatched Right points shape:")
            print(right_points.shape)
            print(right_points)
            
        else: # Otherwise: just reshape the points like correctmatches does
            left_points = np.reshape(left_points, (1, len(left_points), 2))
            right_points = np.reshape(right_points, (1, len(right_points), 2))


        # Triangulate points
        #left_points = left_points[0][0]
        #right_points = right_points[0][0]
        #print("Dummy test val for points:")
        #print(left_points)
        #print(right_points)
        # Left points and right points should be 1x2xN matrices, are currently 1xNx2
        # left_points = np.reshape(left_points, (1, 2, len(left_points)))
        # right_points = np.reshape(right_points, (1, 2, len(right_points)))
        #left_points = np.squeeze(left_points)
        #right_points = np.squeeze(right_points)
        #left_points = np.transpose(left_points)
        #right_points = np.transpose(right_points)
        # left_points = np.expand_dims(left_points, axis=0)
        # right_points = np.expand_dims(right_points, axis=0)
        

        
        #Sort the points by color channel and triangulate them individually
        triangulated_points = np.array([])
        triangulated_points.shape = (4, 0)
        for i in range(0, left_points.shape[1]):
            left = left_points[0][i]
            right = right_points[0][i]
            self.get_logger().info("Pre-triangulation Left points shape:")
            print(left.shape)
            print(left)
            self.get_logger().info("Pre-triangulation Right points shape:")
            print(right.shape)
            print(right)
            triangulated_point = cv2.triangulatePoints(projection_matrix_left, projection_matrix_right, left, right)
            self.get_logger().info("Triangulated point shape:")
            print(triangulated_point.shape)
            triangulated_points = np.hstack((triangulated_points, triangulated_point))
        print("Triangulated points ( unhomogenous ) shape:")
        print(triangulated_points.shape)
        print(triangulated_points)

        

        # Convert to homogenous coordinates
        triangulated_points = triangulated_points / triangulated_points[3]
        
        print("Triangulated points ( homogenous ) shape:")
        print(triangulated_points.shape)
        print(triangulated_points)
        
        self.get_logger().info('Triangulation has reached the end. There are %d triangulated circles' % triangulated_points.shape[1])

        # Get length of triangulated points as integer
        triangulated_points_len = triangulated_points.shape[1]
        print("Triangulated points length: % d" % triangulated_points_len)

        # Create TriangulatedCircleInfoArr message
        self._triangulated_circles = TriangulatedCircleInfoArr() # "Clear" the message
        for i in range(triangulated_points.shape[1]):
            triangulated_circle = TriangulatedCircleInfo()
            triangulated_circle.x = triangulated_points[0][i]
            triangulated_circle.y = triangulated_points[1][i]
            triangulated_circle.color = self.color_dict[color_idx_vec[i]]
            triangulated_circle.bgr_mean = mean_color_vec[i]
            triangulated_circle.bgr_var = var_color_vec[i]
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
        self._publisher_triangulated_circles.publish(self._triangulated_circles)



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