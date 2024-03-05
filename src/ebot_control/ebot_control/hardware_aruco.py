#!/usr/bin/env python3
#ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1a.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import time
import pickle
from std_msgs.msg import String,Int32
from collections import deque
from scipy.ndimage import median_filter
from scipy.ndimage import gaussian_filter1d
##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############
    (topLeft, topRight, bottomRight, bottomLeft) = coordinates
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))
    # You can remove these variables after reading the instructions. These are just for sample.

    area = (((bottomLeft[0]-bottomRight[0])**2+(bottomLeft[1]-bottomRight[1])**2)**0.5)*(((bottomLeft[0]-topLeft[0])**2+(bottomLeft[1]-topLeft[1])**2)**0.5)
    width = (((bottomLeft[0]-bottomRight[0])**2+(bottomLeft[1]-bottomRight[1])**2)**0.5)

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width


def detect_aruco(image,flag):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    #cam_mat = np.array([[915.3003540039062, 0.0, 642.724365234375], [0.0, 914.0320434570312, 361.9780578613281], [0.0, 0.0, 1.0]])
    cam_mat = np.array([[915.3003540039062, 0.0, 642.724365234375], [0.0, 914.0320434570312, 361.9780578613281], [0.0, 0.0, 1.0]])
    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15
    marker_points = np.array([[-size_of_aruco_m / 2, size_of_aruco_m / 2, 0],
                              [size_of_aruco_m / 2, size_of_aruco_m / 2, 0],
                              [size_of_aruco_m / 2, -size_of_aruco_m / 2, 0],
                              [-size_of_aruco_m/ 2, -size_of_aruco_m / 2, 0]], dtype=np.float32)
    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    ids2 = []
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : (0.788*angle_aruco) - ((angle_aruco**2)/3160)


    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################
    image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #image=cv2.GaussianBlur(image, (5,5), 0)
    image=cv2.convertScaleAbs(image, alpha=1.0, beta=3.0)
    #image=cv2.equalizeHist(image)
    '''
    image = cv2.adaptiveThreshold(
        image,  # input image
        255,       # max pixel value
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,  # adaptive method (Gaussian)
        cv2.THRESH_BINARY,  # binarization type
        11,        # block size (size of the neighborhood for adaptive thresholding)
        2          # constant subtracted from the mean for adaptive thresholding
    )
    '''
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    (corners, ids, rejected) = detector.detectMarkers(image)
    if len(corners) > 0 and flag==1:
        ids = ids.flatten()
        for (markerCorner,markerID) in zip(corners,ids):
            corners = markerCorner.reshape((4, 2))
            Rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.15, cam_mat, dist_mat)

            (area,width)=calculate_rectangle_area(corners)
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            cv2.drawFrameAxes(image, cam_mat, dist_mat, Rvec, tvec, 0.8)
            cv2.putText(image,str(markerID),(cX,cY),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
            a,w=calculate_rectangle_area(corners)
            rotation_matrix, _ = cv2.Rodrigues(Rvec)
            yaw_pitch_roll = cv2.RQDecomp3x3(rotation_matrix)
            angle = np.arccos(np.trace(rotation_matrix))
            if(a>aruco_area_threshold):
                center_aruco_list.append((cX,cY))
                width_aruco_list.append(w)
                distance_from_rgb_list.append(tvec)
                width_aruco_list.append(w)
                angle_aruco_list.append(yaw_pitch_roll[0][1])
                ids2.append(markerID)
        """cv2.imshow('image',image)
        cv2.waitKey(10)"""
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids2


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        self.flag_listen_sub = self.create_subscription(Int32,'/flag',self.flagcb,10)
        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 5.0                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)   
        self.publisher_ = self.create_publisher(String, 'topic', 50)
       
        # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.flag=1
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        self.id_queue_dict={}

    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''
        
	############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################
        self.depth_image=self.bridge.imgmsg_to_cv2(data)

    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''
        
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same
        ############################################
        self.cv_image=self.bridge.imgmsg_to_cv2(data)

    def flagcb(self,data):
        
        self.flag=data.data
        print(self.flag)

    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''
        center_list,distance_list,angle_list,width_list,id_list=detect_aruco(self.cv_image,self.flag)
        msg = String()
        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 642.724365234375
        centerCamY = 361.9780578613281
        focalX = 915.3003540039062
        focalY = 914.0320434570312
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)
        rectified_angles=[]
        rectified_distance=[]
        q=[]
        message=[]
        if(len(self.id_queue_dict) == 0):
            self.id_queue_dict={id: deque([0.0]*10) for id in id_list}

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        for(distance,angle1,center,id) in zip(distance_list,angle_list,center_list,id_list):
            cX=center[0]/1000
            cY=center[1]/1000
            temp=['',[],[],[],[],[],[],0.0]
            temp[0]="box"+str(id)
            
            
  # Replace x, y, and z with your vector's components

# Define the rotation angle in radians (90 degrees)
            angle = np.pi / 2  # 90 degrees in radians

# Create the rotation matrix for Y-axis rotation
            rotation_matrix = np.array([
                [np.cos(angle), 0, np.sin(angle)],
                [0, 1, 0],
                [-np.sin(angle), 0, np.cos(angle)]
            ])
            distance = np.dot(rotation_matrix, distance[0][0])
            theta = np.radians(90)

# Create the rotation matrix around the Z-axis
            angle_rad = np.pi*1.5 

# Define the rotation matrix for a rotation about the X-axis
            rotation_matrix = np.array([
                [1, 0, 0],
                [0, np.cos(angle_rad), -np.sin(angle_rad)],
                [0, np.sin(angle_rad), np.cos(angle_rad)]
            ])
            distance = np.dot(rotation_matrix, distance)
            x = distance[0]
            y = distance[1]
            z=distance[2]
            
            rectified_distance.append([x,y,z])
            
           
            #r_angle = ((0.788*angle1) - ((angle1**2)/3160))
            #r_angle=-r_angle
            r_angle=-angle1
            #r_angle=self.filter_data(r_angle,id)
            print('box_'+str(id)+" : "+str(round(r_angle,3)))
            temp[7]=r_angle
            rectified_angles.append(r_angle)
            r = R.from_euler('xyz', [0,0,r_angle], degrees=True)
            r_matrix = r.as_matrix()
               
            theta=np.pi/2
            R_y_rotation = np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])
            r_matrix=np.dot(r_matrix, R_y_rotation)
            
            theta=np.pi/2
            R_z_rotation = np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])
            r_matrix=np.dot(r_matrix, R_z_rotation)
            r = R.from_matrix(r_matrix)
            
            '''
            angle=np.deg2rad(5)
            x_rotation_matrix = np.array([[1, 0, 0],[0, np.cos(angle), -np.sin(angle)],[0, np.sin(angle), np.cos(angle)]])
            r_matrix=np.dot(r_matrix, x_rotation_matrix)  
            '''
            r_matrix=list(r_matrix)
            q = r.as_quat()
            temp[6]=list(q)
            offset=[0.0,0,0.1]
            offset=np.dot(r_matrix,offset)
            x=float(x)+offset[0]
            y=float(y)+offset[1]
            z=float(z)+offset[2]
            t.header.frame_id = 'camera_link'
            idstr=str(id)
            t.child_frame_id = 'cam_'+idstr
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            t.transform.translation.x =float(x) 
            t.transform.translation.y =float(y)
            t.transform.translation.z =float(z)
            self.br.sendTransform(t)
            d1=[0,0,0.08]
            d2=[0,0,0.27]
            point_0_1=np.dot(r_matrix,d1)
            point_0_2=np.dot(r_matrix,d2)
            point_0_1[0]=float(x)-point_0_1[0]
            point_0_1[1]=float(y)-point_0_1[1]
            point_0_1[2]=float(z)-point_0_1[2]
            point_0_2[0]=float(x)-point_0_2[0]
            point_0_2[1]=float(y)-point_0_2[1]
            point_0_2[2]=float(z)-point_0_2[2]
            t.header.frame_id = 'camera_link'
            t.child_frame_id = 'post_pick1_'+idstr
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            t.transform.translation.x =float(point_0_1[0]) 
            t.transform.translation.y =float(point_0_1[1])
            t.transform.translation.z =float(point_0_1[2])
            self.br.sendTransform(t)
            t.header.frame_id = 'camera_link'
            t.child_frame_id = 'post_pick2_'+idstr
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            t.transform.translation.x =float(point_0_2[0]) 
            t.transform.translation.y =float(point_0_2[1])
            t.transform.translation.z =float(point_0_2[2])
            self.br.sendTransform(t)
            try:
                    t2 = self.tf_buffer.lookup_transform('base_link','cam_'+idstr, rclpy.time.Time() )
                    t3 = self.tf_buffer.lookup_transform('base_link','post_pick1_'+idstr, rclpy.time.Time() )
                    t4 = self.tf_buffer.lookup_transform('base_link','post_pick2_'+idstr, rclpy.time.Time() )
                    temp[3]=[t2.transform.translation.x,t2.transform.translation.y,t2.transform.translation.z]
                    temp[2]=[t3.transform.translation.x,t3.transform.translation.y,t3.transform.translation.z]
                    temp[1]=[t4.transform.translation.x,t4.transform.translation.y,t4.transform.translation.z]
                    temp[5]=[t3.transform.rotation.x,t3.transform.rotation.y,t3.transform.rotation.z,t3.transform.rotation.w]
                    temp[4]=[t4.transform.rotation.x,t4.transform.rotation.y,t4.transform.rotation.z,t4.transform.rotation.w]
                    t.header.frame_id = 'base_link'
                    t.child_frame_id = 'obj_'+idstr
                    t.transform.rotation.x = t2.transform.rotation.x
                    t.transform.rotation.y = t2.transform.rotation.y
                    t.transform.rotation.z = t2.transform.rotation.z
                    t.transform.rotation.w = t2.transform.rotation.w
                    t.transform.translation.x =t2.transform.translation.x 
                    t.transform.translation.y =t2.transform.translation.y
                    t.transform.translation.z =t2.transform.translation.z
                    self.br.sendTransform(t)
                    message.append(temp)
            except tf2_ros.LookupException as e:
                print(f"LookupException: {e}")
                break
            except tf2_ros.ConnectivityException as e2:
                print(f"ConnectivityException: {e2}")
                break
        messagestr=str(message)
        msg.data = messagestr
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
    def filter_data(self,angle,id):
        self.id_queue_dict[id].append(angle)
        self.id_queue_dict[id].popleft()
        non_zero_list=[element for element in self.id_queue_dict[id] if element != 0.0]
        non_zero_count = sum(1 for element in self.id_queue_dict[id] if element != 0.0)
        i_cant_think_of_any_variable=[0.0]
        if non_zero_count<10:
            non_zero_list=gaussian_filter1d(non_zero_list,0.65)
            non_zero_list=median_filter(non_zero_list,non_zero_count)
            self.id_queue_dict[id].clear()
            i_cant_think_of_any_variable=[0.0]*(10-len(non_zero_list))
            
            i_cant_think_of_any_variable.extend(non_zero_list)
            
            self.id_queue_dict[id]=deque(i_cant_think_of_any_variable)
        else:
            non_zero_list=gaussian_filter1d(non_zero_list,0.65)
            non_zero_list=median_filter(non_zero_list,non_zero_count)
            self.id_queue_dict[id].clear()
            self.id_queue_dict[id]=deque(non_zero_list)

        return sum(self.id_queue_dict[id])/non_zero_count



        
        ###########################################

##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
