#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import ros_numpy
import numpy as np
import cv2

class imageConverter():
    def __init__(self):
        rospy.init_node("depthCalculatorStereoMed", anonymous=False)
        self.cam0_sub = rospy.Subscriber("/camera0/camera_image/detected", Image, self.cam0_callback)
        self.cam1_sub = rospy.Subscriber("/camera1/camera_image/detected", Image, self.cam1_callback)
        #self.target_sub = rospy.Subscriber("/camera0/camera/disk_position", Point, self.targetPosCb)
        self.cam0Pos_sub = rospy.Subscriber("/camera0/vrpn_client_node/MAV1/pose", PoseStamped, self.cam0PosCb)
        self.cam1Pos_sub = rospy.Subscriber("/camera1/vrpn_client_node/MAV1/pose", PoseStamped, self.cam1PosCb)
        self.gray_img0 = np.ndarray(0)
        self.gray_img1 = np.ndarray(0)
        self.counter = 0
        self.counter_old = 0
        self.imgSize = (720, 1280) # <type 'tuple'>
        #################### camera's parameter ####################
        self.cam0_matrix = np.array(0)
        self.cam1_matrix = np.array(0)
        self.cam0_matrix = np.array(0)
        self.cam1_matrix = np.array(0)
        self.camParamSet()
        #################### camera's pose ####################
        self.cam0_position = np.array(0)
        self.cam1_position = np.array(0)
        self.cam0_rotMatrix = np.array(0)
        self.cam1_rotMatrix = np.array(0)

        self.T = np.array(0)
        self.R = np.array(0)
        self.E = np.array(0)
        self.F = np.array(0)

    def camParamSet(self):
        #################### camera0's intrinsic parameter ####################
        cam0_fx = 893.7527
        cam0_fy = 895.8022
        cam0_cx = 648.9854
        cam0_cy = 374.8394
        self.cam0_matrix = np.array([[cam0_fx,       0, cam0_cx],
                                    [      0, cam0_fy, cam0_cy],
                                    [      0,       0,       1]])
        self.cam0_distCoeffs = np.array([[0.1083, -0.2401, 0, 0]])
        #################### camera1's intrinsic parameter ####################
        cam1_fx = 893.7527
        cam1_fy = 895.8022
        cam1_cx = 648.9854
        cam1_cy = 374.8394
        self.cam1_matrix = np.array([[cam1_fx,       0, cam1_cx],
                                    [      0, cam1_fy, cam1_cy],
                                    [      0,       0,       1]])
        self.cam1_distCoeffs = np.array([[0.1083, -0.2401, 0, 0]])

        #print('cam0 in-matrix: ', self.cam0_matrix)
        #print('cam2 in-matrix: ', self.cam1_matrix)
        #print('cam0 distCoeffs: ', self.cam0_distCoeffs)
        #print('cam0 distCoeffs: ', self.cam1_distCoeffs)

    def camParamCalculate(self):
        if self.cam1_position.all() != 0 and self.cam0_position.all() != 0:
            self.T = self.cam1_position - self.cam0_position
            self.R = self.cam1_matrix.dot(np.linalg.inv(self.cam0_matrix))
            S = np.array([[0, -self.T[2], -self.T[1]], [self.T[2], 0, -self.T[0]], [-self.T[1], self.T[0], 0]])
            self.E = self.R.dot(S)
            F_int = ((np.linalg.inv(self.cam1_rotMatrix)).transpose()).dot(self.E) 
            self.F = F_int.dot((np.linalg.inv(self.cam0_rotMatrix)))

            print('T: ', self.T)
            print('R: ', self.R)
            #print('S: ', S)
            #print('E: ', self.E)
            #print('F: ', self.F)
            #print('self.imgSize: ', self.imgSize)

    def stereoRectify(self):
        if self.cam1_position.all() != 0 and self.cam0_position.all() != 0 and self.gray_img0.all() != 0 and self.gray_img1.all() != 0:
            self.camParamCalculate()

            ################ stereoRectify ################ 
            (cam0_Rectification, cam1_Rectification, cam0_Projection, cam1_Projection, dispartityToDepthMap, cam0_ROI, cam1_ROI) = cv2.stereoRectify(
                    self.cam0_matrix, self.cam0_distCoeffs,
                    self.cam1_matrix, self.cam1_distCoeffs,
                    self.imgSize, self.R, self.T,
                    None, None, None, None, None,
                    cv2.CALIB_ZERO_DISPARITY, -1)

            cam0_MapX, cam0_MapY = cv2.initUndistortRectifyMap(
            self.cam0_matrix, self.cam0_distCoeffs, cam0_Rectification,
            cam0_Projection, self.imgSize, cv2.CV_32FC1)
            
            cam1_MapX, cam1_MapY = cv2.initUndistortRectifyMap(
            self.cam1_matrix, self.cam1_distCoeffs, cam1_Rectification,
            cam1_Projection, self.imgSize, cv2.CV_32FC1)

            ################ calculate a depth map ################
            stereoMatcher = cv2.StereoBM_create()
            
            stereoMatcher.setMinDisparity(4)
            stereoMatcher.setNumDisparities(128)
            stereoMatcher.setBlockSize(11)
            stereoMatcher.setSpeckleRange(16)
            stereoMatcher.setSpeckleWindowSize(45)
            
            REMAP_INTERPOLATION = cv2.INTER_NEAREST
            
            fixed_img0 = cv2.remap(self.gray_img0, cam0_MapX, cam0_MapY, REMAP_INTERPOLATION)
            fixed_img1 = cv2.remap(self.gray_img1, cam1_MapX, cam1_MapY, REMAP_INTERPOLATION)

            
            #cv2.imshow('img0', np.uint8(self.gray_img1))
            #cv2.imshow('img0', fixed_img0)
            #cv2.imshow('img1', fixed_img1)

            #print('fixed_img0',type(fixed_img0))
            #gray_img0 = cv2.cvtColor(fixed_img0, cv2.COLOR_BGR2GRAY)
            #gray_img1 = cv2.cvtColor(fixed_img1, cv2.COLOR_BGR2GRAY)
            depthMap = stereoMatcher.compute(np.uint8(fixed_img0), np.uint8(fixed_img1))
            print(self.gray_img0)
            print(np.uint8(self.gray_img0))
            #depthMap = stereoMatcher.compute(np.uint8(self.gray_img0), np.uint8(self.gray_img1))
            print(depthMap)
            
            DEPTH_VISUALIZATION_SCALE = 2048
            #cv2.imshow('depth', depthMap / DEPTH_VISUALIZATION_SCALE)
            #cv2.waitKey(0)

    def cam0_callback(self, msg_img):
        self.gray_img0 = ros_numpy.numpify(msg_img)
        [height, width] = self.gray_img0.shape
        self.counter = self.counter + 1
        #print("The number of image is:", self.counter)
        #cv2.imshow('img0', self.gray_img0)
    
    def cam1_callback(self, msg_img):
        self.gray_img1 = ros_numpy.numpify(msg_img)
        self.counter = self.counter + 1
        #print("The number of image is:", self.counter)
        #cv2.imshow('img1', self.gray_img1)

    def cam0PosCb(self, mav_pos):
        T = np.array([-0.04, 0.1, -0.1])
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.cam0_position = np.array([mav_pos.pose.position.x, mav_pos.pose.position.y, mav_pos.pose.position.z]) - T
        print('self.cam0_position: ', self.cam0_position)
        orientation = np.array([mav_pos.pose.orientation.x, mav_pos.pose.orientation.y, mav_pos.pose.orientation.z, mav_pos.pose.orientation.w])
        mav_rotMatrix = self.quaternion_rotation_matrix(orientation)
        self.cam0_rotMatrix = np.linalg.inv(R).dot(mav_rotMatrix)
        print('self.cam0_rotMatrix: ', self.cam0_rotMatrix)
    
    def cam1PosCb(self, mav_pos):
        T = np.array([-0.04, 0.1, -0.1])
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.cam1_position = np.array([mav_pos.pose.position.x, mav_pos.pose.position.y, mav_pos.pose.position.z]) - T
        print('self.cam1_position: ', self.cam1_position)
        orientation = np.array([mav_pos.pose.orientation.x, mav_pos.pose.orientation.y, mav_pos.pose.orientation.z, mav_pos.pose.orientation.w])
        mav_rotMatrix = self.quaternion_rotation_matrix(orientation)
        self.cam1_rotMatrix = np.linalg.inv(R).dot(mav_rotMatrix)
        print('self.cam1_rotMatrix: ', self.cam1_rotMatrix)

    def quaternion_rotation_matrix(self, Q):
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix

if __name__ == "__main__":
    ic = imageConverter()
    rospy.sleep(2)
    
    while not rospy.is_shutdown():
        #ic.camParamCalculate()
        ic.stereoRectify()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()