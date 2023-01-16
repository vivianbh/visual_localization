#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import numpy as np
from sympy import symbols, Eq, solve
import cv2

class depthTriangular():
    def __init__(self):
        rospy.init_node("depthCalculatorTriangularMed", anonymous=False)
        self.target0Pos_sub = rospy.Subscriber("/camera0/camera/disk_position", Point, self.target0PosCb)
        self.target1Pos_sub = rospy.Subscriber("/camera1/camera/disk_position", Point, self.target1PosCb)
        self.cam0Pos_sub = rospy.Subscriber("/camera0/vrpn_client_node/MAV1/pose", PoseStamped, self.cam0PosCb)
        self.cam1Pos_sub = rospy.Subscriber("/camera1/vrpn_client_node/MAV1/pose", PoseStamped, self.cam1PosCb)
        self.imgSize = (720, 1280) # <type 'tuple'>
        self.img0Coord = ()
        self.img1Coord = ()
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

    def target0PosCb(self, target_pos):
        self.img0Coord = self.camFrame2ImageFrame(target_pos)
        print('img0: ', self.img0Coord)

    def target1PosCb(self, target_pos):
        self.img1Coord = self.camFrame2ImageFrame(target_pos)
        print('img1: ', self.img1Coord)

    def camFrame2ImageFrame(self, position):
        # oringin at left-top corner
        object_real_width = 0.08
        w = self.cam0_matrix[0][0] * object_real_width / position.z
        x = position.x * self.cam0_matrix[0][0] / position.z + self.cam0_matrix[0][2]
        y = position.y * self.cam0_matrix[1][1] / position.z + self.cam0_matrix[1][2]

        # origin at center
        x = x - self.cam0_matrix[0][2]
        y = y - self.cam0_matrix[1][2]
        return (x,y)

    def cam0PosCb(self, mav_pos):
        T = np.array([-0.04, 0.1, -0.1])
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.cam0_position = np.array([mav_pos.pose.position.x, mav_pos.pose.position.y, mav_pos.pose.position.z]) + T
    
    def cam1PosCb(self, mav_pos):
        T = np.array([-0.04, 0.1, -0.1])
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.cam1_position = np.array([mav_pos.pose.position.x, mav_pos.pose.position.y, mav_pos.pose.position.z]) + T
        #print('mav1: ', mav_pos.pose.position.x, ', ', mav_pos.pose.position.y, ', ', mav_pos.pose.position.z)
        #print('cam1: ', self.cam1_position)

    def DepthCalculator(self):
        if self.cam0_position.all() != 0 and self.cam1_position.all() != 0 and len(self.img0Coord) != 0 and len(self.img1Coord) != 0:
            cam0_vector =  np.array([self.img0Coord[0], -self.img0Coord[1], -self.cam0_matrix[0][0]])
            cam1_vector =  np.array([self.img1Coord[0], -self.img1Coord[1], -self.cam0_matrix[0][0]])
            # solve intersection
            t1, t2 = symbols('t1, t2')
            eq1 = Eq(self.cam0_position[0] + t1 * cam0_vector[0] - t2 * cam1_vector[0] - self.cam1_position[0])
            #eq2 = Eq(self.cam0_position[1] + t1 * cam0_vector[1] - t2 * cam1_vector[1] - self.cam1_position[1])
            eq2 = Eq(self.cam0_position[2] + t1 * cam0_vector[2] - t2 * cam1_vector[2] - self.cam1_position[2])
            #print(eq1)
            #print(eq2)
            var = solve((eq1, eq2), (t1, t2))
            print(var[t1])
            depth = self.cam0_position[2] + var[t1] * cam0_vector[2]
            print('x: ', self.cam0_position[0] + var[t1] * cam0_vector[0], ' ', self.cam1_position[0] + var[t2] * cam1_vector[0])
            print('y: ', self.cam0_position[1] + var[t1] * cam0_vector[1], ' ', self.cam1_position[1] + var[t2] * cam1_vector[1])
            print('z: ', depth, ' ', self.cam1_position[2] + var[t2] * cam1_vector[2])

if __name__ == "__main__":
    dt = depthTriangular()
    
    while not rospy.is_shutdown():
        dt.DepthCalculator()

    rospy.spin()