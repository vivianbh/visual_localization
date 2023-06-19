#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from pyquaternion import Quaternion
try:
   import queue
except ImportError:
   import Queue as queue
from sympy import symbols, Eq, solve
import cv2

class depthTriangular():
    def __init__(self):
        rospy.init_node("depthCalculatorTriangularMed", anonymous=False)
        self.targetWorldPos_sub = rospy.Subscriber("/camera0/world/disk_position", Point, self.targetWorldPos)
        self.target0Pos_sub = rospy.Subscriber("/camera0/camera/disk_position", Point, self.target0PosCb)
        self.target1Pos_sub = rospy.Subscriber("/camera1/camera/disk_position", Point, self.target1PosCb)
        self.cam0Pos_sub = rospy.Subscriber("/camera0/vrpn_client_node/MAV1/pose", PoseStamped, self.cam0PosCb)
        self.cam1Pos_sub = rospy.Subscriber("/camera1/vrpn_client_node/MAV1/pose", PoseStamped, self.cam1PosCb)
        self.loca_pub = rospy.Publisher("/world/target/position", Point, queue_size=10)
        self.imgSize = (720, 1280) # <type 'tuple'>
        self.img0Coord = ()
        self.img1Coord = ()
        self.targetWorld = ()
        self.loca_data = Point()
        self.mav0_quat = Quaternion()
        self.mav1_quat = Quaternion()
        # filter
        self.qA = queue.Queue(10)
        self.qB = queue.Queue(10)
        self.window = 10
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
        cam0_fx = 895.8833
        cam0_fy = 897.7566
        cam0_cx = 637.2570
        cam0_cy = 370.8199
        self.cam0_matrix = np.array([[cam0_fx,       0, cam0_cx],
                                    [      0, cam0_fy, cam0_cy],
                                    [      0,       0,       1]])
        self.cam0_distCoeffs = np.array([[0.1293, -0.2754, 0, 0]])
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
        imgCoord = self.camFrame2ImageFrame(target_pos)
        
        # MA filter
        if self.qA.full():
            self.qA.queue.clear()
        
        self.qA.put(imgCoord)
        if self.qA.full():
            u_sum = 0
            v_sum = 0
            for i in range(self.qA.qsize()):
                coord = self.qA.get()
                u_sum = u_sum + coord[0]
                v_sum = v_sum + coord[1]
            u_avg = u_sum / self.window
            v_avg = v_sum / self.window

            self.img0Coord = (u_avg, v_avg)
            #print('img0: ', self.img0Coord)

    def target1PosCb(self, target_pos):
        imgCoord = self.camFrame2ImageFrame(target_pos)
        
        # MA filter
        if self.qB.full():
            self.qB.queue.clear()
        
        self.qB.put(imgCoord)
        if self.qB.full():
            u_sum = 0
            v_sum = 0
            for i in range(self.qB.qsize()):
                coord = self.qB.get()
                u_sum = u_sum + coord[0]
                v_sum = v_sum + coord[1]
            u_avg = u_sum / self.window
            v_avg = v_sum / self.window

            self.img1Coord = (u_avg, v_avg)
            #print('img1: ', self.img1Coord)

    def camFrame2ImageFrame(self, position):
        # oringin at left-top corner
        object_real_width = 0.08
        w = self.cam0_matrix[0][0] * object_real_width / position.z
        x = position.x * self.cam0_matrix[0][0] / position.z + self.cam0_matrix[0][2]
        y = position.y * self.cam0_matrix[1][1] / position.z + self.cam0_matrix[1][2]

        # origin at center
        #print(x, ' , ', y)
        x = x - self.cam0_matrix[0][2]
        y = y - self.cam0_matrix[1][2]
        #print(x, ' , ', y)
        return (x,y)

    def cam0PosCb(self, mav_pos):
        T = np.array([-0.04, 0.1, -0.1])
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.cam0_position = np.array([mav_pos.pose.position.x, mav_pos.pose.position.y, mav_pos.pose.position.z]) + T
        self.mav0_quat = Quaternion(mav_pos.pose.orientation.w, mav_pos.pose.orientation.x, mav_pos.pose.orientation.y, mav_pos.pose.orientation.z)
    
    def cam1PosCb(self, mav_pos):
        T = np.array([-0.04, 0.1, -0.1])
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.cam1_position = np.array([mav_pos.pose.position.x, mav_pos.pose.position.y, mav_pos.pose.position.z]) + T
        self.mav1_quat = Quaternion(mav_pos.pose.orientation.w, mav_pos.pose.orientation.x, mav_pos.pose.orientation.y, mav_pos.pose.orientation.z)
        #print('mav1: ', mav_pos.pose.position.x, ', ', mav_pos.pose.position.y, ', ', mav_pos.pose.position.z)
        #print('cam1: ', self.cam1_position)

    def DepthCalculator(self):
        if self.cam0_position.all() != 0 and self.cam1_position.all() != 0 and len(self.img0Coord) != 0 and len(self.img1Coord) != 0:
            cam0_vector =  np.array([self.img0Coord[0], -self.img0Coord[1], -self.cam0_matrix[0][0], 1])
            cam1_vector =  np.array([self.img1Coord[0], -self.img1Coord[1], -self.cam0_matrix[0][0], 1])

            cam0_vectorW = self.sensor2WorldFrame(cam0_vector, self.mav0_quat) #cam0_vector[:3] #
            cam1_vectorW = self.sensor2WorldFrame(cam1_vector, self.mav1_quat) #cam1_vector[:3] #

            # solve intersection
            t1, t2 = symbols('t1, t2')
            print('camera position: ', self.cam0_position, '    ', self.cam1_position)
            eq1 = Eq(self.cam0_position[0] + t1 * cam0_vectorW[0], t2 * cam1_vectorW[0] + self.cam1_position[0])
            eq2 = Eq(self.cam0_position[1] + t1 * cam0_vectorW[1], t2 * cam1_vectorW[1] + self.cam1_position[1])
            #eq2 = Eq(self.cam0_position[2] + t1 * cam0_vectorW[2], t2 * cam1_vectorW[2] + self.cam1_position[2])
            #print(eq1)
            #print(eq2)
            var = solve((eq1, eq2), (t1, t2))
            print('var: ', var)
            #print('t1: ', var[t1])
            #print('t2: ', var[t2])
            x = self.cam0_position[0] + var[t1] * cam0_vectorW[0]
            y = self.cam0_position[1] + var[t1] * cam0_vectorW[1]
            depth = self.cam0_position[2] + var[t1] * cam0_vectorW[2]
            print('x: ', self.cam0_position[0] + var[t1] * cam0_vectorW[0], ' ', self.cam1_position[0] + var[t2] * cam1_vectorW[0])
            print('y: ', self.cam0_position[1] + var[t1] * cam0_vectorW[1], ' ', self.cam1_position[1] + var[t2] * cam1_vectorW[1])
            print('z: ', depth, ' ', self.cam1_position[2] + var[t2] * cam1_vectorW[2])

            self.loca_data.x = x
            self.loca_data.y = y
            self.loca_data.z = depth
            self.loca_pub.publish(self.loca_data)
            '''
            err_x = abs((x - self.targetWorld[0])) /self.targetWorld[0] * 100
            err_y = abs((y - self.targetWorld[1])) /self.targetWorld[1] * 100
            err_z = abs((depth - self.targetWorld[2])) /self.targetWorld[2] * 100
            print(err_x, err_y, err_z)
            '''

    def targetWorldPos(self, pos):
        self.targetWorld = (pos.x, pos.y, pos.z)
    
    def sensor2WorldFrame(self, cam_vec, mav_quat):
        q_sensor2Body = Quaternion(axis=[0.0, 0.0, 1.0], angle = -math.pi/2)
        Q = mav_quat.inverse * q_sensor2Body
        #print(Q.transformation_matrix)
        new_cam_vec =  np.dot(Q.transformation_matrix, cam_vec.reshape(4,1))
        #print(new_cam_vec)

        return new_cam_vec[:3, 0]


if __name__ == "__main__":
    dt = depthTriangular()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dt.DepthCalculator()
        rate.sleep()

    #rospy.spin()