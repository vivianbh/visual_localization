#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
import ros_numpy
import numpy as np
import cv2

class imageConverter():
    def __init__(self):
        rospy.init_node("imageConverter_py", anonymous=False)
        self.img_sub = rospy.Subscriber("/camera_image/detected", Image, self.callback)
        self.img = np.ndarray(0)
        self.counter = 0
        self.counter_old = 0

    def callback(self, msg_img):
        self.img = ros_numpy.numpify(msg_img)
        self.counter = self.counter + 1
        print("The number of image is:", self.counter)
        
        self.img = np.asanyarray(self.img)
        print('type: ', type(self.img))
        cv2.imshow('image', self.img)
        cv2.waitKey(0)

        # write image into file
        filename = 'camera0_' + str(self.counter) + '.jpg'
        cv2.imwrite(filename, self.img)

        self.img = np.ndarray(0)
        rospy.sleep(1)

if __name__ == "__main__":
        ic = imageConverter()
        rospy.spin()
'''
        while not rospy.is_shutdown():
            print('Enter2')
            ic.storeImage()
'''
        