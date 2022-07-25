#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import sys
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Img
from PIL import Image
from datetime import datetime
from config import config

size = (640, 480) # Resolution of QT2 RealSense camera
frameRate = 30.0 # Frame rate of QT2 RealSense camera
now = datetime.now()
fileName = "output/" + now.strftime("%d_%m_%Y-%H_%M_%S")
fourcc = cv.VideoWriter_fourcc(*'MJPG')
vid = cv.VideoWriter(fileName + ".avi", fourcc, frameRate, size, True)

def endProgram():
    print("Shutdown initiated. Ending program.")
    vid.release()

def visualRecorder(data):
    if(vid.isOpened()):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
        vid.write(img)
    else:
        rospy.signal_shutdown("OpenCV VideoWriter is no longer open. Program ending.")

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.on_shutdown(endProgram)
    
    rospy.Subscriber(config.ROS_TOPIC, Img, visualRecorder)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
