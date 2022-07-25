#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import sys
import paramiko
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Img
from PIL import Image
from datetime import datetime
from config import config as cfg
from scp import SCPClient

size = (640, 480) # Resolution of QT2 RealSense camera
frameRate = 30.0 # Frame rate of QT2 RealSense camera
now = datetime.now()
fileName = "output/" + now.strftime("%d_%m_%Y-%H_%M_%S")
fourcc = cv.VideoWriter_fourcc(*'MJPG')
vid = cv.VideoWriter(fileName + ".avi", fourcc, frameRate, size, True)

def endProgram():
    print("Shutdown initiated. Ending program.")
    vid.release()
    dataPipeline(fileName + ".avi")

def visualRecorder(data):
    if(vid.isOpened()):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
        vid.write(img)
    else:
        rospy.signal_shutdown("OpenCV VideoWriter is no longer open. Program ending.")

def dataPipeline(filename):
    # Send video to host
    vid = open(filename)
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(cfg.HOSTNAME, 22, cfg.CV_USERNAME, cfg.CV_PASSCODE)
    scp = SCPClient(client.get_transport())
    scp.put(vid, remote_path=cfg.SERVER_PROJECT_PATH)

    # Receive data from host and publish it
    scp.get(cfg.SERVER_PROJECT_PATH + "/data.txt", "/input/")
    inputFile = open("/input/data.txt", "r")
    data = inputFile.read()
    inputFile.close()
    pub = rospy.Publisher(cfg.ROS_TYNAMO_TOPIC, std_msgs.msg.String, queue_size=10)
    pub.publish(data)

def qtActionCallback(data)
    decision = int(data)
    pub = rospy.Publisher("/qt_robot/emotion/show", std_msgs.msg.String, queue_size=10)
    if(decision == 0):
        pub.publish("data: 'QT/sad'")
    elif(decision == 1):
        pub.publish("data: 'QT/happy'")
    else:
        print("Invalid decision")

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.on_shutdown(endProgram)
    
    rospy.Subscriber(cfg.ROS_CAMERA_TOPIC, data, visualRecorder)
    rospy.Subscriber(cfg.ROS_TYNAMO_TOPIC, String, qtActionCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
