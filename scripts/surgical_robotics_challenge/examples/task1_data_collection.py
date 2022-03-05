#!/usr/bin/env python
from __future__ import print_function

import sys
import os

import rospy
# import ambf
from ambf_client import Client
from cv_bridge import CvBridge, CvBridgeError
# from pynput.keyboard import Key, Listener
from sensor_msgs.msg import Image

import json
import cv2
import numpy as np
from numpy.linalg import inv
from surgical_robotics_challenge.scene import Scene
from surgical_robotics_challenge.camera import Camera
from ambf_client import Client
import time
from datetime import datetime
import tf_conversions.posemath as pm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

np.set_printoptions(precision=3, suppress=True)


class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.imgL_subs = rospy.Subscriber(
            "/ambf/env/cameras/cameraL/ImageData", Image, self.left_callback
        )
        self.imgR_subs = rospy.Subscriber(
            "/ambf/env/cameras/cameraR/ImageData", Image, self.right_callback
        )

        self.left_frame = None
        self.left_ts = None
        self.right_frame = None
        self.right_ts = None

        # Wait until subscribers and publishers are ready
        rospy.sleep(0.5)

    def left_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.left_frame = cv2_img
            self.left_ts = msg.header.stamp
        except CvBridgeError as e:
            print(e)

    def right_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.right_frame = cv2_img
            self.right_ts = msg.header.stamp
        except CvBridgeError as e:
            print(e)

    def save_data(self, T_WN, T_WF, T_WL, T_WR, img_pt_L, img_pt_R, folder_path):
        print('Saving the current image')
        now = datetime.now()

        if not cv2.imwrite(folder_path + now.strftime("%Y-%m-%d-%H-%M-%S") + "_L.jpg", self.left_frame):
            raise Exception("Could not write image")

        if not cv2.imwrite(folder_path + now.strftime("%Y-%m-%d-%H-%M-%S") + "_R.jpg", self.right_frame):
            raise Exception("Could not write image")

        with open(folder_path + now.strftime("%Y-%m-%d-%H-%M-%S") + "_coords_L.txt", 'a') as f:
            for i in range(img_pt_L.shape[0]):
                print(str(int(img_pt_L[i, 0, 0])) + ' ' + str(int(img_pt_L[i, 0, 1])), file=f)

        with open(folder_path + now.strftime("%Y-%m-%d-%H-%M-%S") + "_coords_R.txt", 'a') as f:
            for i in range(img_pt_L.shape[0]):
                print(str(int(img_pt_R[i, 0, 0])) + ' ' + str(int(img_pt_R[i, 0, 1])), file=f)

        with open(folder_path + now.strftime("%Y-%m-%d-%H-%M-%S") + "_transforms.txt", 'a') as f:
            print('needle', file=f)
            print('\n'.join(map(lambda b: ' '.join(map(str, b)), T_WN)), file=f)
            print('cam_frame', file=f)
            print('\n'.join(map(lambda b: ' '.join(map(str, b)), T_WF)), file=f)
            print('camL', file=f)
            print('\n'.join(map(lambda b: ' '.join(map(str, b)), T_WL)), file=f)
            print('camR', file=f)
            print('\n'.join(map(lambda b: ' '.join(map(str, b)), T_WR)), file=f)
        pass

def get_cam_params():
    fvg = 1.2
    width = 1920
    height = 1080
    f = height / (2 * np.tan(fvg / 2))

    intrinsic_params = np.zeros((3, 3))
    intrinsic_params[0, 0] = f
    intrinsic_params[1, 1] = f
    intrinsic_params[0, 2] = width / 2
    intrinsic_params[1, 2] = height / 2
    intrinsic_params[2, 2] = 1.0

    print("intrinsic parameters:")
    print(intrinsic_params)

    return intrinsic_params

if __name__ == "__main__":
    # Connect to AMBF and setup image suscriber
    rospy.init_node("image_listener")
    saver = ImageSaver()

    c = Client("client_n")
    c.connect()
    time.sleep(0.3)

    scene = Scene(c)
    ambf_cam_l = Camera(c, "cameraL")
    ambf_cam_r = Camera(c, "cameraR")
    ambf_cam_frame = Camera(c, "CameraFrame")

    intrinsic_params = get_cam_params()
    
    # Get pose for the needle and the camera
    T_WN = pm.toMatrix(scene.needle_measured_cp())  # Needle to world
    T_FL = pm.toMatrix(ambf_cam_l.get_T_c_w())  # CamL to CamFrame
    T_FR = pm.toMatrix(ambf_cam_r.get_T_c_w())  # CamR to CamFrame
    T_WF = pm.toMatrix(ambf_cam_frame.get_T_c_w())  # CamFrame to world

    # Get image
    imgL = saver.left_frame
    imgR = saver.right_frame

    #Calculate needle to left camera transformation
    T_WL = T_WF.dot(T_FL)
    T_LN = inv(T_WL).dot(T_WN)

    #Calculate needle to right camera transformation
    T_WR = T_WF.dot(T_FR)
    T_RN = inv(T_WR).dot(T_WN)

    # Convert AMBF camera axis to Opencv Camera axis
    F = np.array([[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
    T_LN_CV2 = F.dot(T_LN)
    T_RN_CV2 = F.dot(T_RN)

    # Project center of the needle with OpenCv
    rvecsL, _ = cv2.Rodrigues(T_LN_CV2[:3, :3])
    tvecsL = T_LN_CV2[:3, 3]
    rvecsR, _ = cv2.Rodrigues(T_RN_CV2[:3, :3])
    tvecsR = T_RN_CV2[:3, 3]
    # needle_salient points 
    theta = np.linspace(np.pi / 3, np.pi, num=9).reshape((-1, 1))
    radius = 0.1018
    needle_salient = radius * np.hstack((np.cos(theta), np.sin(theta), theta * 0))
    
    #Project points
    img_pt_L, _ = cv2.projectPoints(
        needle_salient,
        rvecsL,
        tvecsL,
        intrinsic_params,
        np.float32([0, 0, 0, 0, 0]),
    )

    img_pt_R, _ = cv2.projectPoints(
        needle_salient,
        rvecsR,
        tvecsR,
        intrinsic_params,
        np.float32([0, 0, 0, 0, 0]),
    )

    # # Display image
    # for i in range(img_pt_L.shape[0]):
    #     img_L = cv2.circle(
    #         imgL, (int(img_pt_L[i, 0, 0]), int(img_pt_L[i, 0, 1])), 3, (255, 0, 0), -1
    #     )
    # for i in range(img_pt_R.shape[0]):
    #     img_R = cv2.circle(
    #         imgR, (int(img_pt_R[i, 0, 0]), int(img_pt_R[i, 0, 1])), 3, (255, 0, 0), -1
    #     )
    #
    # cv2.imshow("img_L", img_L)
    # cv2.imshow("img_R", img_R)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    saver.save_data(T_WN, T_WF, T_WL, T_WR,
                    img_pt_L, img_pt_R,
                    folder_path="/home/practicepoint/yiwei/surgical_robotics_challenge/scripts/surgical_robotics_challenge/examples/task1_data/")
    

    
