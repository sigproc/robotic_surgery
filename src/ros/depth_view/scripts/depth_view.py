#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import numpy as np

from sensor_msgs.msg import Image

BRIDGE = CvBridge()
WINDOW_NAME = "Depth view"

def new_image(image):
    try:
        cv_image = BRIDGE.imgmsg_to_cv2(image, "16UC1")
    except CvBridgeError as e:
        rospy.logerr("Error converting image: {0}".format(e.message))

    # Convert depth image to floating point image varying between 0 and 1
    depth = np.asarray(cv_image).astype(np.float32) / cv_image.max()
    depth = depth[...,0]

    # Make this a pretty color image (NxMx4)
    depth_color = (plt.cm.cubehelix(depth) * 255).astype(np.uint8)[...,:3]
    cv2.imshow(WINDOW_NAME, depth_color)

def main():
    rospy.init_node("depth_view")
    rospy.Subscriber("image", Image, new_image)

    cv2.namedWindow(WINDOW_NAME)
    cv2.startWindowThread()

    rospy.spin()

if __name__ == '__main__':
    main()
