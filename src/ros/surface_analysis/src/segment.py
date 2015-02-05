#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot
import numpy

from sensor_msgs.msg import Image

class SegmentationNode(object):
    """docstring for Se"""
    def __init__(self):

        # Initialise member variables
        self.depthImage = None
        self.colourImage = None
        self.irImage = None
        
        pub = rospy.Publisher("segmented/depth", Image)

        BRIDGE = CvBridge()

        # Load subscribe to topics from Kinect
        rospy.Subscriber("depthImage", Image, self.loadDepth)
        rospy.Subscriber("colourImage", Image, self.loadColour)
        rospy.Subscriber("irImage", Image, self.loadIR)


    # Segmentation logic
    def segment():

        

        # Publish the processed image to the next stage of the pipeline
        print "Publishing processed image"    
        if depthImage is not None:
    	   pub.publish(BRIDGE.cv2_to_imgmsg(depthImage))

    # Convert images from kinect to OpenCV images
    # --- Depth ---
    def loadDepth(image):
        print "Loading depth image"
        try:
            depthImage = BRIDGE.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: {0}".format(e.message))

        self.segment()

    # --- Colour ---
    def loadColour(image):
        print "Loading RGB image"
        try:
            self.colourImage = BRIDGE.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: {0}".format(e.message))

    # --- IR ---
    def loadIR(image):
        print "Loading IR image"
        try:
            irImage = BRIDGE.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: {0}".format(e.message))

# Main
def main():
    # Initialise node
    rospy.init_node("segment")
    print "Starting segmentation node"

    rospy.spin()

if __name__ == '__main__':
    main()


