#!/usr/bin/env python

import rospy
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot
import numpy

from sensor_msgs.msg import Image

class SegmentationNode(object):
    def __init__(self):

        # Initialise member variables
        self.depthImage = None
        self.colourImage = None
        self.irImage = None
        
        self.pub = rospy.Publisher("segmented/depth", Image)

        self.BRIDGE = CvBridge()

        # Load subscribe to topics from Kinect
        rospy.Subscriber("depthImage", Image, self.loadDepth)
        rospy.Subscriber("colourImage", Image, self.loadColour)
        rospy.Subscriber("irImage", Image, self.loadIR)


    # Segmentation logic
    def segment(self, image):

        # Set the target depth and range
        target = 1500.0
        rng = 500.0

        # Crop the image
        image = self.crop(image, target)

        # Threshold depth values between an upper and lower limit
        image = self.threshold(image, target-rng, target+rng)

        return image
        
    def crop(self, image, target):
        # Constants representing the field of view of the Kinect
        MINDEPTH = 400.0
        MAXDEPTH = 4000.0
        HFOV = 70.6 * math.pi/180   # Horizontal field of view in radians
        VFOV = 60 * math.pi/180     # Vertical field of view in radians

        # Cropped dimensions
        width,height = image.shape[:2]
        print "Width = {} Height = {}".format(width,height)
        cw = target / MAXDEPTH * width
        ch = target / MAXDEPTH * height

        left = (width - cw) / 2
        top = (height - ch) / 2
        right = (width + cw) / 2
        bottom = (height + ch) / 2 
        print "Target {} HFOV {} tan {} cw {}".format(target, HFOV, math.tan(HFOV), cw)
        image = image[left:right, top:bottom]

        return image


    def threshold(self, image, lower, upper):
        mask = numpy.logical_and(image > lower,image < upper)
        print "Max = {} Min = {} Mean = {}".format(image.max(), image.min(), image.mean()) 
        print "Mask: upper = {} lower = {}".format((image < upper).sum(),(image > lower).sum())
        return image * mask

    # Callbacks to convert images from kinect to OpenCV images
    # --- Depth ---
    def loadDepth(self, image):
        print "Loading depth image"
        try:
            self.depthImage = self.BRIDGE.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: {0}".format(e.message))

        self.depthImage = self.segment(self.depthImage)

        # Publish the processed image to the next stage of the pipeline
        print "Publishing processed image"    
        if self.depthImage is not None:
            self.pub.publish(self.BRIDGE.cv2_to_imgmsg(self.depthImage))



    # --- Colour ---
    def loadColour(self, image):
        print "Loading RGB image"
        try:
            self.colourImage = self.BRIDGE.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: {0}".format(e.message))

    # --- IR ---
    def loadIR(self, image):
        print "Loading IR image"
        try:
            irImage = self.BRIDGE.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: {0}".format(e.message))

# Main
def main():
    # Initialise node
    rospy.init_node("segment")
    print "Starting segmentation node"

    # Create segmentation object
    segNode = SegmentationNode()

    rospy.spin()

if __name__ == '__main__':
    main()


