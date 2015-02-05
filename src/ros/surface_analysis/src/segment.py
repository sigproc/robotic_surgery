#!/usr/bin/env python

import rospy
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

        # Threshold depth values between an upper and lower limit
        upper = 2000
        lower = 1000
        image = self.threshold(image, lower, upper)
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


