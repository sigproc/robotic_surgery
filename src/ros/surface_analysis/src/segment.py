import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot
import numpy

from sensor_msgs.msg import Image

# Global variables
BRIDGE = CvBridge()
