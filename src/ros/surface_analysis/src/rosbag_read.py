#!/usr/bin/env python

# Node to read Kinect data from a file and publish it to a topic
import rospy
import rosbag
import yaml
import os.path
from sensor_msgs.msg import Image

def main():
  # Initialise node
  rospy.init_node("rosbag_read")

  # Open the rosbag
  fname = rospy.get_param("~filename")
  print "Opening bagfile data from location " + fname
  bag = rosbag.Bag(fname, 'r')

  # Continuously publish data once per second 
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    for topic, msg, t in bag.read_messages():
      print "Republishing topic '" + topic + "'"
      pub = rospy.Publisher(topic, Image, 1)
      pub.publish(msg)
      rate.sleep()

  # Create publishers
  #depthPub = rospy.Publisher('kinectDepth', Image, 1)
  #colourPub = rospy.Publisher('kinectColour', Image, 1)
  #irPub = rospy.Publisher('kinectIR', Image, 1)

  # Publish data from viewbag
  #depthPub.publish(bag.read_messages('kinectDepth'))
  #colourPub.publish(bag.read_messages('kinectColour'))
  #irPub.publish(bag.read_messages('kinectIR'))

  #rospy.spinOnce()

if __name__ == '__main__':
    main()