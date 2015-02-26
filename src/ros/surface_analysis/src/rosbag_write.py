#!/usr/bin/env python

# Node to receive data from the Kinect and write it to a file
# so that it can be used offline
import rospy
import rosbag
import functools

from sensor_msgs.msg import Image

def main():
  # Initialise node
  rospy.init_node("rosbag_write")
  print "Starting node 'rosbag_write'"

  # Open the rosbag for writing
  fname = rospy.get_param("~filename")

  # Create a writer object
  writer = KinectWriter(fname)

  # Subscribe to relevant topics
  rospy.Subscriber("kinectDepth", Image, functools.partial(writer.newMessage, 'kinectDepth'))
  rospy.Subscriber("kinectColour", Image, functools.partial(writer.newMessage,'kinectColour'))
  rospy.Subscriber("kinectIR", Image, functools.partial(writer.newMessage, 'kinectIR'))

  rospy.spin()

class KinectWriter(object):

  def __init__(self, filename):
      self._bag = rosbag.Bag(filename, 'w')
      self._fname = filename

  def newMessage(self, topic, newmsg):
    # Write the recieved message to the rosbag
    print "Message recieved from topic: " + topic
    try:
      self._bag.write(topic, newmsg)
      print "Writing data to " + self._fname
    except Exception as e:
      print "Data could not be written to " + self._fname + ":\n" + e

  def __del__(self):
    # Close the rosbag when the node ends
    self._bag.close()

if __name__ == '__main__':
    main()