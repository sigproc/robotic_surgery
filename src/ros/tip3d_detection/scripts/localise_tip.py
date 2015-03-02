#!/usr/bin/env python

import rospy

def main():
    # Register the node with ROS
    rospy.init_node('localise_tip', anonymous=True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__=='__main__':
    main()
