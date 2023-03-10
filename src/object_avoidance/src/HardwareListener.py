#! /usr/bin/python3

import rospy
from std_msgs.msg import String
from HardwareManager import HardwareManager

def LidarQuery():

    rospy.init_node('hw_manager')

    rospy.Subscriber("/LIDAR", String, )


if __name__ == '__main__':
    LidarQuery()