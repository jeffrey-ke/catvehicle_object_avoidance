#! /usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from object_avoidance.msg import RepulsionVector
from object_avoidance.msg import RepulsionVectorArray
from HardwareManager.HardwareManager import HardwareManager

hw_m = HardwareManager(threshold=10)

def feedManager(data: LaserScan):
    global hw_m
    hw_m.angleIncrement = data.angle_increment
    hw_m.startingAngle = data.angle_min
    hw_m.feedLidarRange(data.ranges)

    # obstacleVectors = hw_m.getObstacleVectors()
    # pub.publish(obstacleVectiors)
    pass

def hardwareListener():

    rospy.init_node('hw_listener')

    rospy.Subscriber("/catvehicle/front_laser_points", LaserScan, feedManager)
    # pub = rospy.Publisher('/catvehicle/obstacle_repulsion', RepulsionVectorArray)
    rospy.spin()


if __name__ == '__main__':
    hardwareListener()