#! /usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from object_avoidance.msg import ObstacleArray, Obstacle
from HardwareManager.HardwareManager import HardwareManager
from nav_msgs.msg import Odometry
import numpy as np

hw_m = HardwareManager(threshold=10)
carPos = [0,0,0]

def feedManager(data: LaserScan):
    global hw_m, obstacleVectors
    hw_m.angleIncrement = data.angle_increment
    hw_m.startingAngle = data.angle_min
    hw_m.feedLidarRange(data.ranges)
    pass

# def updateCarPos(data: Odometry):
#     global carPos
#     pose = data.pose.pose.position
#     carPos = [pose.x, pose.y, pose.z]
#     pass

def main():

    rospy.init_node('hw_listener')
    rospy.Subscriber("/catvehicle/front_laser_points", LaserScan, feedManager)
    # rospy.Subscriber("catvehicle/odom", Odometry, updateCarPos)
    pub = rospy.Publisher("/catvehicle/obstacles", ObstacleArray, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = ObstacleArray()
        msg.count = hw_m.numberOfObstaclesDetected()
        msg.obstacles = [Obstacle(_range, angle) for _range, angle, index in hw_m.getObstaclesArray()]
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass