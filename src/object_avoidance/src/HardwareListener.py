#! /usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from object_avoidance.msg import RepulsionVectorArray
from HardwareManager.HardwareManager import HardwareManager
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Vector3

hw_m = HardwareManager(threshold=10)
obstacleVectors = []
carPos = [0,0,0]
def feedManager(data: LaserScan):
    global hw_m, obstacleVectors
    hw_m.angleIncrement = data.angle_increment
    hw_m.startingAngle = data.angle_min
    hw_m.feedLidarRange(data.ranges)

    obstacleVectors = hw_m.getObstacleVectors(np.array(carPos))
    pass

def updateCarPos(data: Odometry):
    global carPos
    pose = data.pose.pose.position
    carPos = [pose.x, pose.y, pose.z]
    pass
def hardwareListener():

    rospy.init_node('hw_listener')
    pub = rospy.Publisher('/catvehicle/obstacle_repulsion', RepulsionVectorArray)
    rospy.Subscriber("/catvehicle/front_laser_points", LaserScan, feedManager)
    rospy.Subscriber("catvehicle/odom", Odometry, updateCarPos)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        arr = RepulsionVectorArray()
        vecs = Vector3()
        arr.vectors = []
        if (len (obstacleVectors) > 0):
            vecs.x = obstacleVectors[0][0]
            vecs.y = obstacleVectors[0][1]
            vecs.z = obstacleVectors[0][2]
            arr.vectors = [vecs]
        arr.obstacle_count = len(obstacleVectors)
        pub.publish(arr)
        rate.sleep()


if __name__ == '__main__':
    try: 
        hardwareListener()
    except rospy.ROSInterruptException:
        pass