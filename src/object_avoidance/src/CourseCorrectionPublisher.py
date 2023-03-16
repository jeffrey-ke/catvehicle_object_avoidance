#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from CourseCorrector.CourseCorrector import CourseCorrector
from sensor_msgs.msg import LaserScan
from HardwareManager.HardwareManager import HardwareManager

import math


point = (0.0, 0.0, 0.0)
heading = (0.0, 0.0, 0.0)

cc = CourseCorrector()
hw_m = HardwareManager(threshold=13)
obstacle_in_lidar_frame = None
# def reportPose(data: Odometry):
#     orientation = data.pose.pose.orientation
#     print("Quaternion pose: x: {} y: {} z: {} w: {}".format(orientation.x, orientation.y, orientation.z, orientation.w))
#     explicit_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
#     pitch, roll, yaw = euler_from_quaternion(explicit_quat)
#     print("Euler pose: {}".format(euler_from_quaternion(explicit_quat)))
#     if(yaw > math.pi):
#         course_angular.z = 0.0

def updatePose(data: Odometry):
    global point, heading
    pose = data.pose.pose
    point = (pose.position.x, pose.position.y, pose.position.z)

    explicit_quat = [pose.orientation.x, pose.orientation.y, 
                     pose.orientation.z, pose.orientation.w]
    heading = euler_from_quaternion(explicit_quat)

def feedManager(data: LaserScan):
    global hw_m, obstacle_in_lidar_frame
    hw_m.angleIncrement = data.angle_increment
    hw_m.startingAngle = data.angle_min
    obstacle_in_lidar_frame = hw_m.feedAndReturnEscapepoint(data.ranges)
    print("Feeding hardware manager..")
    


def main():
    global cc, hw_m

    rospy.init_node('CourseCorrectionPublisher')
    pub = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=10)
    rospy.Subscriber('/catvehicle/odom', Odometry, updatePose)
    rospy.Subscriber('/catvehicle/front_laser_points', LaserScan, feedManager)
    rate = rospy.Rate(100)

    
    cc.giveWaypoint((50, 0, 0)) #destination waypoint
    # cc.giveWaypoint((25, -3, 0))
    while not rospy.is_shutdown():
        cc.givePose(point=point, heading=heading)
        if (obstacle_in_lidar_frame is not None):
            print("\t\tDodging!!")
            wp = cc.getAvoidanceWaypoint([obstacle_in_lidar_frame[0], obstacle_in_lidar_frame[1], 0, 1])
            print()
            print("Obstacle in lidar frame: ", obstacle_in_lidar_frame)
            print("corresponding waypoint: ", wp)
        linear, angular = cc.getCommandVelocity()
        twist_linear = Vector3(linear[0], linear[1], linear[2])
        twist_angular = Vector3(angular[0], angular[1], angular[2])
        course = Twist(twist_linear, twist_angular)
        pub.publish(course)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass