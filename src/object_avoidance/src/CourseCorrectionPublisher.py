#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

course_linear = Vector3(0.0, 0.0, 0.0)
course_angular = Vector3(0.0, 0.0, 1.0)

def reportPose(data: Odometry):
    orientation = data.pose.pose.orientation
    print("Quaternion pose: x: {} y: {} z: {} w: {}".format(orientation.x, orientation.y, orientation.z, orientation.w))
    explicit_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    print("Euler pose: {}".format(euler_from_quaternion(explicit_quat)))


def main():
    global course_linear, course_angular
    rospy.init_node('CourseCorrectionPublisher')
    pub = rospy.Publisher('/catvehicle/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/catvehicle/odom', Odometry, reportPose)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        course = Twist(course_linear, course_angular)
        pub.publish(course)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass