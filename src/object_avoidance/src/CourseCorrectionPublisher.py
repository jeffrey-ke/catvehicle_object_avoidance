#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from object_avoidance.msg import RepulsionVectorArray

course_linear = Vector3(5.0, 5.0, 0.0)
course_angular = Vector3(0.0, 0.0, 0.0)
def correctVelocity(data):
    global course_linear, course_angular
    repulsion_forces = data.vectors
    for force in repulsion_forces:
        course_linear.x = course_linear.x - force.x
        course_linear.y = course_linear.y - force.y
        course_linear.z = course_linear.z - force.z
    

def main():
    global course_linear, course_angular
    rospy.init_node('CourseCorrectionPublisher')
    pub = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=10)
    rospy.Subscriber('/catvehicle/obstacle_repulsion', RepulsionVectorArray, correctVelocity)
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