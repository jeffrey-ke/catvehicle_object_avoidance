from src.HardwareManager.HardwareManager import HardwareManager
from src.CourseCorrector.CourseCorrector import CourseCorrector
import numpy as np
import math

def testGenerateWaypoint():
    cc = CourseCorrector()
    hw_m = HardwareManager(threshold=10)

    from data.LaserScanTestData import obstacle_in_front
    obstacle_lidar_frame = hw_m.feedAndReturnEscapepoint(obstacle_in_front)
    pose = (0, 0, 0)
    theta = -math.pi/4
    cc.givePose(pose, (0, 0, theta))


    transformation_matrix = np.array( [[np.cos(theta), -np.sin(theta), 0, pose[0]], 
                                       [np.sin(theta), np.cos(theta), 0, pose[1]], 
                                       [0, 0, 1, pose[2]], 
                                       [0, 0, 0, 1] ])
    print(obstacle_lidar_frame)
    obstacle_in_lidar= np.atleast_2d([obstacle_lidar_frame[0], obstacle_lidar_frame[1], 0, 1]).T
    
    
    avoidance_waypoint = cc.getAvoidanceWaypoint(obstacle_lidar_frame)
    result = np.matmul(transformation_matrix, obstacle_in_lidar)
    print(avoidance_waypoint)
    assert(False)

    assert (avoidance_waypoint == (result[0][0], result[1][0], result[2][0]))

    #remember that after you getAvoidanceWaypoint, you have to pop the last avoidance waypoint
