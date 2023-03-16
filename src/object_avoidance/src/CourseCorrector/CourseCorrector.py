import math
import numpy as np

class CourseCorrector():
    
    def __init__(self, K_l = 2.0, K_a = 5.0):
        self.point = (0.0,0.0,0.0)
        self.heading = (0.0,0.0,0.0)
        self.waypoint = []
        self.angle_margin = 0.01
        self.destination_margin = 2.0
        self.K_l = K_l
        self.K_a = K_a
        self.l_saturation = 10.0

    def getAvoidanceWaypoint(self, obstacle_in_lidar_frame):
        if (len(self.waypoint) > 2):
            self.waypoint.pop() #pop the last obstacle waypoint
        theta = self.heading[2]
        pose = self.point
        transformation_matrix = np.array( [[np.cos(theta), -np.sin(theta), 0, pose[0]], 
                                       [np.sin(theta), np.cos(theta), 0, pose[1]], 
                                       [0, 0, 1, pose[2]], 
                                       [0, 0, 0, 1] ])
        arg = np.atleast_2d([obstacle_in_lidar_frame[0], obstacle_in_lidar_frame[1], 0, 1]).T
        result = np.matmul(transformation_matrix, arg)
        new_obstacle_waypoint = (result[0][0], result[1][0], result[2][0])
        self.giveWaypoint(new_obstacle_waypoint)
        return new_obstacle_waypoint
        

    def givePose(self, point, heading=None):
        self.point = point
        if (heading is not None):
            self.heading = heading

    def giveWaypoint(self, point):
        self.waypoint.append(point)
    
    def getDistanceToWaypoint(self):
        currentWaypoint = self.waypoint[-1]
        error = math.sqrt (math.pow(currentWaypoint[0] - self.point[0], 2)
                          + math.pow(currentWaypoint[1] - self.point[1], 2)
                          + math.pow(currentWaypoint[2] - self.point[2], 2))
        error = error if error < self.l_saturation else self.l_saturation #saturation
        return error
    
    def getHeadingError(self):
        currentWaypoint = self.waypoint[-1]
        desired_heading = math.atan2 (currentWaypoint[1] - self.point[1], 
                                      currentWaypoint[0] - self.point[0])
        error = desired_heading - self.heading[2] #only return error in yaw

        while (error < -math.pi):
            error = error + 2 * math.pi
        while (error > math.pi):
            error = error - 2 * math.pi
        return error
    
    def checkIfArrived(self):
        l_error = self.getDistanceToWaypoint()
        if (l_error < self.destination_margin):
            if(len(self.waypoint) > 1):
                self.waypoint.pop()

    def getCommandVelocity(self):
        linear = (0, 0, 0)
        angular = (0, 0, 0)

        self.checkIfArrived()

        h_error = self.getHeadingError()
        l_error = self.getDistanceToWaypoint()
        if (l_error > self.destination_margin): #car has arrived
            if (abs(h_error) > self.angle_margin):
                angular = (0, 0, self.K_a * h_error)
                linear = (2.0, 0, 0)
            else:
                linear = (self.K_l * l_error, 0, 0)

        return (linear, angular)