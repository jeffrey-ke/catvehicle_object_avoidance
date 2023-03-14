import math

class CourseCorrector():
    
    def __init__(self, K_l = 2.0, K_a = 2.0):
        self.point = (0.0,0.0,0.0)
        self.heading = (0.0,0.0,0.0)
        self.waypoint = (0.0,0.0,0.0)
        self.error_margin = 0.01
        self.K_l = K_l
        self.K_a = K_a


    def givePose(self, point, heading=None):
        self.point = point
        if (heading is not None):
            self.heading = heading

    def giveWaypoint(self, point):
        self.waypoint = point
    
    def getDistanceToWaypoint(self):
        return math.sqrt (math.pow(self.waypoint[0] - self.point[0], 2)
                          + math.pow(self.waypoint[1] - self.point[1], 2)
                          + math.pow(self.waypoint[2] - self.point[2], 2))
    
    def getHeadingError(self):
        desired_heading = math.atan2 (self.waypoint[1] - self.point[1], 
                                      self.waypoint[0] - self.point[0])
        error = desired_heading - self.heading[2] #only return error in yaw

        while (error < -math.pi):
            error = error + 2 * math.pi
        while (error > math.pi):
            error = error - 2 * math.pi
        return error
    
    def getCommandVelocity(self):
        linear = (0, 0, 0)
        angular = (0, 0, 0)
        h_error = self.getHeadingError()
        l_error = self.getDistanceToWaypoint()
        if (h_error > self.error_margin):
            angular = (0, 0, self.K_a * h_error)
        elif (l_error > self.error_margin):
            linear = (self.K_l * l_error, 0, 0)

        return (linear, angular)