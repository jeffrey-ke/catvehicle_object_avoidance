import math

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


    def givePose(self, point, heading=None):
        self.point = point
        if (heading is not None):
            self.heading = heading

    def giveWaypoint(self, point):
        self.waypoint.append(point)
    
    def getDistanceToWaypoint(self):
        currentWaypoint = self.waypoint[-1]
        print()
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