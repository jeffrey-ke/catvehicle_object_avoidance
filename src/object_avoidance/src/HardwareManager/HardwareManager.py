import numpy as np
import math

headon_obstacle = np.array([27, 2, 0])

class HardwareManager():
    def __init__(self, threshold, angleIncrement=0.017541900277137756, startingAngle=-1.5700000524520874):
        self.threshold = threshold
        self.angleIncrement = angleIncrement
        self.startingAngle = startingAngle
        self.obstaclesArray = []
        self.angle_margin_offset = math.pi/4

    def feedLidarRange(self, lidar_ranges):
        angle = self.startingAngle
        index = 0
        for _range in lidar_ranges:
            
            if _range < self.threshold:
                self.obstaclesArray.append((_range, angle, index))
            angle += self.angleIncrement
            index = index + 1
        # self.obstaclesArray = np.array([(_range, self.startingAngle + i * self.angleIncrement) 
        #                                 for _range, i in enumerate(lidar_ranges)  if _range < self.threshold])
    def feedAndReturnEscapepoint(self, lidar_ranges):
        angle = self.startingAngle
        obstacle_found = False
        index = 0
        for _range in lidar_ranges:
            if _range < self.threshold:
                obstacle_found = True
                break
            index = index + 1

        if (obstacle_found is True):
            angle = self.startingAngle + index * self.angleIncrement #radians, the angle at the index
            
            lidar_ranges = lidar_ranges[index:-1] # start at the index you first found the object
            for _range in lidar_ranges:
                if _range >= self.threshold * 2:
                    _range = _range if _range != float('inf') else self.threshold * 2
                    print('Angle: ', angle)
                    return (_range * np.cos(angle + self.angle_margin_offset), _range * np.sin(angle + self.angle_margin_offset))
                angle = angle + self.angleIncrement
        else:
            return None

    def getStartingAngle(self):
        return self.startingAngle

    def numberOfObstaclesDetected(self):
        return len(self.obstaclesArray)
    
    def getObstaclesArray(self):
        return self.obstaclesArray

    def getSafeDistance(self):
        return self.threshold
    
    def getAngleIncrement(self):
        return self.angleIncrement
    
