import numpy as np

headon_obstacle = np.array([27, 2, 0])

class HardwareManager():
    def __init__(self, threshold, angleIncrement=0.017541900277137756, startingAngle=-1.5700000524520874):
        self.threshold = threshold
        self.angleIncrement = angleIncrement
        self.startingAngle = startingAngle
        self.obstaclesArray = []

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

    def getObstacleVectors(self, carPos):
        return [self.obstacleAvoidance(K=2, n=2, objPos=headon_obstacle, carPos=carPos)]

    def obstacleAvoidance(self, K, n, objPos, carPos):
        diffVector = carPos - objPos + 0.000001
        distances = diffVector / np.linalg.norm(diffVector)
        distanceVec = np.array([1/float(distance) for distance in distances])
        forceVector = (K*(distanceVec)**n)

        return forceVector

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
    
