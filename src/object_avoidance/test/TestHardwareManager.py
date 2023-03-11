import numpy as np

from src.HardwareManager import HardwareManager
from sensor_msgs import PointCloud

def testConstructor():
    hw_m = HardwareManager(threshold=10, angleIncrment=0.5)
    assert(hw_m.getSafeDistance() == 10)
    assert(hw_m.getAngleIncrement() == 0.5)

def testLidarObstacleFront():

    hw_m = HardwareManager(threshold=10)
    from LaserScanTestData import obstacle_in_front
    obstacle_in_front = np.array(obstacle_in_front)
    hw_m.feedLidarRange(obstacle_in_front)

    assert(hw_m.numberOfObstaclesDetected() == 28)
    assert(len(hw_m.getObstaclesArray()) == 28)
    assert (np.where(obstacle_in_front < hw_m.getSafeDistance()) == np.array([index[1]/hw_m.getAngleIncrement() for index in hw_m.getObstaclesArray()]))


