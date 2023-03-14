import numpy as np

from src.HardwareManager.HardwareManager import HardwareManager

def testConstructor():
    hw_m = HardwareManager(threshold=10, angleIncrement=0.5)
    assert(hw_m.getSafeDistance() == 10)
    assert(hw_m.getAngleIncrement() == 0.5)

def testLidarObstacleFront():

    hw_m = HardwareManager(threshold=10)
    from data.LaserScanTestData import obstacle_in_front
    obstacle_in_front = np.array(obstacle_in_front)
    hw_m.feedLidarRange(obstacle_in_front)

    assert(hw_m.numberOfObstaclesDetected() == 28)
    assert(len(hw_m.getObstaclesArray()) == 28)
    print(np.where(obstacle_in_front < hw_m.getSafeDistance())[0])
    # print(hw_m.getObstaclesArray())
    print(np.array([tup[2] for tup in hw_m.getObstaclesArray()]))
    assert(np.all(np.where(obstacle_in_front < hw_m.getSafeDistance())[0] == np.array([tup[2] for tup in hw_m.getObstaclesArray()])))

    # This is a test of the functionality where every time the hw_m is fed LiDAR ranges, it creates
    # an array of tuples of the format (range, angle), where range is the distance detected that's
    # beneath the threshold, and angle is at the angle where that range was detected. 

 