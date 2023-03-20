from src.CourseCorrector.CourseCorrector import CourseCorrector
import math

def testWaypointDistanceCalculator():
    c = CourseCorrector()
    startPose = (0.0,0.0,0.0)
    c.givePose(startPose)
    endpoint = (3.0, 4.0, 5.0)
    c.giveWaypoint(endpoint)

    correctDistance = math.sqrt ( math.pow(endpoint[0] - startPose[0], 2) 
                                  + math.pow(endpoint[1] - startPose[1], 2) 
                                  + math.pow(endpoint[2] - startPose[2], 2) )
    assert (c.getDistanceToWaypoint() == correctDistance)

def testHeadingErrorCalculation():
    c = CourseCorrector()
    startPose = (0.0, 0.0, 0.0)
    startHeading = (0.0, 0.0, 0.0) # Euler angles

    endPoint = (3.0, 4.0, 5.0)
    c.givePose(startPose, startHeading)
    c.giveWaypoint(endPoint)
    correctHeading = math.atan2 (endPoint[1] - startPose[1], endPoint[0] - startPose[0])
    if (correctHeading < -math.pi):
        correctHeading = correctHeading + 2 * math.pi
    elif (correctHeading > math.pi):
        correctHeading = correctHeading - 2 * math.pi
    assert(c.getHeadingError() == correctHeading)

def testVelocityWhenHeadingError():
    c = CourseCorrector()
    endPoint = (3.0, 4.0, 5.0)
    c.giveWaypoint(endPoint)
    #Using default start point and heading

    correct_linear = (2, 0, 0)
    correct_angular = (0, 0, c.K_a * c.getHeadingError())
    assert (c.getCommandVelocity() == (correct_linear, correct_angular))

def testVelocityNoHeadingError():
    c = CourseCorrector()
    endpoint = (5.0, 0.0, 0.0)
    c.giveWaypoint(endpoint)

    correct_linear = (c.K_l * c.getDistanceToWaypoint(), 0, 0)
    correct_angular = (0, 0, 0)
    assert (c.getCommandVelocity() == (correct_linear, correct_angular))

    ninety_degrees = (0, 0, math.pi/2.0)
    endpoint = (0.0, 15.0, 0.0)
    c.givePose(c.point, ninety_degrees)
    c.giveWaypoint(endpoint)

    correct_linear = (c.K_l * c.getDistanceToWaypoint(), 0, 0)
    correct_angular = (0, 0, 0)
    assert (c.getCommandVelocity() == (correct_linear, correct_angular))

def testWaypointTransitions():
    c = CourseCorrector()

    final_destination = (50.0, 50.0, 0.0)
    c.giveWaypoint(final_destination)

    first_stop = (30.0, -30,0, 0.0)
    c.giveWaypoint(first_stop)

    correct_linear = (2, 0, 0)
    correct_angular = (0, 0, c.K_a * c.getHeadingError())
    assert (c.getCommandVelocity() == (correct_linear, correct_angular))


    c.givePose(first_stop) #car has arrived to the first stop; now we expect it to head to the next waypoint

    cmd_vel = c.getCommandVelocity()
    correct_linear = (2, 0, 0)
    correct_angular = (0, 0, c.K_a * c.getHeadingError())
    assert (cmd_vel == (correct_linear, correct_angular))
    assert (c.waypoint[-1] == (50.0, 50.0, 0.0))




    