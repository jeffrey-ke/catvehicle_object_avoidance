# catvehicle_object_avoidance
```
                                                                                                /catvehicle/cmd_vel    
                                                                                                      ^
                                                                                                      |
                                                                                                      |
                                                                                                      |
                                                                                                      |
                                                                                                      |
                                                                                                      |
                                                                                     +----------------+--------------------+
                                                +------------------------------------> CourseCorrectionPublisher ROS script|
                                                |                                    +----------------^--------------------+
                                                |                                                     |
                                                |                                                     |
                                                |                                                     |
                                                |                                                     |
                                                |                                                     |
                                                |                                     /catvehicle/obstacle_velocity_estimation
                                                |                                                     ^
                                                |                                                     |
                                                |                                                     |
/catvehicle/pose          +---------------------+---------------------+        +----------------------+---------------+
      |                   |                                           |        |                                      |
      |                   |    SafetyPerimeterPublisher ROS Script    |        |     HardwareListener ROS script      |
      +-------------------+--------------------+                      |        |                                      |
                          |                    |                      |        |    +---------------------------+     |
                          |  +-----------------v-------------------+  |        |    |                           |     |
                          |  |                                     |  |        |    | HardwareManager.py Class  |     |
                          |  |    SafetyPerimeterGenerator Class   |  |        |    |                           |     |
                          |  |                                     |  |        |    +---^-------------^---------+     |
                          |  +-------------------------------------+  |        |        |             |               |
                          |                                           |        |        |             |               |
                          +-------------------------------------------+        +--------+-------------+---------------+
                                                                                        |             |
                                                                       +----------------+             +----------------+
                                                                       |                                               |
                                                                       |                                               |

                                                      /catvehicle/camera_left/image_raw_left        /catvehicle/camera_right/image_raw_right
```
### Important launch commands:
- roslaunch catvehicle catvehicle_neighborhood.launch

