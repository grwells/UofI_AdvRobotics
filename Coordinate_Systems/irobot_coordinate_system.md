# iRobot Coordinate System
The coordinate system used by the Create3 is defined under the [ROS2 Interface at the bottom of the page](https://iroboteducation.github.io/create3_docs/api/ros2/#ros-2-coordinate-system).
It is right-handed, meaning that the direction the robot is facing after rebooting is defined as positive x, with the positive y axis extending to the left and z up, with the
default units being in meters. Note that the robot will only reboot when placed on the dock.

This is inconvenient since in most use cases we use the docked state to guarantee that our robot starts from a known position and orientation(Pose). If we were to reboot the robot while docked, the
x-axis would extend through the dock and undocking to begin a program would point the robot in the negative x direction. To re-enter the positive x, we would have to first drive the robot
around the dock, a fact which then means re-docking would require driving around the dock again so that the robot's IR sensors could pick up the dock. This is too complicated, and
solving the problem mathematically would also require rotating everything 180 degrees to match the original left-handed coordinate system.

There is a simple solution to this problem, thankfully. Using the `irobot_create_msgs/service/ResetPose` service we can set the pose of the robot to whatever we want. Use the process
below to set the position of your robot at the beginning of your program.
1. Start with your robot docked and turned on.
2. Undock the robot.
3. Call `[robot_id]/reset_pose` and pass a pose like `{position:{x:0.0, y:[robot_offset], z:0.0}, orientation:{x:0,y:0,z:0,w;1}}` where each position value is in meters. 
Remember that your robot should be offset in the y-axis as indicated in the table below. For documentation, refer to 
[iRobot's interface repository](https://github.com/iRobotEducation/irobot_create_msgs/tree/2.1.0). The Reset Pose service is listed at the end of `README.md`.

After reseting your pose, you will be able to use the `irobot_create_msgs/action/NavigateToPosition` action to drive to any pose/goal you specify.

## Robot Starting Coordinates
Reset your robot pose to the coordinates below after undocking. Use the same robot order assigned in class.


| Robot Order | (X, Y) m| 
| :---: | :---: | 
| #1 | (0, 0) | 
| #2 | (0, 0.642) | 
| #3 | (0, 1.284) | 
| #4 | (0, 1.926) | 

## Robot Bounding Box
The box within which the robots are to navigate will be defined by these points: (0,-0.3), (0,2.3), (2, -0.3), (2, 2.3).
