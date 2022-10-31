# XBox Controller for iRobot Create3
`xbox_drone_controller.py` runs a ROS2 Node that uses input from a generic USB connected controller to drive a Create3 robot.

This is accomplished by publishing to the `cmd_vel/` topic exposed by the ROS2 interface. Although fun to play with, the controller is
limited to driving forward and turning on a very limited radius. 

To run the script, you need to install the inputs library for python. Run `pip install inputs` to install.
