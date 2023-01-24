# Simple Action Example
This directory contains a custom ROS2 package which, when run demonstrates using actions to:

1. Undock robot
2. Drive 1 meter

It also demonstrates using the keyboard to call functions which start and stop the robot as it is executing the actions.

## Example Setup
1. `cd` into example directory.
2. Run `colcon build`.
3. Start new terminal session.
4. `cd` into example directory and source setup with `source install/setup.sh`.
5. `cd` into `simple_action_example/src/goal_slasher/goal_slasher`.
6. Run code with `python3 main.py`
7. Use `s` key to start actions and `c` to cancel actions.
