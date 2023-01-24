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

## Dependencies
* install Pynput library for keyboard input with `pip install pynput`
* _**Git submodules:**_ the irobot_create_msgs package is included here as a submodule. This means that when you pull this repository `src/irobot_create_msgs` may be an empty folder. To fix this run `git submodule update --init --recursive` or simply clone irobot_create_msgs to the `src/` directory with:

        git clone -b humble https://github.com/iRobotEducation/irobot_create_msgs.git
