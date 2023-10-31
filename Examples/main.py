#-------------------------------------------
#
# @file     main.py
# @author   Jordan Reed
# @date     October, 2023
# @brief    Program to test out the ROS2 driver designed by James Lasso. 
#           Robot picks up 2 dice, moves them to a conveyor, then moves them back home.
#
# ------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import sys
sys.path.append('/home/jordanreed/Documents/Robots/robotics/robots2/workspace/src/')

import fanuc_ros2_interfaces
from fanuc_ros2_interfaces.action import ShunkGripper, WriteJointOffset, WriteJointPose, WriteJointPosition

from action_msgs.msg import GoalStatus

class MyNode(Node):
    """ A class that will create a single node, and reuse a single action client, to move a robot.
    """
    def __init__(self):
        """Initializes MyNode class Has a single action client

        :param str namespace: what the namespace of robot is
        """
        # call superclass init
        super().__init__('ros2_test')

        self.result = None

        self._action_client = None

    def send_joint_pose_goal(self, joints:list):
        """ Function to pass a list of joints into Goal object and send to action server.

        :param list joints: array of floats for joint positions
        """
        if len(joints) != 6:
            print(f'6 joint positions needed. {len(joints)} passed.')
            exit(1)
        
        goal = WriteJointPose.Goal()
        goal.joint1 = float(joints[0])
        goal.joint2 = float(joints[1])
        goal.joint3 = float(joints[2])
        goal.joint4 = float(joints[3])
        goal.joint5 = float(joints[4])
        goal.joint6 = float(joints[5])
        self.send_goal(WriteJointPose, "WriteJointPose", goal)

    def send_goal(self, action_type, action_name:str, goal):
        """Sets the action client and sends the goal to the robot. Spins node until result callback is received.

        :param action_type: Action type of the action to take
        :param str action_name: name of action to append to namespace command
        :param goal: goal object with necessary parameters to complete goal
        """
        self.get_logger().info(f"Sending goal for '{action_name}'")
        # create/reuse action client with new goal info
        self._action_client = ActionClient(self, action_type, f'/{action_name}')

        # wait for server
        self.get_logger().warning("Waiting for server...")
        self._action_client.wait_for_server()

        # server available
        self.get_logger().warning("Server available. Sending goal now...")

        # send goal
        self.send_goal_future = self._action_client.send_goal_async(goal) #, self.feedback_callback)

        # add done callback for response
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # spin node until done
        self.result = None
        while self.result == None:
            rclpy.spin_once(self)

        # goal done
        self.get_logger().warning(f"{action_name} action done")
    
    def feedback_callback(self, feedback):
        """Callback when getting feedback. Will print feedback

        :param _type_ feedback: _description_
        """
        self.get_logger().info(f'received feedback: {feedback}')
    
    def goal_response_callback(self, future):
        """Callback when a response has been received. Calls get result callback.

        :param _type_ future: _description_
        """
        goal_handle = future.result()
        # print(f'goal handle: {goal_handle}')
        if not goal_handle.accepted:
            self.get_logger().error("GOAL REJECTED")
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Prints result of goal. Sets self.result variable in order to stop spinning node.

        :param _type_ future: _description_
        """
        self.result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal Succeeded! Result info hidden.")
        else:
            self.get_logger().error(f"Goal Failed with status: {status}")
        
def main():
    rclpy.init()
    node = MyNode()

    # positions
    robot_home = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    die1_home_hover = [8.1, 21.0, -42.3, -4.6, -47.3, 24.3]
    die1_home_grab = [8.1, 26.1, -49.6, -5.2, -40.2, 25.2]
    die1_conveyor_hover = [76.8, 12.6, -15.6, -2.9, -74.8, 45.3]
    die1_conveyor_grab = [76.8, 16.5, -28.4, -3.2, -62.1, 46.1]

    die2_home_hover = [29.1, 29.3, 1.7, -3.3, -91.7, 1.9]
    die2_home_grab = [29.1, 39.9, -35.7, -4.1, -54.4, 4.5]
    die2_conveyor_hover = [55.9, 27.3, -8.4, -2.7, -83.4, 64.1]
    die2_conveyor_grab = [55.9, 30.1, -20.4, -2.8, -71.4, 64.7]


    # attempt to make while loop better
    try:
        while True:
            # ------
            # reset robot to home position
            # ------

            node.send_joint_pose_goal(robot_home)   # go back to home position

            cur_goal = ShunkGripper.Goal()  # make sure gripper is open
            cur_goal.command = 'open'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)

            # --------
            # grab and move die one to conveyor
            # --------

            node.send_joint_pose_goal(die1_home_hover)  # hover over die one
            node.send_joint_pose_goal(die1_home_grab)   # move to grab die
            
            cur_goal = ShunkGripper.Goal()  # grab die
            cur_goal.command = 'close'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)

            node.send_joint_pose_goal(die1_home_hover)      # hover over die 1 home (to make sure robot doesn't run into anything)
            node.send_joint_pose_goal(die1_conveyor_hover)  # hover over conveyor 1
            node.send_joint_pose_goal(die1_conveyor_grab)   # place die 1

            cur_goal = ShunkGripper.Goal()  # let go of die
            cur_goal.command = 'open'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)
            
            node.send_joint_pose_goal(die1_conveyor_hover)  # hover over conveyor 1 (don't run into anything)

            # ------
            # grab and move die 2 to conveyor
            # ------

            node.send_joint_pose_goal(die2_home_hover)  # hover over die 2
            node.send_joint_pose_goal(die2_home_grab)   # move to grab die 2

            cur_goal = ShunkGripper.Goal()  # grab die
            cur_goal.command = 'close'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)

            node.send_joint_pose_goal(die2_home_hover)      # hover over die 2
            node.send_joint_pose_goal(die2_conveyor_hover)  # hover over conveyor 2
            node.send_joint_pose_goal(die2_conveyor_grab)   # place die 2

            cur_goal = ShunkGripper.Goal()  # let go of die
            cur_goal.command = 'open'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)

            node.send_joint_pose_goal(die2_conveyor_hover)  # hover over conveyor 2 (avoid collisions)

            # ---------
            # move die 1 back to home
            # ---------

            node.send_joint_pose_goal(die1_conveyor_hover)  # hover over conveyor 1
            node.send_joint_pose_goal(die1_conveyor_grab)   # move to grab die 1

            cur_goal = ShunkGripper.Goal()  # grab die
            cur_goal.command = 'close'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)
            
            node.send_joint_pose_goal(die1_conveyor_hover)  # hover over conveyor 1 (avoid collisions)
            node.send_joint_pose_goal(die1_home_hover)      # hover over die one home
            node.send_joint_pose_goal(die1_home_grab)       # move to grab die
            
            cur_goal = ShunkGripper.Goal()  # let go die
            cur_goal.command = 'open'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)

            node.send_joint_pose_goal(die1_home_hover)      # hover over die one home (avoid collisions)

            # ---------
            # move die 2 back home
            # ---------

            node.send_joint_pose_goal(die2_conveyor_hover)  # hover over conveyor 2
            node.send_joint_pose_goal(die2_conveyor_grab)   # move to grab die 2

            cur_goal = ShunkGripper.Goal()  # grab die
            cur_goal.command = 'close'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)
            
            node.send_joint_pose_goal(die2_conveyor_hover)  # hover over conveyor 2 (avoid collisions)
            node.send_joint_pose_goal(die2_home_hover)      # hover over die 2 home
            node.send_joint_pose_goal(die2_home_grab)       # move to grab die
            
            cur_goal = ShunkGripper.Goal()  # let go die
            cur_goal.command = 'open'
            node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)

            node.send_joint_pose_goal(die2_home_hover)      # hover over die 2 home (avoid collisions)
    except KeyboardInterrupt:
        # ------
        # reset robot
        # ------

        print('KeyboardInterrupt, sending robot home and shutting down.')

        node.send_joint_pose_goal(robot_home)   # go back to home position

        cur_goal = ShunkGripper.Goal()  # make sure gripper is open
        cur_goal.command = 'open'
        node.send_goal(ShunkGripper, 'ShunkGripper', cur_goal)

        # shut down node
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    print('--PYTHON SCRIPT--')
    main()