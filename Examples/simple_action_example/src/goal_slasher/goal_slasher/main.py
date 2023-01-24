import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock

from pynput.keyboard import KeyCode
from key_commander import KeyCommander

class Slash(Node):
    """
    Example class for demonstrating goal canceling and different methods of goal execution.
    """

    def __init__(self, namespace):
        super().__init__('slasher')

        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock')
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance')

        self._goal_uuid = None


    def get_goal_id(self, future):
        """
        Save the goal UUID attached to a goal response.
        """
        self.get_logger().warning('retrieved goal UUID')
        self._goal_uuid = future.result()


    def slash_goal(self):
        """
        To slash is to cancel any goal which is currently running.
        If no goal is currently running then a message is printed and nothing 
            is done.
        """

        def reset_uuid(self):
            """
            callback to reset the goal uuid when successful at cancelling the goal
            """
            self._goal_uuid = None

        if self._goal_uuid is not None:
            self.get_logger().warning('requesting goal cancellation')
            self._goal_uuid.cancel_goal()

        else:
            self.get_logger().warning('no goal to cancel at the moment')



    def drive_away(self):
        """
        Undocks the robot using the blocking version of send_goal()
        so that the robot doesn't attempt to drive until the 
        goal is complete.

        Afterward an asynchronous goal is sent which allows
        the goal to be cancelled before it finishes executing.
        """
        self.get_logger().warning('WAITING FOR SERVER')
        # wait until the robot server is found and
        #   ready to receive a new goal
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('UNDOCKING')

        # create new Undock goal object to send to server
        undock_goal = Undock.Goal()

        # send the goal: send_goal() blocks until action
        #   is complete without returning a UUID for the goal.
        #   Thus it is impossible to cancel the goal from this
        #   script.
        self._undock_ac.send_goal(undock_goal)

        # print statement after goal completes since send_goal() blocks
        self.get_logger().warning('UNDOCKED')

        # wait for DriveDistance action server (blocking)
        self._drive_ac.wait_for_server()
        self.get_logger().warning('DRIVING!')

        # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 1.0

        # send goal asynchronously:
        #   when the goal is sent asynchronously we are able
        #   to attach a callback which will capture the goal
        #   unique identifier(UUID) which can then be used to 
        #   cancel the goal's execution prematurely
        goal_response = self._drive_ac.send_goal_async(drive_goal)

        # attach the callback to get goal UUID after it has
        #   been accepted by the server
        goal_response.add_done_callback(self.get_goal_id)
        

        

if __name__ == '__main__':
    rclpy.init()
    
    namespace = 'create3_05B9'
    s = Slash(namespace)
    keycom = KeyCommander([
        (KeyCode(char='c'), s.slash_goal), # attach cancel goal callback
        (KeyCode(char='s'), s.drive_away), # attach start action callback
        ])

    rclpy.spin(s) # execute slash callbacks until shutdown or destroy is called
    rclpy.shutdown()


