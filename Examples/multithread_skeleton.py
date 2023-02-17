import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg._goal_status import GoalStatus

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock
from irobot_create_msgs.msg import HazardDetectionVector

from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from threading import Lock
from rclpy.executors import MultiThreadedExecutor


# To help with Multithreading
lock = Lock()

class Slash(Node):
    """
    Class to coordinate actions and subscriptions
    """

    def __init__(self, namespace):
        super().__init__('slasher')

        # 2 Seperate Callback Groups for handling the bumper Subscription and Action Clients
        cb_Subscripion = MutuallyExclusiveCallbackGroup()
        #cb_Action = cb_Subscripion
        cb_Action =MutuallyExclusiveCallbackGroup()

        # Subscription to Hazards, the callback function attached only looks for bumper hits
        self.subscription = self.create_subscription(
            HazardDetectionVector, f'/{namespace}/hazard_detection', self.listener_callback, qos_profile_sensor_data,callback_group=cb_Subscripion)

        # Action clients for movements
        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock',callback_group=cb_Action)
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance',callback_group=cb_Action)

        # Variables
        self._goal_uuid = None



    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. Here it parses the message from the Robot and if its
        a 'bump' message, cancel the current action. 

        For this to work, make sure you have:
        ros__parameters:
            reflexes_enabled: false
        in your Application Configuration Parameters File!!!
        '''

        # If it wasn't doing anything, there's nothing to cancel
        if self._goal_uuid is None:
            return

        # msg.detections is an array of HazardDetection from HazardDetectionVectors.
        # Other types can be gotten from HazardDetection.msg
        for detection in msg.detections:
            if detection.type == 1:   #If it is a bump
                self.get_logger().warning('HAZARD DETECTED')
                with lock: # Make this the only thing happening
                    self.get_logger().warning('CANCELING GOAL')           
                    self._goal_uuid.cancel_goal_async()
                    # Loop until the goal status returns canceled
                    while self._goal_uuid.status is not GoalStatus.STATUS_CANCELED:
                        pass    
                    print('Goal canceled.')


#--------------------Async send goal calls-----------------------------
    def sendDriveGoal(self,goal):
        """
        Sends a drive goal asynchronously and 'blocks' until the goal is complete
        """
        
        with lock:
            drive_handle = self._drive_ac.send_goal_async(goal)
            while not drive_handle.done():
                pass # Wait for Action Server to accept goal
            
            # Hold ID in case we need to cancel it
            self._goal_uuid = drive_handle.result() 

        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            pass # Wait until a Status has been assigned

        # After getting goalID, Loop while the goal is currently running
        while self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED:
            if self._goal_uuid.status is GoalStatus.STATUS_CANCELED:
                break # If the goal was canceled, stop looping otherwise loop till finished
            pass
        
        with lock:
            # Reset the goal ID, nothing should be running
            self._goal_uuid = None 

#----------------------------------------------------------------------


    def drive_away(self):
        """
        Undocks robot and drives out a meter asynchronously
        """
        # Freshly started, undock
        self.get_logger().warning('WAITING FOR SERVER')
    # wait until the robot server is found and ready to receive a new goal
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('UNDOCKING')

    # create new Undock goal object to send to server
        undock_goal = Undock.Goal()

        self._undock_ac.send_goal(undock_goal)
        self.get_logger().warning('UNDOCKED')

    # wait for DriveDistance action server (blocking)
        self._drive_ac.wait_for_server()
        self.get_logger().warning('DRIVING!')

    # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 3.0

    # send goal to async function
        self.sendDriveGoal(drive_goal)

        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 3.0

        self.sendDriveGoal(drive_goal)





if __name__ == '__main__':
    rclpy.init()

    namespace = 'create3_05AE'
    s = Slash(namespace)

    # 1 thread for the Subscription, another for the Action Clients
    exec = MultiThreadedExecutor(2)
    exec.add_node(s)

    keycom = KeyCommander([
        (KeyCode(char='r'), s.drive_away),
        ])

    print("r: Start drive_away")
    try:
        exec.spin() # execute slash callbacks until shutdown or destroy is called
    except KeyboardInterrupt:
        print('KeyboardInterrupt, shutting down.')
        print("Shutting down executor")
        exec.shutdown()
        print("Destroying Node")
        s.destroy_node()
        print("Shutting down RCLPY")
        rclpy.try_shutdown()
