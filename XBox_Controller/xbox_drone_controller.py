import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Vector3

from inputs import get_gamepad
import math
import threading


class XBox_Controller(Node):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)
    def __init__(self, robot_id: str = 'create3_05F8'):
        super().__init__('xbox_controller')
        self._qos = QoSProfile(
                        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                        depth=5,
                        durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                    )

        self._publisher = self.create_publisher(Twist, 
                                                topic=f'/{robot_id}/cmd_vel',
                                                qos_profile=self._qos) 

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

        self.create_timer(0.25, self.read)


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XBox_Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XBox_Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XBox_Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XBox_Controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XBox_Controller.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XBox_Controller.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state


    def read(self): # return the buttons/triggers that you care about in this methode
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        a = self.A
        b = self.X # b=1, x=2
        rb = self.RightBumper
        
        #linear_vel = Vector3(x=float(math.sqrt((math.pow(x,2) + math.pow(y,2)))), y=0.0, z=0.0)
        linear_vel = Vector3(x=float(-y), y=0.0, z=0.0)
        twist = Twist(linear=linear_vel, 
                        angular=Vector3(x=1.0, y=0.0, z=float(-x)))
        self._publisher.publish(twist)
        self.get_logger().info('published twist')

        return [x, y, a, b, rb]



if __name__ == '__main__':
    rclpy.init()
    controller = XBox_Controller(robot_id='create3_05F8')

    rclpy.spin(controller)

    rclpy.shutdown()
