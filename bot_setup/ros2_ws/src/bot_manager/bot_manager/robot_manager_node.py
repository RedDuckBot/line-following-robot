#!/usr/binenv python3

import rclpy 
from rclpy.node import Node
from bot_interfaces.action import MotorsInstruct 
from bot_interfaces.msg import XboxController
from rclpy.action import ActionClient
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from enum import Enum

#Lifecycle node transition states
TRANSITION_STATES = {
    0: "TRANSITION_CREATE",
    1: "TRANSITION_CONFIGURE",
    2: "TRANSITION_CLEANUP",
    3: "TRANSITION_ACTIVATE",
    4: "TRANSITION_DEACTIVATE",
    5: "TRANSITION_UNCONFIGURED_SHUTDOWN",
    6: "TRANSITION_INACTIVE_SHUTDOWN",
    7: "TRANSITION_ACTIVE_SHUTDOWN",
    8: "TRANSITION_DESTROY"
}

class LifecycleState(Enum):
    UNKOWN = 0    
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4


class RobotNodeManager(Node):
    """
    Represents a node manager for incoming messages/requests outside of the 
    the robot's network . Currently, routes incoming messages from a remote 
    controller (xbox 360 controller) and sends motor instuctions to the 
    motor controller node (action server) or cancels line following task,
    depending on which mode the robots in, which is manual or line following. 

    Modes:
        back button press  ---> manual mode
        start button press ---> line following mode

    Attributes:
        motors_client_: Represents an action client for motor controller node
        controller_client_: Represensts a subscriber for Xbox360 contr inputs
    """

    def __init__(self):

        super().__init__("robot_manager")
        
        motors_topic_name = "/motor_controller_actions"

        self.motors_client_ = ActionClient(self, MotorsInstruct,
                 motors_topic_name)
        self.controller_client_ = self.create_subscription(XboxController,
                "/xbox_controller", self.handle_controller_messages, 10)

        #Create command line interfaces for PID node
        self.cli_change_state = self.create_client(ChangeState, 
                                    '/pid_node/change_state')
        self.cli_get_state = self.create_client(GetState, 
                                '/pid_node/get_state')

        while not self.cli_get_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pid node state service...')

        self.manual_mode = True 
        self.is_line_following = False
        self.pid_node_state = None 

        self.get_logger().info("Robot manager started")

    def handle_controller_messages(self, msg: XboxController):
        """
        Handles Xbox360 controller messages.

        Args:
                msg (XboxController): Contains controller inputs.
        """

        if msg.start: 
            self.handle_line_following_mode(msg)

        if not msg.start: 
            self.handle_manual_mode(msg)

    def handle_line_following_mode(self, msg: XboxController):
        """
        Handle the start button press by notifying PID node to perform line
        following mode.

        Args:
                msg (XboxController): Contains controller inputs.
        """

        if self.is_line_following:
            self.get_logger().info("Line following mode already active.")
        else:
            self.is_line_following = True
            self.manual_mode = False
            self.get_logger().info("Activating line following mode") 
            state = self.get_current_state()
            if state != None: self.help_line_mode(state.id)

    def help_line_mode(self, state: int):
        """
        Helper function for handle_line_following_mode.
        """

        if state == LifecycleState.INACTIVE.value:
            self.change_state(3)
        else:
            self.get_logger().info("PID motor node is not in active state.")
            self.get_logger().info("Before line following mode can start, set the node's state to active from cmd.")

    def handle_manual_mode(self, msg):
        """
        Handle joy-stick inputs when manual mode is enabled. 

        Args:
                msg (XboxController): Contains controller inputs.
        """

        if self.manual_mode == False: 
            self.is_line_following = False
            self.manual_mode = True
            self.get_logger().info("Switched to manual mode.")
            #self.change_state(4)
            #Set joy values to zero to hit the 'brakes' on the robot
            msg.left_joy_y = 0.0
            msg.right_joy_y = 0.0

        self.send_motors_goal(msg)

    def change_state(self, transition_id: int):
        """
        Set the state of lifecycle, PID motor node.

        Args:
            transition_id: an integer associated with a lifecycle state
        """

        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.cli_change_state.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(
                f'Successfully changed state: {TRANSITION_STATES[transition_id]}')
        else:
            self.get_logger().error('Failed to change state')

    def get_current_state(self):
        """
        Get the state of lifecycle, PID motor node and return it's state.
        """

        req = GetState.Request()
        future = self.cli_get_state.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            state = future.result().current_state.label
            self.get_logger().info(f'Current state of PID motor node: {state}')
            return state
        else:
            self.get_logger().error('Failed to get current state')
            return None

    def send_motors_goal(self, msg: XboxController):
        """
        Send joy-stick inputs to motor controller action server.
        Utility func. for handle_controller_messages.

        Args:
                msg (XboxController): Contains controller inputs.
        """

        #Create motors goal with default values
        motors_goal = MotorsInstruct.Goal()
        motors_goal.clockwise = False
        motors_goal.counter_clockwise = False
        motors_goal.forward = False
        motors_goal.reverse = False
        motors_goal.idle = False

        self.set_direction_for_motor_goal(motors_goal, msg)
        self.set_motor_efforts_for_motor_goal(motors_goal, msg)

        self.motors_client_.send_goal_async(motors_goal)

    def set_direction_for_motor_goal(self, motors_goal, msg: XboxController):
        """
            Use joy-stick values to determine direction of each set of motors.

            Args:
                msg (XboxController): Contains controller inputs.
        """

        if (msg.left_joy_y < 0.0 and msg.right_joy_y < 0.0): 
            motors_goal.forward = True
        elif (msg.left_joy_y > 0.0 and msg.right_joy_y > 0.0): 
            motors_goal.reverse = True
        elif (msg.left_joy_y < 0.0 and msg.right_joy_y > 0.0):
            motors_goal.counter_clockwise = True
        elif (msg.left_joy_y > 0.0 and msg.right_joy_y < 0.0): 
            motors_goal.clockwise = True
        else:
            motors_goal.idle = True

    def set_motor_efforts_for_motor_goal(self, motors_goal, 
        msg:XboxController):
        """
            Use joy-stick values to determine motor efforts.

            Args:
                msg (XboxController): Contains controller inputs.
        """
        if motors_goal.idle:
            motors_goal.left_motors_effort = 0.0
            motors_goal.right_motors_effort = 0.0
        else:
            motors_goal.left_motors_effort = abs(msg.left_joy_y * 100)
            motors_goal.right_motors_effort = abs(msg.right_joy_y * 100)


def main(args=None):
    rclpy.init(args=args)
    manager_node = RobotNodeManager()
    rclpy.spin(manager_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()