#!/usr/binenv python3

import rclpy 
from rclpy.node import Node
from bot_interfaces.action import MotorsInstruct 
from bot_interfaces.msg import XboxController
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import Transition
from rclpy.lifecycle import LifecycleNodeInterface


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
        self.lifecycle_node_client = self.create_client(LifecycleNodeInterface,
                                        "/PIDMotor")


        self.manual_mode = True 
        self.is_line_following = False

        self.get_logger().info("Robot manager started")


    def handle_controller_messages(self, msg: XboxController):
        """
        Handles Xbox360 controller messages.

        Args:
                msg (XboxController): Contains controller inputs.
        """
        if msg.start == True and self.is_line_following == False: 
            self.manual_mode = False
            self.is_line_following = True
            self.handle_line_following_mode()
        else:
            self.get_logger().info("Line following mode already enabled.")

        if msg.back == True and self.is_line_following:
            self.manual_mode = True
            self.is_line_following = False
            self.handle_line_following_mode()
        else:
            self.get_logger().info("Manual mode is already enabled.")

        if self.manual_mode:
            self.send_motors_goal(msg)

    def handle_line_following(self):
        self.get_logger().info()

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