#!/usr/binenv python3

import rclpy 
from rclpy.node import Node
from bot_interfaces.action import MotorsInstruct 
from bot_interfaces.msg import XboxController
from rclpy.action import ActionClient


class RobotNodeManager(Node):
    """
    Represents a node manager for incoming messages/requests outside of the 
    the robot's network . Currently, routes incoming messages from a remote 
    controller (xbox 360 controller) sends motor instuctions to the 
    motor controller node (action server). 

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

        self.get_logger().info("Robot manager started")


    def handle_controller_messages(self, msg: XboxController):
        """
        Handles Xbox360 controller messages.

        Args:
                msg (XboxController): Contains controller inputs.
        """
        
        self.send_motors_goal(msg)

    def send_motors_goal(self, msg: XboxController):
        """
        Send joy-stick inputs to motor controller action server.
        Utility func. for handle_controller_messages.

        Args:
                msg (XboxController): Contains controller inputs.
        """

        #Create motors goal with default values
        motors_goal = MotorsInstruct.Goal()
        motors_goal.steer_right = False
        motors_goal.steer_left = False
        motors_goal.forward = False
        motors_goal.reverse = False
        motors_goal.left_motors_effort = 0.0
        motors_goal.right_motors_effort = 0.0

        self.set_direction_for_motor_goal(motors_goal, msg)
        self.set_motor_efforts_for_motor_goal(motor_goals, msg)

        self.motors_client_.send_goal_async(motors_goal)

    def set_direction_for_motor_goal(self, motors_goal, msg: XboxController):
        """
            Use joy-stick values to determine direction of each set of motors.

            Args:
                msg (XboxController): Contains controller inputs.
        """

        if (msg.left_joy_y < 0.0) and (msg.right_joy_y < 0.0):
            motors_goal.forward = True
        else if (msg.left_joy_y > 0.0) and (msg.right_joy_y > 0.0):
            motors_goal.reverse = True
        else if (msg.left_joy_y < 0.0) and (msg.right_joy_y > 0.0):
            motors_goal.steer_left = True
        else if (msg.left_joy_y > 0.0) and (msg.right_joy_y < 0.0):
            motors_goal.steer_right = True

    def set_motor_efforts_for_motor_goal(self, motors_goals, 
        msg:XboxController):
        """
            Use joy-stick values to determine motor efforts.

            Args:
                msg (XboxController): Contains controller inputs.
        """

            motors_goals.left_motors_effort = abs(msg.left_joy_y * 100)
            motors_goals.right_motors_effort = abs(msg.right_joy_y * 100)


def main(args=None):
    rclpy.init(args=args)
    manager_node = RobotNodeManager()
    rclpy.spin(manager_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
