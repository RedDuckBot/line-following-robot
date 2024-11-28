#!/usr/binenv python3

import rclpy 
from rclpy.node import Node
from bot_interfaces.action import MotorsInstruct 
from bot_interfaces.msg import XboxController
from rclpy.action import ActionClient


class RobotNodeManager(Node):
    """
    Represents a node manager within  a robot's network. Currently, 
    routes incoming messages for motor controller. 

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

        #Create motors goal
        motors_goal = MotorsInstruct.Goal()
        motors_goal.left_joy_stick_y = msg.left_joy_y
        motors_goal.right_joy_stick_y = msg.right_joy_y
        
        self.motors_client_.send_goal_async(motors_goal)

def main(args=None):
    rclpy.init(args=args)
    manager_node = RobotNodeManager()
    rclpy.spin(manager_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
