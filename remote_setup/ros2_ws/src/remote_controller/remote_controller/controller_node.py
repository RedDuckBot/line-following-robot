import rclpy 
from rclpy.node import Node 
from bot_interfaces.msg import XboxController
from xbox360controller import Xbox360Controller
from remote_controller.controller_data import Controller

class XboxControllerNode(Node):
    """
    Represents a Xbox360 controller node for publishing its inputs

    Attributes:
        controller_events_ (Xbox360Controller): Controller object for events
        controller_ (Controller): stores xbox 360 controller states & values
        controller_publisher_ : the publisher of XboxController messages on 
                                /xbox_controller topic
    """

    def __init__(self):
        super().__init__("controller")
        self.controller_publisher_ = self.create_publisher(XboxController,
                                        "/xbox_controller", 10)

        #Initialize and setup xbox controller for events
        self.controller_events_ = Xbox360Controller(0,axis_threshold=0.0)
        self.controller_events_.button_x.when_pressed = self.on_button_press
        self.controller_events_.button_y.when_pressed = self.on_button_press
        self.controller_events_.button_b.when_pressed = self.on_button_press
        self.controller_events_.button_a.when_pressed = self.on_button_press
        self.controller_events_.button_thumb_l.when_pressed = self.on_button_press
        self.controller_events_.button_thumb_r.when_pressed = self.on_button_press
        self.controller_events_.button_select.when_pressed = self.on_button_press
        self.controller_events_.button_start.when_pressed = self.on_button_press
        self.controller_events_.button_mode.when_pressed = self.on_button_press
        self.controller_events_.axis_r.when_moved = self.on_joy_move 
        self.controller_events_.axis_l.when_moved = self.on_joy_move 
        self.controller_events_.trigger_l.when_moved = self.on_trigger_press 
        self.controller_events_.trigger_r.when_moved = self.on_trigger_press 
        self.controller_events_.button_x.when_released = self.on_button_released
        self.controller_events_.button_y.when_released = self.on_button_released
        self.controller_events_.button_b.when_released = self.on_button_released
        self.controller_events_.button_a.when_released = self.on_button_released
        self.controller_events_.button_thumb_l.when_released = self.on_button_released
        self.controller_events_.button_thumb_r.when_released = self.on_button_released
        self.controller_events_.button_select.when_released = self.on_button_released
        self.controller_events_.button_mode.when_released = self.on_button_released

        self.controller_ = Controller()

        self.get_logger().info("Xbox controller node initialized.")

    def on_joy_move(self, axis):

        if not (axis.y > -0.25 and axis.y < 0.25) or  \
            not (axis.x > -0.25 and axis.x < 0.25):
            if axis.name == "axis_r":
                self.controller_.right_joy_x = axis.x
                self.controller_.right_joy_y = axis.y
            else:
                self.controller_.left_joy_y = axis.y
                self.controller_left_joy_x = axis.x
        else:
            if axis.name == "axis_r":
                self.controller_.right_joy_x = 0.0
                self.controller_.right_joy_y = 0.0
            else:
                self.controller_.left_joy_y = 0.0
                self.controller_.right_joy_x = 0.0
        self.publish_controller_input()
        
    def on_trigger_press(self, trigger):
        if trigger.name == "button_trigger_l":
            self.controller_.left_trigger = trigger.value
        else:
            self.controller.right_trigger = trigger.value

        self.publish_controller_input()

    def on_button_press(self, button):
        self.set_controller_state(button)
        self.publish_controller_input()

    def on_button_released(self, button):
        self.set_controller_state(button)

    def publish_controller_input(self):
        msg = self.create_publisher_msg()
        self.controller_publisher_.publish(msg)

    def set_controller_state(self, button) -> None:
        """
            Sets the button states for self.controller_

            Args:
                button: Conntains the name of button pressed
        """

        match button.name :
            case "button_x":
                self.controller_.x = not self.controller_.x 
                return 
            case "button_y":
                self.controller_.y = not self.controller_.y 
                return
            case "button_b":
                self.controller_.b = not self.controller_.b 
                return
            case "button_a":
                self.controller_.a = not self.controller_.a 
                return
            case "button_thumb_l":
                self.controller_.left_bumper = not self.controller_.left_bumper 
                return
            case "button_thumb_r":
                self.controller_.right_bumper = not self.controller_.right_bumper 
                return
            case "button_select":
                self.controller_.back = not self.controller_.back 
                return
            case "button_start":
                self.controller_.start = not self.controller_.start 
                return
            case "button_mode":
                self.controller_.mode = not self.controller_.mode 
                return
                    
    def create_publisher_msg(self) -> XboxController:
        msg = XboxController()

        msg.x = self.controller_.x
        msg.y = self.controller_.y
        msg.b = self.controller_.b
        msg.left_bumper = self.controller_.left_bumper
        msg.right_bumper = self.controller_.right_bumper
        msg.back = self.controller_.back
        msg.start = self.controller_.start
        msg.mode = self.controller_.mode

        msg.left_joy_x = self.controller_.left_joy_x
        msg.left_joy_y = self.controller_.left_joy_y
        msg.right_joy_x = self.controller_.right_joy_x
        msg.right_joy_y = self.controller_.right_joy_y
        msg.right_trigger = self.controller_.right_trigger 

        return msg

def main():
    rclpy.init()
    controller_node = XboxControllerNode()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()