from dataclasses import dataclass

@dataclass
class Controller:
    """
        Data class stores the states and values of button presses and 
        joystick/trigger for Xbox 360 controller, respectively.
    """

    #States of button presses
    x: bool = False
    y: bool = False
    b: bool = False
    a: bool = False
    left_bumper: bool = False
    right_bumper: bool = False
    back: bool = False #Also called select
    mode: bool = False #Big Center button

    #Values for joysticks and triggers
    left_joy_x: float = 0.0
    left_joy_y: float = 0.0
    right_joy_x: float = 0.0
    right_joy_y: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0
