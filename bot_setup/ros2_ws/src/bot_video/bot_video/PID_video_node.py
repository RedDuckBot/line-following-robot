#!/usr/bin/env python3

import rclpy, socket, pickle, threading, cv2, time
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn, LifecycleNode
import numpy as np
from bot_video.PID import PID
from bot_interfaces.action import MotorsInstruct 
from rclpy.action import ActionClient



class PIDMotorNode(LifecycleNode):
    """
    Represents a PID controller node for motor node for line following task.

    Uses images of black tape on floor to calculate corrective measure.
    The corrective measure is calculated based on far the robot is from the 
    center of line/tape (the center of the tape is know as it's centroid).

    Symbols:
        Fx:        Refers to center x coordinate of frame of floor view image
        Cx:        Refers to x coordinate of centroid of tape

    Attributes:
        host_ip (str):              IP of server for PID controller 
        host_port (int):            Port of server for PID controller 
        motors_client:              Represents an action client for motors node
        video_server_UDPsocket:     Socket for floor view server      
        pid_thread:                 Thread sends motor instructions 
        is_active (bool):           Notify pid_thread when to stop sending
        max_error (float):          Max distance between Fx - Cx 
        Fx (float):                 Defined in symbols 
    """

    def __init__(self):
        super().__init__("pid_node")
        self.declare_parameters(
                namespace='',
                parameters=[
                    ("pid_server_ip", rclpy.Parameter.Type.STRING),
                    ("pid_port", rclpy.Parameter.Type.INTEGER),
                ])
        self.host_ip = self.get_parameter("pid_server_ip").value
        self.host_port = self.get_parameter("pid_port").value

        self.video_server_UDPsocket = None
        self.pid_thread = None
        self.is_active = False
        self.max_error = 0.0 
        self.Fx = 0.0
        self.motors_client = ActionClient(self, MotorsInstruct, 
                                          "/motor_controller_actions")
        
        self.get_logger().info("PID motor controller initialized.")
        self.get_logger().info("Before setting up for configure state make sure robot is placed on line.")

    def on_configure(self, previous_state: LifecycleState):
        """
        While in configuration state, go through setup for line following task.

        Args:
            previous_state (LifecycleState): last state of life cycle ROS2 node
        """

        self.get_logger().info("IN on_configre")

        socket_pass = self.setup_UDP_socket()

        if not socket_pass:
            return TransitionCallbackReturn.FAILURE

        self.setup_path()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState):
        """
        While in activation state, robot performs line following task.

        Args:
            previous_state (LifecycleState): last state of life cycle ROS2 node
        """

        self.get_logger().info("IN on_activate")

        self.is_active = True
        self.pid_thread = threading.Thread(target=self.pid_controller)
        self.pid_thread.start()

        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState):
        """
        While in deactivation state, the robot is aquitted line following task.

        Args:
            previous_state (LifecycleState): last state of life cycle ROS2 node
        """

        self.get_logger().info("IN on_deactivate")
        self.is_active = False
        time.sleep(1.0)
        self.motors_client.send_goal_async(self.create_motors_goal())

        return super().on_deactivate(previous_state)
    
    def on_cleanup(self, previous_state: LifecycleNode):
        """
        While in cleanup state, close socket for video connection.

        Args:
            previous_state (LifecycleState): last state of life cycle ROS2 node
        """

        self.get_logger().info("IN on_cleanup")
        self.video_server_UDPsocket.close()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState):
        """
        "While in shutdown state, close resources, i.e. possibly the 
        pid controller thread and UDP video socket.

        Args:
            previous_state (LifecycleState): last state of life cycle ROS2 node
        """

        state_label = previous_state.label

        self.get_logger().info("IN shutdown")

        if self.is_active:
            self.is_active = False

        if state_label in ["activate","inactive"]:
            self.video_server_UDPsocket.close()

        return TransitionCallbackReturn.SUCCESS  

    def setup_UDP_socket(self) -> bool:
        """
        Setup UDP socket for listening for new images from floor view server.

        Return: bool: true if setup was successful otherwise false
        """

        setup_successful = True

        try:
            self.video_server_UDPsocket = socket.socket(socket.AF_INET,
                                            socket.SOCK_DGRAM)
            self.video_server_UDPsocket.bind((self.host_ip,self.host_port))
        except socket.error as e:
            self.get_logger().info("Failed to setup socket")
            setup_successful = False

        return setup_successful

    def release_resources(self):
        self.video_server_UDPsocket.close()

    def pid_controller(self):
        """
        This function is used in a thread to send motor instructions to motor 
        controller node. Uses a PID controller and images to create the motor
        instructions.
        """

        max_motor_effort = 100.0
        time_interval = 1.0 #seconds
        kp = max_motor_effort / self.max_error
        ki = 0.0
        kd = 0.0
        pid = PID(time_interval,self.Fx,kp,ki,kd)

        while self.is_active: 
            img = self.getImage()
            Cx, _ = self.getCentroid(img)
            pid_output = pid.compute_adjustment(Cx)
            self.sendMotorInstruction(abs(pid_output),pid.getError())
            time.sleep(time_interval)
    
    def sendMotorInstruction(self, pid_output: float, error: float): 
        """
        Create motor instruction and send it to motor controller node.

        Args:
            pid_output (float): Corrective action cal. using Fx-Cx (error)
            error (float):      error from Fx-Cx
        """

        direction = ""
        message = ""
        motors_goal = self.create_motors_goal()

        if error > -25.0 and error < 25.0: #Forward
            direction = "forward"
            motors_goal.forward = True
        elif error > 0: #Turn left; counter-clockwise
            direction = "left"
            motors_goal.counter_clockwise = True
        else: #Turn right; clockwise
            direction = "right"
            motors_goal.clockwise = True

        if motors_goal.forward or pid_output > 100.0: 
            motors_goal.left_motors_effort = 50.0 
            motors_goal.right_motors_effort = 50.0 
        else:
            if pid_output < 85.0:
                pid_output = 85.0 

            motors_goal.left_motors_effort = pid_output
            motors_goal.right_motors_effort = pid_output
        
        message = direction + " L: " + str(motors_goal.left_motors_effort) + " R: " + str(motors_goal.right_motors_effort) 
        self.get_logger().info(message)

        self.motors_client.send_goal_async(motors_goal)

    def create_motors_goal(self):
        """
        Create motors goal for motor controller server.

        Return:
            motor goal object
        """

        motors_goal = MotorsInstruct.Goal() 
        motors_goal.forward = False
        motors_goal.reverse = False
        motors_goal.clockwise = False
        motors_goal.counter_clockwise = False
        motors_goal.idle = False
        motors_goal.left_motors_effort = 0.0
        motors_goal.right_motors_effort = 0.0

        return motors_goal
    
    def getImage(self):
        """
        Retrieve image from video broadcasting server.

        Return: A binary image
        """

        threshold = 60
        max_val = 255
        kernel = np.ones((5,5), np.uint8)

        payload = self.video_server_UDPsocket.recvfrom(1000000)
        data = payload[0]
        data = pickle.loads(data)

        img = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
        _, binary_image = cv2.threshold(img, threshold, max_val, 
                                    cv2.THRESH_BINARY)
        binary_image = cv2.bitwise_not(binary_image)
        binary_image = cv2.dilate(binary_image, kernel,iterations=1)

        return binary_image

    def getCentroid(self, img):
        """
        Get centroid of black tape on floor of         

        Args:
            img: Binary image from robots floor view

        Return: 
            tuple: centroid, i.e. (x,y) cartesian coordinates
        """

        contour = self.getContour(img)
        M = cv2.moments(contour)
        Cx = int(M['m10']/M['m00'])
        Cy = int(M['m01']/M['m00'])

        return (Cx,Cy)

    def setup_path(self):
        """
        As part of on_configure state, the class attributes Fx and max_error
        get calculated before performing line following task.

        Post:
            Class attributes Fx and max_error are set.
        """

        self.get_logger().info("Lets setup the robots line path.")
        self.get_logger().info("Place robot on the center of line path.") 

        img = self.getImage()
        _, width = img.shape
        self.Fx = width//2

        contour = self.getContour(img)
        _, _, contour_width, _ = cv2.boundingRect(contour)

        self.max_error = self.Fx - (contour_width // 2)
        self.get_logger().info(str(self.max_error))

    def getContour(self, img):
        """
        Get contour of tape from binary image.

        Args:
            img: binary image of robots floor view.
        
        Return:
            contour of tape from  
        """

        contours, _ =  cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=cv2.contourArea)

        return contour

def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDMotorNode()
    rclpy.spin(pid_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()