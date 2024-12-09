#!/usr/bin/env python3

import rclpy, socket, pickle, threading, cv2
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn, LifecycleNode
import numpy as np


class PIDMotorNode(LifecycleNode):
    """
    Represents a PID controller node for motor node.

    Uses images of black tape on floor to calculate corrective measure.

    Attributes:
        host_ip (str):              IP of server for camera floor view
        host_port (int):            Port of server for camera floor view
        video_server_UDPsocket:     Socket for floor view server      
        pid_thread:                 Thread sends motor instructions 
        is_active (bool):           Notify pid_thread when to stop sending
    """

    def __init__(self):
        super().__init__("PIDMotor")

        self.declare_parameters(
            namespace='',
            parameters=[
                ("host_ip", rclpy.Parameter.Type.STRING),
                ("host_port",rclpy.Parameter.Type.INTEGER)
            ]
        )

        self.host_ip = self.get_parameters("host_ip")
        self.host_port = self.get_parameters("host_port")
        self.video_server_UDPsocket = None
        self.pid_thread = None
        self.is_acitve = False
        
        self.get_logger().info("PID motor controller initialized.")

    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configre")

        socket_pass = self.setup_UDP_socket()

        if not socket_pass:
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")

        self.is_active = True
        self.pid_thread = threading.Thread(target=self.pid_controller)
        self.pid_thread.start()

        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")

        return super().on_deactivate(previous_state)
    
    def on_cleanup(self, previous_state: LifecycleNode):
        self.get_logger().info("IN on_cleanup")

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN shutdown")

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
        controller node. Uses a PID controller and images to cteate the motor
        instructions.
        """

        while self.is_active: 
            img = self.getImage()
            tape_centroid = self.getCentroid(img)

            _, width, _ = img.shape
            x_difference = (width//2) - tape_centroid[0] 

    def getImage(self):
        """
        Retrive image from video broadcasting server.

        Return: A binary image
        """
        threshold = 127
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
        Get centroid of black tape on floor of image.

        Args:
            img: Binary image from robots floor view

        Return: 
            tuple: centroid, i.e. (x,y) cartesian coordinates
        """

        contours, _ =  cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = [contours[0]]

        M = cv2.moments(contours[0])
        Cx = int(M['m10']/M['m00'])
        Cy = int(M['m01']/M['m00'])

        return (Cx,Cy)

