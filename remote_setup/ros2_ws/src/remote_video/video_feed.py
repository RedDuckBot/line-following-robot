#!/usr/bin/env python3

import cv2, socket, pickle, sys
import numpy as np

"""Program acts as a UDP server for viewing Stroam's video camera."""

remote_host_ip = ""
port = 6670

def getBinaryImage(img):
    threshold = 60 #Adjust this variable to help make tape line more noticeable
    max_val = 255
    kernel = np.ones((5,5), np.uint8)

    _, binary_image = cv2.threshold(img, threshold, max_val, 
                        cv2.THRESH_BINARY)
    binary_image = cv2.bitwise_not(binary_image)
    binary_image = cv2.dilate(binary_image, kernel,iterations=1)

    return binary_image

def main():
    video_server_UDPsocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    video_server_UDPsocket.bind((remote_host_ip,port))

    print("Video Server ready for bot's floor view")
    while True:
        payload = video_server_UDPsocket.recvfrom(1000000)
        data = payload[0]

        data = pickle.loads(data)

        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        binary_image = getBinaryImage(gray_image)

        cv2.imshow("Floor View", binary_image)

        if cv2.waitKey(5) & 0xFF == 113:
            break

    cv2.destroyAllWindows()
    video_server_UDPsocket.close()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        remote_host_ip = sys.argv[1]
        main()
    else:
        print("No remote ip arg passed")
