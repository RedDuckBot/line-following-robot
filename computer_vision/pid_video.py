import numpy as np
import cv2
from PID import PID

color = (0, 255, 0)  # Green color in BGR
radius = 5  
thickness = -1  # Thickness: -1 fills the circle

image_height = 480
image_width = 640

def getBinaryImage(img):
    threshold = 127
    max_val = 255
    kernel = np.ones((5,5), np.uint8)

    _, binary_image = cv2.threshold(img, threshold, max_val, 
                                cv2.THRESH_BINARY)
    binary_image = cv2.bitwise_not(binary_image)
    binary_image = cv2.dilate(binary_image, kernel,iterations=1)

    return binary_image

def getCentroid(img):
    contours, _ =  cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contour = contours[0]
    M = cv2.moments(contour)
    Cx = int(M['m10']/M['m00'])
    Cy = int(M['m01']/M['m00'])
    _, _, width, _ = cv2.boundingRect(contour)

    return (Cx,Cy, width)

def sendMotorInstruction(pid_output: float, error: float):
    left_motors_effort = 0
    right_motors_effort = 0
    direction = ""
    is_forward = False

    if error > -20.0 and error < 20.0: #Forward
        #print("Forward")
        direction = "Forward"
        is_forward = True
    elif error > 0: 
        #print("Left")
        direction = "Left"
    else: 
        # print("Right")
        direction = "Right"

    if is_forward or pid_output > 100.0:
        left_motors_effort = 100.0
        right_motors_effort = 100.0
    else:
        left_motors_effort = pid_output
        right_motors_effort = pid_output

    print(f"{direction}, L:{left_motors_effort}, R:{right_motors_effort}")

def main():

    Fx = image_width // 2
    max_error = 90 # max_error = Fx - (contour_width // 2)  
    max_motor_effort = 100.0 
    kp = max_motor_effort / max_error
    ki = 0.0
    kd = 0.0
    pid = PID(1.0,Fx,kp,ki,kd)

    #Setup camera
    cap = cv2.VideoCapture(0)
    cap.set(3,image_width)
    cap.set(4,image_height)

    while cap.isOpened():
        _, img = cap.read()
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        binary_image = getBinaryImage(gray_image)
        Cx, Cy, contour_width = getCentroid(binary_image)

        cv2.circle(img, (Cx,Cy), radius, color, thickness)

        pid_output = pid.compute_adjustment(Cx)
        sendMotorInstruction(abs(pid_output), pid.getError())

        # print(Fx - (contour_width//2))

        cv2.imshow("Floor View orignal", img)
        cv2.imshow("Floor View binary", binary_image)

        if cv2.waitKey(5) & 0xFF == 113:
            break

    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    main()
