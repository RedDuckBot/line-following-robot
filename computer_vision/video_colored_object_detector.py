import cv2
import numpy as np

"""
This program is able to detect certain colors of objects and draw rectangle 
around the identified object. Currently, the program detects green objects.
"""

videoCap = cv2.VideoCapture(4)

# Define HSV range for the green color (Hue, Saturation , value)
lowerGreen = np.array([40, 70, 70])    
upperGreen = np.array([80, 255, 255])  

area_threshold = 400 #The minimum expected number of pixels the object of interest may occupy

rec_box_color = (0,255,0)
rec_line_thickness = 3

while True:
    ret, originalFrame = videoCap.read()
    
    hsvFrame = cv2.cvtColor(originalFrame, cv2.COLOR_BGR2HSV)

    # Create a binary mask where green colors are within the range
    maskFrame = cv2.inRange(hsvFrame, lowerGreen, upperGreen)

    # Use morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    maskFrame = cv2.erode(maskFrame, kernel, iterations=1)
    maskFrame = cv2.dilate(maskFrame, kernel, iterations=2)

    contours, _ = cv2.findContours(maskFrame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > area_threshold:  
            # Draw a bounding rectangle around the largest green object
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(originalFrame, (x, y), (x + w, y + h),rec_box_color, rec_line_thickness)
            cv2.putText(originalFrame, "Green Object Detected", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, rec_box_color, 2)

    cv2.imshow("Camera Feed", originalFrame)
    cv2.imshow("Mask Feed", maskFrame)

    #Press q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

videoCap.release()
cv2.destroyAllWindows()

