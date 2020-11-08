import cv2 
import numpy as np
import matplotlib.pyplot as plt

stream = cv2.VideoCapture('http://localhost:8081/stream/video.mjpeg')

while ( stream.isOpened() ):
    # Read a frame from the stream
    ret, img = stream.read()
    if ret: # ret == True if stream.read() was successful
        
        # Start coordinate, here (5, 5) 
        # represents the top left corner of rectangle 
        start_point = (400, 400) 
        
        # Ending coordinate, here (220, 220) 
        # represents the bottom right corner of rectangle 
        end_point = (600, 600) 
        
        # Blue color in BGR 
        color = (255, 0, 0) 
        
        # Line thickness of 2 px 
        thickness = 5
        
        # Using cv2.rectangle() method 
        # Draw a rectangle with blue line borders of thickness of 2 px 
        img = cv2.rectangle(img, start_point, end_point, color, thickness) 
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        colour = "red"
        if colour == "red":
            lower = np.array([160, 80, 80])
            upper = np.array([190, 255, 255])
        if colour == "orange":
            lower = np.array([0, 80, 80])
            upper = np.array([28, 255, 255])
        if colour == "yellow":
            lower = np.array([20, 80, 80])
            upper = np.array([45, 255, 255])
        if colour == "green":
            lower = np.array([40, 80, 80])
            upper = np.array([80, 255, 255])
        if colour == "blue":
            lower = np.array([80, 80, 80])
            upper = np.array([140, 255, 255])
        if colour == "pink":
            lower = np.array([140, 80, 80])
            upper = np.array([160, 255, 255])   #, np.uint8)

        mask = cv2.inRange(hsv, lower, upper)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        img = cv2.drawContours(img, contours, -1, (0, 255, 0), 3)

        plt.imshow(img)
        plt.show()