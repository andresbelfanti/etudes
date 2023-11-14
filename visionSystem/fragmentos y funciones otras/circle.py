import numpy as np
import cv2

def emboss(image):
    kernel = np.array([[0,-1,-1],
                            [1,0,-1],
                            [1,1,0]])
    return cv2.filter2D(image, -1, kernel)


#capture = cv2.VideoCapture("001.mp4")
capture = cv2.VideoCapture(0)

#capture = cv2.VideoCapture(0)

while capture.isOpened():
    # grab the current frame and initialize the status text
    grabbed, frame = capture.read()

    if frame is not None:
        # convert the frame to grayscale, blur it, and detect circles
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray,5) 
      #  circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,1,20, param1=50,param2=30,minRadius=0,maxRadius=0)

        circles = cv2.HoughCircles(image=blur, method=cv2.HOUGH_GRADIENT, dp=0.9, 
                            minDist=50, param1=110, param2=39,minRadius=10, maxRadius=40)

        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            #circles = np.uint16(np.around(circles[0,:]))

            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(blur, (x, y), r, (255, 0, 255), 2)

            # show the frame and record if a key is pressed
    cv2.imshow("Frame", blur)
    cv2.imshow("emboss", emboss(gray))
            # if the 'q' key is pressed, stop the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()