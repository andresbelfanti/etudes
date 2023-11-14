#Detect circles with a webcam using opencv
import cv2 #Use this form instead of "import cv2" to get rid of cv2 errors in VS Code editor. https://stackoverflow.com/a/61077968/51358
import numpy as np

##==================================================================

def circleDetection(frame):
    # convert image to grayscale
    try:
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except:
        print("cvt error")
    # apply a blur using the median filter
    img = cv2.medianBlur(img, 5)    
    # find circles in grayscale image using the Hough transform
    circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT, dp=0.009, 
                            minDist=0.01, param1=50, param2=10, minRadius=1, maxRadius=10)

#gray: Input image (grayscale).
#circles: A vector that stores sets of 3 values: xc,yc,r for each detected circle.
#HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV.
#dp = 1: The inverse ratio of resolution.
#min_dist = gray.rows/16: Minimum distance between detected centers.
#param_1 = 200: Upper threshold for the internal Canny edge detector.
#param_2 = 100*: Threshold for center detection.
#min_radius = 0: Minimum radius to be detected. If unknown, put zero as default.
#max_radius = 0: Maximum radius to be detected. If unknown, put zero as default

    if np.any(circles):
        print("------------------")
        print("number of circles: ", len(circles[0, :]))

        for co, i in enumerate(circles[0, :]):
            # draw the outer circle with radius i[2] in green
           # cv2.rectangle(img, (x, y), (20,20), (0, 0, 100), 3)
            # draw the center as a circle with radius 2 in red
            cv2.circle(frame,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),1)
            
    try:
        cv2.imshow('Video', frame)
    except:
        print("frames")

#============================================
## start code


video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()
    circleDetection(frame)
          # if the 'q' key is pressed, stop the loop
    if cv2.waitKey(1) & 0xFF == ord('&'):
        break
# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
