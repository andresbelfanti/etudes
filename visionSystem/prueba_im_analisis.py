import numpy as np
import cv2
import matplotlib.pyplot as plt
from skimage import feature

class LocalBinaryPatterns:
  def __init__(self, numPoints, radius):
    self.numPoints = numPoints
    self.radius = radius

  def describe(self, image, eps = 1e-7):
    lbp = feature.local_binary_pattern(image, self.numPoints, self.radius, method="uniform")
    (hist, _) = np.histogram(lbp.ravel(), bins=np.arange(0, self.numPoints+3), range=(0, self.numPoints + 2))

    # Normalize the histogram
    hist = hist.astype('float')
    hist /= (hist.sum() + eps)

    return hist, lbp
#####################################################################################


def emboss(image):
    kernel = np.array([[0,-1,-1],
                            [1,0,-1],
                            [1,1,0]])
    return cv2.filter2D(image, -1, kernel)


#capture = cv2.VideoCapture("001.mp4")
capture = cv2.VideoCapture(2)

desc = LocalBinaryPatterns(24, 8)
#contrast = contrast.flatten()
#dissimilarity = dissimilarity.flatten()
#homogeneity = homogeneity.flatten()
#energy = energy.flatten()
#correlation = correlation.flatten()
#ASM = ASM.flatten()
#hist = hist.flatten()

#features = np.concatenate((contrast, dissimilarity, homogeneity, energy, correlation, ASM, hist), axis=0) 
#capture = cv2.VideoCapture(0)

while capture.isOpened():
    # grab the current frame and initialize the status text
    grabbed, frame = capture.read()
    image = frame

    if frame is not None:
        # convert the frame to grayscale, blur it, and detect circles
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray,5) 
      #  circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,1,20, param1=50,param2=30,minRadius=0,maxRadius=0)

        circles = cv2.HoughCircles(image=blur, method=cv2.HOUGH_GRADIENT, dp=0.05, 
                            minDist=0.1, param1=110, param2=39,minRadius=0, maxRadius=70)

        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            #circles = np.uint16(np.around(circles[0,:]))

            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(blur, (x, y), r, (0, 0, 0), 2)

            # show the frame and record if a key is pressed
    cv2.imshow("Frame", blur)
    ###############################
    cv2.imshow("NN", frame)
    hist, lbp = desc.describe(gray)
    print("Histogram of Local Binary Pattern value: {}".format(hist))

    ##########################

    thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3)

    canny = cv2.Canny(thresh, 120, 255, 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    opening = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
    dilate = cv2.dilate(opening, kernel, iterations=2)

    cnts = cv2.findContours(dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 3000
    for c in cnts:
        area = cv2.contourArea(c)
        if area > min_area:
            cv2.drawContours(image, [c], -1, (36, 255, 12), 2)

    cv2.imshow('image', image)
        
###############################################

    cv2.imshow("emboss", lbp)
 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


cv2.imwrite('image.png', image)
cv2.waitKey(0)