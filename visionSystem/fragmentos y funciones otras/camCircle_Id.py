#Detect circles with a webcam using opencv
import cv2 #Use this form instead of "import cv2" to get rid of cv2 errors in VS Code editor. https://stackoverflow.com/a/61077968/51358
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort

tracker = DeepSort(max_age=5)

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
    circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT, dp=10, 
                            minDist=1, param1=150, param2=10, minRadius=1, maxRadius=10)
#gray: Input image (grayscale).
#circles: A vector that stores sets of 3 values: xc,yc,r for each detected circle.
#HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV.
#dp = 1: The inverse ratio of resolution.
#min_dist = gray.rows/16: Minimum distance between detected centers.
#param_1 = 200: Upper threshold for the internal Canny edge detector.
#param_2 = 100*: Threshold for center detection.
#min_radius = 0: Minimum radius to be detected. If unknown, put zero as default.
#max_radius = 0: Maximum radius to be detected. If unknown, put zero as default
    results = []
    if np.any(circles):
      #  print("------------------")
       # print("number of circles: ", len(circles[0, :]))
        for co, data in enumerate(circles[0, :]):

            # draw the outer circle with radius i[2] in green
           # cv2.circle(frame,(int(data[0]),int(data[1])),int(data[2]),(0,255,0),1)
            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[0]+data[2]), int(data[1]+data[2])
            # add the bounding box (x, y, w, h), confidence and class id to the results list

            results.append([[xmin, ymin, xmax, ymax], 0.5, 1])

    # 2. Object Tracking
        # update the tracker with the new detections
    tracks = tracker.update_tracks(results, frame=frame)
    for track in tracks:
        if not track.is_confirmed():
            continue
        # get the track id and the bounding box
        track_id = track.track_id
        ltrb = track.to_ltrb()


        xmin, ymin, xmax, ymax, nombre = int(ltrb[0]), int(ltrb[1]), int(ltrb[2]), int(ltrb[3])
        cv2.circle(frame,(xmax-xmin/2, ymax-ymin/2),(0,255,0),1)
       # cv2.putText(frame,str(track_id), xmax, ymax, 12)
        #cv2.imshow(str(track_id),frame[ymin:ymax+1, xmin:xmax+1]) #Cropping // pasar a array de img y combinar



#============================================
## start code


video_capture = cv2.VideoCapture("/home/andrex/ANDY/shared/CCEBA 2023/CCEBA_codes y material/etudes/visionSystem/img_test/video_2023-11-09_18-46-31.mp4")

while True:
    ret, frame = video_capture.read()
    circleDetection(frame)
          # if the 'q' key is pressed, stop the loop
    if cv2.waitKey(1) & 0xFF == ord('&'):
        break

                
    try:
        cv2.imshow('Video', frame)
    except:
        print("frames")

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
