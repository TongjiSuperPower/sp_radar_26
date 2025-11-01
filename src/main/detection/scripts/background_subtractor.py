import numpy as np
import cv2 as cv

cap = cv.VideoCapture('./src/main/detection/media/20240516BO301QL_short_cut.mp4')	# read from a video file
# cap = cv.VideoCapture(0)   # read from a webcam
fgbg = cv.createBackgroundSubtractorKNN()

while(1):
    ret, frame = cap.read()
    # origin_frame = frame.copy()
    # frame.resize((1080, 1920))
    fgmask = fgbg.apply(frame)
    # fgmask.resize((1920, 1080))
    # contours,hierarchy = cv.findContours(fgmask, 1, 2)
    # for i in range(0,len(contours)):  
    #     x, y, w, h = cv.boundingRect(contours[i])   
    #     cv.rectangle(frame, (x,y), (x+w,y+h), (153,153,0), 5) 
    
    cv.imshow('frame',fgmask)
    # cv.imshow("origin", origin_frame)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv.destroyAllWindows()
