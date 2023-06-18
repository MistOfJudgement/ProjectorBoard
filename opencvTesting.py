import cv2
import numpy as np

nothing = lambda x: None
vid = cv2.VideoCapture()
vid.open("/dev/video2")

cv2.namedWindow("frame")
cv2.createTrackbar("lowthreshold", "frame", 0, 255, nothing)

cv2.createTrackbar("highthreshold", "frame", 0, 255, nothing)
while True:
    ret, frame = vid.read()
    _, frame = cv2.threshold(frame, cv2.getTrackbarPos("lowthreshold", "frame"), cv2.getTrackbarPos("highthreshold", "frame"), cv2.THRESH_BINARY)
    if(frame.shape[0] <= 0): continue
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
vid.release()
cv2.destroyAllWindows()
