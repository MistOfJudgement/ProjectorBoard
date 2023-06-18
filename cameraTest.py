import cv2
import numpy as np
def empty(x): pass
cam = cv2.VideoCapture()
cam.open("/dev/video2")
cv2.namedWindow("frame")
cv2.createTrackbar("Saving?", "frame", 0, 1, empty)
saved = np.full(cam.read()[1].shape[:2], 0, dtype=np.uint8)

cv2.namedWindow("thresh")
cv2.createTrackbar("threshold", "thresh", 0, 255, empty)
#camera loop
while True:
    ret, frame = cam.read()
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(frame, cv2.getTrackbarPos("threshold", "thresh"), 255, cv2.THRESH_BINARY)
    saved = saved + bw
    if cv2.getTrackbarPos("Saving?", "frame") == 0:
        saved = np.full(bw.shape, 0, dtype=np.uint8)
    cv2.imshow("frame", frame)
    cv2.imshow("thresh", bw)
    cv2.imshow("saved", saved)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()

cv2.destroyAllWindows()
