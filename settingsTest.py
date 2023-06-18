import cv2 as cv
import os
vid = cv.VideoCapture()
vid.open("/dev/video2")
vid.set(cv.CAP_PROP_AUTO_EXPOSURE, 0)
os.system(f"v4l2-ctl -d /dev/video2 -c gain_automatic=0")
cv.namedWindow("frame")
cv.createTrackbar("brightness", "frame", 0, 255, lambda x: None)
cv.createTrackbar("contrast", "frame", 32, 255, lambda x: None)
cv.createTrackbar("saturation", "frame", 64, 255, lambda x: None)
cv.createTrackbar("hue", "frame", 0, 255, lambda x: None)
cv.createTrackbar("exposure", "frame", 120, 255, lambda x: None)
cv.createTrackbar("gain", "frame", 20, 63, lambda x: None)
# cv.createTrackbar("auto_expose", "frame", 0, 1, lambda x: None)
while True:
    
	vid.set(cv.CAP_PROP_BRIGHTNESS, cv.getTrackbarPos("brightness", "frame"))
	vid.set(cv.CAP_PROP_CONTRAST, cv.getTrackbarPos("contrast", "frame"))
	vid.set(cv.CAP_PROP_SATURATION, cv.getTrackbarPos("saturation", "frame"))
	vid.set(cv.CAP_PROP_HUE, cv.getTrackbarPos("hue", "frame"))
	vid.set(cv.CAP_PROP_EXPOSURE, cv.getTrackbarPos("exposure", "frame"))
	vid.set(cv.CAP_PROP_GAIN, cv.getTrackbarPos("gain", "frame"))
	# vid.set(cv.CAP_PROP_AUTO_EXPOSURE, cv.getTrackbarPos("auto_expose", "frame"))
	ret, frame = vid.read()
	cv.imshow("frame", frame)
	if cv.waitKey(1) == ord('q'):
		break