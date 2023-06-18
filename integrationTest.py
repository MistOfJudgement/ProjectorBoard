import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import os
cameraPath = "/dev/video2"
video = cv.VideoCapture()
video.open(cameraPath)
# video.set(cv.CAP_PROP_AUTO_EXPOSURE, 0)
# os.system(f"v4l2-ctl -d {cameraPath} -c gain_automatic=0")
os.system(f"v4l2-ctl -d {cameraPath} -c gain_automatic=1")
#so opencv doesn't really do the finding of window sizes so I'll not worry about that
whiteboardSize = (1920, 1080)

arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard((8, 5), 0.04, 0.02, arucoDict)
calibrationImage = board.generateImage(whiteboardSize)

detector = aruco.CharucoDetector(board)
destCorners, destIds, destMarkers, destMarkerIds = detector.detectBoard(calibrationImage)

#sort the corners and ids so they match up
destIds, destCorners = zip(*sorted(zip(destIds, destCorners), key=lambda x: x[0]))
cv.namedWindow("camera", cv.WINDOW_NORMAL)

cv.namedWindow("whiteboard", cv.WINDOW_NORMAL)
cv.createTrackbar("brightness", "camera", 0, 255, lambda x: None)
cv.createTrackbar("threshold", "camera", 0, 255, lambda x: None)
# cv.namedWindow("camera", cv.WINDOW_NORMAL)
calibrating = True
H: np.ndarray = None

while calibrating:
	ret, frame = video.read()
	corners, ids, markers, markerIds = detector.detectBoard(frame)
	
	if ids is not None:
	# 	# H, mask = cv.findHomography(corners, destCorners)
	# 	# calibrating = False
	# 	# break
		aruco.drawDetectedCornersCharuco(frame, corners, ids, (255, 0, 0)) 
  
	if ids is not None and len(ids) > 7:
		ids, corners = zip(*sorted(zip(ids, corners), key=lambda x: x[0]))
		workingDestCorners = [c for i, c in zip(destIds, destCorners) if i in ids] 
		corners = np.array(corners)
		workingDestCorners = np.array(workingDestCorners)
		H, mask = cv.findHomography(corners, workingDestCorners)
		# frame = cv.warpPerspective(frame, H, whiteboardSize)
		# calibrating = False
	cv.imshow("camera", frame)
	cv.imshow("whiteboard", calibrationImage)
	
	if cv.waitKey(1) == ord('q'):
		calibrating = False
		break
	if cv.waitKey(1) == ord('c'):
		if H is not None:
			calibrating = False
			break
		else:
			print("Calibration failed, not enough markers")
			continue

print("Calibration complete")

saved = np.full((whiteboardSize[1], whiteboardSize[0]), 255, dtype=np.uint8)
cv.createTrackbar("saving?", "whiteboard", 0, 1, lambda x: None)

while True:
	video.set(cv.CAP_PROP_BRIGHTNESS, cv.getTrackbarPos("brightness", "camera"))
    
	ret, frame = video.read()
	bw= cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
	_, bw = cv.threshold(bw, cv.getTrackbarPos("threshold", "camera"), 255, cv.THRESH_BINARY)

	
	render = cv.warpPerspective(bw, H, whiteboardSize)
	#draw border around whiteboard
	# outline = np.full((whiteboardSize[1], whiteboardSize[0]), 0, dtype=np.uint8)
	# cv.rectangle(outline, (0, 0, whiteboardSize[0], whiteboardSize[1]), 255, 10)
	# outline = cv.warpPerspective(outline, H, whiteboardSize, cv.WARP_INVERSE_MAP)
	# HInv = np.linalg.inv(H)
	# outline = cv.warpPerspective(outline, HInv, bw.shape[::-1])
	# cv.imshow("outline", outline)
	
	
	saved = cv.bitwise_and(saved, saved, mask=cv.bitwise_not(render))
	# cv.imshow("render", frame)
	outline = np.full((whiteboardSize[1], whiteboardSize[0]), 0, dtype=np.uint8)
	cv.rectangle(outline, (0, 0, whiteboardSize[0], whiteboardSize[1]), 255, 25)
	outline = cv.warpPerspective(outline, H, frame.shape[:2][::-1], flags=cv.WARP_INVERSE_MAP)
	bw = bw + outline
	if cv.getTrackbarPos("saving?", "whiteboard") == 0:
		saved = np.full((whiteboardSize[1], whiteboardSize[0]), 255, dtype=np.uint8)
	
	cv.imshow("whiteboard", saved)
	cv.imshow("camera", bw)
	if cv.waitKey(1) == ord('q'):
		break

# cv.destroyAllWindows()
# video.release()