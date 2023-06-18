import cv2 as cv
import numpy as np

cameraPath = "/dev/video2"
video = cv.VideoCapture()
video.open(cameraPath)
chessboardSize = (10, 7)
chessboardSquareSize = 150
chessboardImage = np.zeros((chessboardSize[1]*chessboardSquareSize, chessboardSize[0]*chessboardSquareSize, 3), np.uint8)
cv.namedWindow("camera", cv.WINDOW_NORMAL)
cv.namedWindow("chessboard", cv.WINDOW_NORMAL)
# cv.setWindowProperty("chessboard", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
cv.namedWindow("aligned", cv.WINDOW_NORMAL)
dirty = True
desiredCorners = []
def drawChessboard():
	global desiredCorners
	for i in range(0, chessboardSize[0]):
		for j in range(0, chessboardSize[1]):
			if (i+j)%2 == 0:
				cv.rectangle(chessboardImage, (i*chessboardSquareSize, j*chessboardSquareSize), ((i+1)*chessboardSquareSize, (j+1)*chessboardSquareSize), (255, 255, 255), -1)
	ret, desiredCorners = cv.findChessboardCorners(chessboardImage,(chessboardSize[0]-1, chessboardSize[1]-1))
drawChessboard()

while True:
	ret, frame = video.read()
    
	windowSize = cv.getWindowImageRect("chessboard")
	#if the window is resized, resize the chessboard image
	
    
	cv.imshow("chessboard", chessboardImage)
	# findChessboardCorners
	ret, corners = cv.findChessboardCorners(frame, (chessboardSize[0]-1, chessboardSize[1]-1))
	
	if ret:
		print("found corners")
		# for corner in corners:
		# 	cv.circle(frame, (corner[0][0], corner[0][1]), 10, (255, 0, 0), -1)
		cv.drawChessboardCorners(frame, chessboardSize, corners, ret)
		# for corner in corners:
		# 	cv.circle(frame, (corner[0][0], corner[0][1]), 10, (255, 0, 0))
			#determine if the corner is one of the vertex corners in order to determine the draw window
		H, mask = cv.findHomography(corners, desiredCorners)
		if ret:
			aligned = cv.warpPerspective(frame, H, (chessboardImage.shape[1], chessboardImage.shape[0]))
			cv.imshow("aligned", aligned)
			

	cv.imshow("camera", frame)
		# cv.waitKey(0)
		# cv.destroyAllWindows()
		# break
    
	# cv.imshow("camera", frame)
	if cv.waitKey(1) == ord('q'):
		break

video.release()
cv.destroyAllWindows()