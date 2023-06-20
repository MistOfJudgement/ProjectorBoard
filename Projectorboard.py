import cv2 as cv
import cv2.aruco as aruco
import numpy as np

def sortUsingIds(ids, corners):
    ids, corners = zip(*sorted(zip(ids, corners), key=lambda x: x[0]))
    ids = np.array(ids)
    corners = np.array(corners)
    return ids, corners
class CalibrationBoard:
    def __init__(self, name="calibration") -> None:
        self.area = (1920, 1080)
        self.name = name
        self.board = aruco.CharucoBoard((8, 5), 0.04, 0.02, aruco.getPredefinedDictionary(aruco.DICT_4X4_50))
        self.detector = aruco.CharucoDetector(self.board)
        self.calibrationImage = self.board.generateImage(self.area)
        self.destCorners, self.destIds, self.destMarkers, self.destMarkerIds = self.detector.detectBoard(self.calibrationImage)
        self.destIds, self.destCorners = sortUsingIds(self.destIds, self.destCorners)
        # self.destMarkerIds, self.destMarkers = sortUsingIds(self.destMarkerIds, self.destMarkers)
        cv.namedWindow(name, cv.WINDOW_NORMAL)
    def getMatchingCorners(self, ids):
        return np.asarray([c for i, c in zip(self.destIds, self.destCorners) if i in ids])
    def display(self) -> None:
        cv.imshow(self.name, self.calibrationImage)
"""The CalibratedCamera is responsible for calibrating the camera and keeping calibrated, along with returning the transformed image."""
class CalibratedCamera:

    def __init__(self, calibrator=CalibrationBoard(), name="camera") -> None:
        self.camera = cv.VideoCapture()        
        self.calibrator = calibrator
        self.calibrated = False
        self.H: np.ndarray = None
        self.workingAreaSize = self.calibrator.area
        self.minMarkers = 4
        self.name = name
    def displayCalibrationWindow(self) -> None:
        cv.namedWindow("camera", cv.WINDOW_NORMAL)
        cv.createTrackbar("brightness", "camera", 0, 255, lambda x: self.camera.set(cv.CAP_PROP_BRIGHTNESS, x))
    
    def display(self) -> None:
        ret, frame = self.camera.read()
        if not ret:
            return
        cv.imshow("camera", frame)
    def tryCalibration(self) -> bool:
        self.calibrator.display()
        ret, frame = self.camera.read()
        if not ret:
            return False
        corners, ids, markers, markerIds = self.calibrator.detector.detectBoard(frame)
        if ids is not None and len(ids) > self.minMarkers:
            ids, corners = sortUsingIds(ids, corners)
            usableCorners = self.calibrator.getMatchingCorners(ids)
            self.H, _ = cv.findHomography(corners, usableCorners)
            self.calibrated = True
            return True
        return False
    
    """If the camera is calibrated, returns the transformed image, otherwise returns the raw image."""
    def read(self) -> np.ndarray:
        ret, frame = self.camera.read()
        if not ret:
            return None
        if self.calibrated:
            frame = cv.warpPerspective(frame, self.H, self.workingAreaSize)
        return frame
        


        
def projectorboard():
    camID = "/dev/video2"
    calibrator = CalibrationBoard()
    camera = CalibratedCamera(calibrator)
    camera.minMarkers = 8
    camera.camera.open(camID)
    camera.displayCalibrationWindow()
    while True:
        camera.display()
        camera.calibrator.display()
        key = cv.waitKey(1)
        if key == ord('q'):
            break
        if key == ord('c'):
            if camera.tryCalibration():
                break
    cv.destroyWindow(camera.name)
    while True:
        frame = camera.read()
        if frame is None:
            break
        cv.imshow(calibrator.name, frame)
        camera.display()
        if cv.waitKey(1) == ord('q'):
            break


if __name__ == "__main__":
    projectorboard()