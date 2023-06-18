import cv2 as cv
import cv2.aruco as aruco
import numpy as np

from matplotlib import pyplot as plt

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
marker = aruco.generateImageMarker(dictionary, 23, 200, 1)
cv.imshow("win", marker)
while True:
    if cv.waitKey(1) & 0xFF == ord('q'): break
    
cv.destroyAllWindows()