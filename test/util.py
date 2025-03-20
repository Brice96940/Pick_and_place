import numpy as np
import cv2 as cv

def get_limits(color):

    c =  np.uint8([[color]])
    hsvc = cv.cvtColor(c, cv.COLOR_BGR2HSV)

    lowerLimit = hsvc[0][0][0] -10, 120, 120
    UpperLimit = hsvc[0][0][0] +10, 255, 255

    if lowerLimit[0] < 0:
        lowerLimit = hsvc[0][0][0] -0, 120, 120

    print(lowerLimit,UpperLimit)

    lowerLimit = np.array(lowerLimit, dtype= np.uint8)
    UpperLimit = np.array(UpperLimit, dtype= np.uint8)

    return lowerLimit, UpperLimit

