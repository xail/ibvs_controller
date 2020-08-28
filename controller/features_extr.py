import numpy as np
import cv2 as cv

Threshold = 200

# legacy version of feature extraction
def extract_f_vec(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    detector = cv.AKAZE_create()
    kp, des = detector.detectAndCompute(gray, None)
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        s = np.append(s, [[point.pt[0], point.pt[1]]], axis=0)
    return s

# function for converting keypoints to s feature vector
def kp_to_s(kp, x=0, y=0):
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        s = np.append(s, [[point.pt[0] - x, point.pt[1] - y]], axis=0)
    return np.delete(s, 0, 0)
