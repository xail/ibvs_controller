import numpy as np
import cv2 as cv

Threshold = 200


def extract_f_vec(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #cuda_im = cv.cuda_GpuMat(cv.cuda.GpuMat_defaultAllocator())
    #cuda_im.upload(gray)
    detector = cv.AKAZE_create()
    kp, des = detector.detectAndCompute(gray, None)
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        s = np.append(s, [[point.pt[0], point.pt[1]]], axis=0)
    return s


def kp_to_s(kp):
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        s = np.append(s, [[point.pt[0], point.pt[1]]], axis=0)
    return np.delete(s, 0, 0)
