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


def kp_to_s(kp, x=0, y=0):
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        s = np.append(s, [[point.pt[0] - x, point.pt[1] - y]], axis=0)
    return np.delete(s, 0, 0)


def kp_to_s_with_K(kp, K, x=0, y=0):
    K_inv = np.linalg.inv(K)
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        point_buff = np.dot(K_inv, np.array([point.pt[0] - x, point.pt[1] - y, 1]))
        s = np.append(s, [[point_buff[0], point_buff[1]]], axis=0)
    return np.delete(s, 0, 0)


def kp_to_s_with_P_R(kp, P, R):
    P_inv = np.linalg.pinv(P)
    P_new = np.append(R, np.zeros([3, 1]), axis=1)
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        point_buff = np.dot(P_new, np.dot(P_inv, np.array([point.pt[0], point.pt[1], 1])))
        s = np.append(s, [[point_buff[0], point_buff[1]]], axis=0)
    return np.delete(s, 0, 0)


def kp_to_s_with_KPR(kp, K, P, R, x=0, y=0):
    K_inv = np.linalg.inv(K)
    P_inv = np.linalg.pinv(P)
    #T = np.zeros([3, 1])
    x_c = 0.064
    y_c = -0.065
    z_c = 0.094
    T = np.array([-x_c, -y_c, -z_c]).reshape(3, 1)
    P_new = np.append(R, T, axis=1)
    s = np.ndarray(shape=(1, 2), dtype=float)
    for point in kp:
        point_buff = np.dot(K_inv, np.dot(P_new, np.dot(P_inv, np.array([point.pt[0], point.pt[1], 1]))))
        s = np.append(s, [[point_buff[0], point_buff[1]]], axis=0)
    return np.delete(s, 0, 0)