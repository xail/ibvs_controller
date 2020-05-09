import cv2 as cv
import numpy as np


def b_force(kp_des, desc_des, kp_s, desc_s):
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    kp_num = 100
    acc = 25.

    # Match descriptors.
    matches = bf.match(desc_des, desc_s)

    # Sort them in the order of their distance.
    matches = sorted(matches, key=lambda x: x.distance)

    src_pts = np.float32([kp_des[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp_s[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)

    matchesMask = mask.ravel().tolist()
    inliers = []
    for idx, match in enumerate(matches):
        if matchesMask[idx] == 1:
            inliers.append(match)

    obj_len = 0
    test_len = 0
    for idx1, inl1 in enumerate(inliers):
        for idx2, inl2 in enumerate(inliers):
            obj_len += np.linalg.norm(np.float32(kp_des[inl1.queryIdx].pt) - np.float32(kp_des[inl2.queryIdx].pt))
            test_len += np.linalg.norm(np.float32(kp_s[inl1.trainIdx].pt) - np.float32(kp_s[inl2.trainIdx].pt))

    valid_kp_des = []
    valid_kp_s = []
    if len(inliers) == 0:
        return valid_kp_des, valid_kp_s, 0, 1
    if len(inliers) > kp_num:
        inliers = inliers[:kp_num]
    k_in_acc_range = 0
    for inl in inliers:
        if inl.distance > acc:
            break
        k_in_acc_range += 1
    for i in range(0, len(inliers)-1):
        valid_kp_s = np.append(valid_kp_s, kp_s[inliers[i].trainIdx])
        valid_kp_des = np.append(valid_kp_des, kp_des[inliers[i].queryIdx])
    return valid_kp_des, valid_kp_s, float(k_in_acc_range)/float(kp_num), test_len/obj_len


