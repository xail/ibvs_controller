import numpy as np
import cv2 as cv
import controller.bfmtch as bf
import controller.features_extr as fe
from os.path import expanduser
import controller.eff_reg as er

home = expanduser("~")

folder = home + '/resources/'
acc = 0.8


class ControlLaw(object):

    kp_des = []
    desc_des = []

    def __init__(self, lmbd=1, img_filename='Obj.png', folder=folder, detector=cv.AKAZE_create(),
                 DOF=2):
        self.detector = detector
        self.lmbd = lmbd
        self.DOF = DOF
        self.stop = False
        self.error = False
        img = cv.imread(folder + img_filename)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        self.kp_des, self.desc_des = self.detector.detectAndCompute(gray, None)

    @staticmethod
    def __l(x, y, z):
        return [[-1 / z, 0, x / z, x * y, -(1 + np.power(x, 2)), y],
                [0, -1 / z, y / z, 1 + np.power(y, 2), -x * y, -x]]

    def __l_dof(self, x, y, z):
        mask = mask = np.ones(6, dtype=bool)
        if self.DOF == 2:
            mask[[0, 1, 3, 5]] = False
        l = np.array(self.__l(x, y, z))
        return l[:, mask]

    @staticmethod
    # old version for 2DOF
    def __l_2DOF(x, y, z):
        return [[x / z, -(1 + np.power(x, 2))],
                [y / z, -x * y]]

    def __lx(self, s, z):
        x = s[0, 0]
        y = s[0, 1]
        i = 0
        lx = self.__l_dof(x, y, z[i])
        if (len(s) - 1) > 0:
            for i in range(1, len(s)):
                x = s[i, 0]
                y = s[i, 1]
                l_buf = self.__l_dof(x, y, z[i])
                lx = np.append(lx, l_buf, axis=0)
        return lx

    def vel(self, img):
        z = np.full(len(self.kp_des), 0.5)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        kp_img, desc_img = self.detector.detectAndCompute(gray, None)
        # no keypoints error check
        if len(kp_img) == 0:
            print("No keypoints on this frame")
            self.error = True
            self.stop = True
            return np.zeros(shape=(6, 1))
        valid_kp_des, valid_kp_img, curr_acc, z_scale = bf.b_force(self.kp_des, self.desc_des, kp_img, desc_img)
        # no valid kp in frame error check
        if len(valid_kp_img) == 0:
            print("No valid keypoints on this frame")
            self.stop = True
            self.error = True
            return np.zeros(shape=(6, 1))
        width, height = img.shape[:2]
        s = fe.kp_to_s(valid_kp_img, width/2)
        s_des = fe.kp_to_s(valid_kp_des, width/2)
        lx_apr = np.linalg.pinv(self.__lx(s, z))
        z *= z_scale
        lx = np.linalg.pinv(self.__lx(s, z))
        lx_sum = (lx + lx_apr)/2
        s_temp = np.reshape(s - s_des, (-1, 1))
        # print("lx")
        # print(lx)
        # print("lx_apr")
        # print(lx_apr)
        v = self.lmbd * np.dot(lx_sum, s_temp)
        # end point check by accuracy
        if curr_acc > acc:
            self.stop = True

        return v

    def eff(self, img, dt, nu, z_nu_prev, z_e_prev, z_mu_prev, l_type=0):
        z = np.full(len(self.kp_des), 0.5)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        kp_img, desc_img = self.detector.detectAndCompute(gray, None)
        # no keypoints error check
        if len(kp_img) == 0:
            print("No keypoints on this frame")
            self.error = True
            self.stop = True
            return np.zeros([2]), z_nu_prev, z_e_prev, z_mu_prev
        valid_kp_des, valid_kp_img, curr_acc, z_scale = bf.b_force(self.kp_des, self.desc_des, kp_img, desc_img)
        # no valid kp in frame error check
        if len(valid_kp_img) == 0:
            print("No valid keypoints on this frame")
            self.stop = True
            self.error = True
            return np.zeros([2]), z_nu_prev, z_e_prev, z_mu_prev
        width, height = img.shape[:2]
        s = fe.kp_to_s(valid_kp_img, width/2)
        s_des = fe.kp_to_s(valid_kp_des, width/2)
        lx_apr = self.__lx(s, z)
        z *= z_scale
        lx = self.__lx(s, z)
        lx_sum = (lx + lx_apr)/2
        s_temp = np.reshape(s - s_des, (-1,))
        if l_type == 1:
            l_mat = lx
        elif l_type == 2:
            l_mat = lx_sum
        else:
            l_mat = lx_apr
        if z_e_prev is not None:
            if len(z_e_prev) > len(l_mat):
                z_e_prev = z_e_prev[0:len(l_mat)]
            elif len(z_e_prev) < len(l_mat):
                l_mat = l_mat[0:len(z_e_prev)]
        else:
            z_e_prev = s_temp
        m, z_nu, z_e, z_mu = er.Reg(dt, nu, l_mat, z_nu_prev, z_e_prev, z_mu_prev)
        # end point check by accuracy
        if curr_acc > acc:
            self.stop = True
        #print("z_nu =", z_nu)
        #print("z_mu =", z_mu)
        print(m)
        return m*-0.01, z_nu, z_e, z_mu

    def simp_eff(self, img, v, l_type=0):
        z = np.full(len(self.kp_des), 0.5)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        kp_img, desc_img = self.detector.detectAndCompute(gray, None)
        # no keypoints error check
        if len(kp_img) == 0:
            print("No keypoints on this frame")
            self.error = True
            self.stop = True
            return np.zeros([2])
        valid_kp_des, valid_kp_img, curr_acc, z_scale = bf.b_force(self.kp_des, self.desc_des, kp_img, desc_img)
        # no valid kp in frame error check
        if len(valid_kp_img) == 0:
            print("No valid keypoints on this frame")
            self.stop = True
            self.error = True
            return np.zeros([2])
        width, height = img.shape[:2]
        s = fe.kp_to_s(valid_kp_img, width/2)
        s_des = fe.kp_to_s(valid_kp_des, width/2)
        lx_apr = self.__lx(s, z)
        z *= z_scale
        lx = self.__lx(s, z)
        lx_sum = (lx + lx_apr)/2
        s_temp = np.reshape(s - s_des, (-1,))
        if l_type == 1:
            l_mat = lx
        elif l_type == 2:
            l_mat = lx_sum
        else:
            l_mat = lx_apr

        m = er.SimpleReg(v, s_temp, l_mat)
        # end point check by accuracy
        if curr_acc > acc:
            self.stop = True
        #print("z_nu =", z_nu)
        #print("z_mu =", z_mu)
        print(m)
        return m