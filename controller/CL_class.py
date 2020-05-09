import numpy as np
import cv2 as cv
import controller.bfmtch as bf
import controller.features_extr as fe
import rclpy
from os.path import expanduser
import controller.motor as mtr
import controller.eff_reg as er

home = expanduser("~")

folder = home + '/resources/'
acc = 0.8
K = np.array([[804.5931813021568, 0.0, 640.5], [0.0, 804.5931813021568, 360.5], [0.0, 0.0, 1.0]])
P = np.array([[804.5931813021568, 0.0, 640.5, -56.32152269115098],
             [0.0, 804.5931813021568, 360.5, 0.0],
             [0.0, 0.0, 1.0, 0.0]])
R = np.array([[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]])


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
        self.vel_logger = []
        self.m_logger = []
        self.e_logger = []
        self.z_e_logger = []
        self.z_v_logger = []
        self.eta_logger = []
        self.z_eta_logger = []
        self.p_logger = []
        self.K_e_z_e_logger = []
        self.L_logger = []
        self.time_logger = []

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

    def vel(self, img, l_type=0):
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
        if l_type == 1:
            l_mat = lx
        elif l_type == 2:
            l_mat = lx_sum
        else:
            l_mat = lx_apr
        v = self.lmbd * np.dot(l_mat, s_temp)
        # end point check by accuracy
        if curr_acc > acc:
            self.stop = True

        return v

    def eff(self, img, z_nu_prev, z_e_prev, z_v_prev, tau_prev, clock_sub, vel_sub, pos_sub, p_prev, l_type=0):
        z = np.full(len(self.kp_des), 0.5)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        kp_img, desc_img = self.detector.detectAndCompute(gray, None)
        # no keypoints error check
        if len(kp_img) == 0:
            print("No keypoints on this frame")
            self.error = True
            self.stop = True
            return np.zeros([2]), np.zeros([2]),  z_nu_prev, z_e_prev, z_v_prev, p_prev
        valid_kp_des, valid_kp_img, curr_acc, z_scale = bf.b_force(self.kp_des, self.desc_des, kp_img, desc_img)
        # no valid kp in frame error check
        if len(valid_kp_img) == 0:
            print("No valid keypoints on this frame")
            self.stop = True
            self.error = True
            return np.zeros([2]), np.zeros([2]),  z_nu_prev, z_e_prev, z_v_prev, p_prev
        width, height = img.shape[:2]
        #s = fe.kp_to_s_with_K(valid_kp_img, K, width/2, height/2)
        #s_des = fe.kp_to_s_with_K(valid_kp_des, K, width/2, height/2)
        # s = fe.kp_to_s_with_KPR(valid_kp_img, K, P, R)#, width/2)#, height)
        # s_des = fe.kp_to_s_with_KPR(valid_kp_des, K, P, R)#, width/2)#, height)
        s = fe.kp_to_s(valid_kp_img, width / 2, height/2)
        s_des = fe.kp_to_s(valid_kp_des, width / 2, height/2)
        s_temp = s - s_des
        lx_apr = self.__lx(s, z)
        L0 = self.__lx(s_des, z)
        z /= z_scale
        #print('z=', z_scale)
        lx = self.__lx(s, z)
        lx_sum = (lx + lx_apr)/2
        s_temp = np.reshape(s_temp, (-1,))
        #print('s_e = ', s_temp[0:10])
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
                s_temp = s_temp[0:len(z_e_prev)]
                L0 = L0[0:len(z_e_prev)]
        else:
            z_e_prev = s_temp
        m, z_eta, z_e, z_v, p, K_e = er.Reg_dist(l_mat, z_nu_prev, z_e_prev, z_v_prev, tau_prev, clock_sub, pos_sub, s_temp,
                                           p_prev, L0)
        # end point check by accuracy
        #print('z=', z[0])
        #print('accur = ', curr_acc)
        if curr_acc > acc:
            self.stop = True
        v = mtr.motor(m, clock_sub, vel_sub)
        self.vel_logger.append(vel_sub.vel2)
        self.m_logger.append(m)
        self.e_logger.append(s_temp)
        self.z_e_logger.append(z_e)
        self.z_v_logger.append(z_v)
        self.eta_logger.append(pos_sub.eta)
        self.z_eta_logger.append(z_eta)
        self.p_logger.append(p)
        self.K_e_z_e_logger.append(K_e)
        self.L_logger.append(l_mat)
        self.time_logger.append(clock_sub.clock)
        # if abs(v[0]) > 1 or abs(v[1]) > 1:
        if z[0] < 0.55 or clock_sub.clock > 60:
            self.stop = True
        return v, m, z_eta, z_e, z_v, p

    def simp_eff(self, img, clock_sub, vel_sub, l_type=0):
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
        # s = fe.kp_to_s_with_KPR(valid_kp_img, K, P, R)#, width/2)#, height)
        # s_des = fe.kp_to_s_with_KPR(valid_kp_des, K, P, R)#, width/2)#, height)
        s = fe.kp_to_s(valid_kp_img, width / 2)
        s_des = fe.kp_to_s(valid_kp_des, width / 2)
        s_temp = s - s_des
        lx_apr = self.__lx(s, z)
        z *= z_scale
        # print('z=', z_scale)
        lx = self.__lx(s, z)
        lx_sum = (lx + lx_apr) / 2
        s_temp = np.reshape(s_temp, (-1,))
        #print('s_e = ', s_temp[0:10])
        if l_type == 1:
            l_mat = lx
        elif l_type == 2:
            l_mat = lx_sum
        else:
            l_mat = lx_apr
        rclpy.spin_once(vel_sub)
        m = er.SimpleReg(vel_sub.vel2, s_temp, l_mat)
        # end point check by accuracy
        if curr_acc > acc:
            self.stop = True
        #print('acc=', curr_acc)
        v = mtr.motor(m, clock_sub, vel_sub)
        self.vel_logger.append(v)
        self.m_logger.append(m)
        self.e_logger.append(s_temp)
        return v

    # def eff(self, img, dt, nu, z_nu_prev, z_e_prev, z_mu_prev, l_type=0):
    #     z = np.full(len(self.kp_des), 1.)
    #     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #     kp_img, desc_img = self.detector.detectAndCompute(gray, None)
    #     # no keypoints error check
    #     if len(kp_img) == 0:
    #         print("No keypoints on this frame")
    #         self.error = True
    #         self.stop = True
    #         return np.zeros([2]), z_nu_prev, z_e_prev, z_mu_prev
    #     valid_kp_des, valid_kp_img, curr_acc, z_scale = bf.b_force(self.kp_des, self.desc_des, kp_img, desc_img)
    #     # no valid kp in frame error check
    #     if len(valid_kp_img) == 0:
    #         print("No valid keypoints on this frame")
    #         self.stop = True
    #         self.error = True
    #         return np.zeros([2]), z_nu_prev, z_e_prev, z_mu_prev
    #     width, height = img.shape[:2]
    #     #s = fe.kp_to_s_with_K(valid_kp_img, K, width/2, height/2)
    #     #s_des = fe.kp_to_s_with_K(valid_kp_des, K, width/2, height/2)
    #     s = fe.kp_to_s_with_KPR(valid_kp_img, K, P, R)#, width/2)#, height)
    #     s_des = fe.kp_to_s_with_KPR(valid_kp_des, K, P, R)#, width/2)#, height)
    #     s_temp = s - s_des
    #     lx_apr = self.__lx(s_temp, z)
    #     z *= z_scale
    #     #print('z=', z_scale)
    #     lx = self.__lx(s, z)
    #     lx_sum = (lx + lx_apr)/2
    #     s_temp = np.reshape(s_temp, (-1,))
    #     print('s_e = ', s_temp[0:10])
    #     if l_type == 1:
    #         l_mat = lx
    #     elif l_type == 2:
    #         l_mat = lx_sum
    #     else:
    #         l_mat = lx_apr
    #     if z_e_prev is not None:
    #         if len(z_e_prev) > len(l_mat):
    #             z_e_prev = z_e_prev[0:len(l_mat)]
    #         elif len(z_e_prev) < len(l_mat):
    #             l_mat = l_mat[0:len(z_e_prev)]
    #     else:
    #         z_e_prev = s_temp
    #     m, z_nu, z_e, z_mu = er.Reg(dt, nu, l_mat, z_nu_prev, z_e_prev, z_mu_prev, s_temp)
    #     # end point check by accuracy
    #     print('accur = ', curr_acc)
    #     if curr_acc > acc:
    #         self.stop = True
    #     #print("z_mu =", z_mu)
    #     #print(m)
    #     return m, z_nu, z_e, z_mu

    # def simp_eff_save(self, img, v, l_type=0):
    #     z = np.full(len(self.kp_des), 1.)
    #     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #     kp_img, desc_img = self.detector.detectAndCompute(gray, None)
    #     # no keypoints error check
    #     if len(kp_img) == 0:
    #         print("No keypoints on this frame")
    #         self.error = True
    #         self.stop = True
    #         return np.zeros([2])
    #     valid_kp_des, valid_kp_img, curr_acc, z_scale = bf.b_force(self.kp_des, self.desc_des, kp_img, desc_img)
    #     # no valid kp in frame error check
    #     if len(valid_kp_img) == 0:
    #         print("No valid keypoints on this frame")
    #         self.stop = True
    #         self.error = True
    #         return np.zeros([2])
    #     width, height = img.shape[:2]
    #     s = fe.kp_to_s_with_KPR(valid_kp_img, K, P, R)#, width/2)#, height)
    #     s_des = fe.kp_to_s_with_KPR(valid_kp_des, K, P, R)#, width/2)#, height)
    #     s_temp = s - s_des
    #     lx_apr = self.__lx(s_temp, z)
    #     z *= z_scale
    #     # print('z=', z_scale)
    #     lx = self.__lx(s, z)
    #     lx_sum = (lx + lx_apr) / 2
    #     s_temp = np.reshape(s_temp, (-1,))
    #     #print('s_e = ', s_temp[0:10])
    #     if l_type == 1:
    #         l_mat = lx
    #     elif l_type == 2:
    #         l_mat = lx_sum
    #     else:
    #         l_mat = lx_apr
    #     m = er.SimpleReg(v, s_temp, l_mat)
    #     # end point check by accuracy
    #     if curr_acc > acc:
    #         self.stop = True
    #     print('acc=', curr_acc)
    #     #print("z_nu =", z_nu)
    #     #print("z_mu =", z_mu)
    #     #print(m)
    #     return m