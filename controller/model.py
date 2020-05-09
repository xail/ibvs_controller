import numpy as np
import scipy.signal as sig

def R(eta):
    return np.asarray([[np.cos(eta[2]), 0.0], [np.sin(eta[2]), 0.0], [0.0, -1.0]])


def T(L, H_e):
    right = np.dot(np.transpose(L), H_e)
    left_1 = -np.linalg.pinv(np.transpose(L).dot(L))
    left_2 = np.dot(left_1, np.transpose(L))
    return np.dot(left_2, H_e)


def F_1(nu):
    return -np.transpose(R(nu)).dot(K_1)


def F_2(L, K_e, H_e):
    return -K_e + A.dot(T(L, H_e)) + K_v.dot(T(L, H_e))


# A = np.asarray([[0.9804, -0.0219], [0.0252, 0.9473]])
# B = np.asarray([[0.3071, 0.3071], [-0.8010, -0.8010]])
A = -np.identity(2)
B = np.identity(2)

K_1 = 20 * np.identity(3)
K_2 = 6 * np.identity(3)
K_v = np.asarray([[3, 0], [0, 10]])
#K_v = np.array([[0.2630,-0.0413],[0.2630 ,  -0.0413]])

#H_e = 0.5 * np.identity(8)
alpha = - np.identity(2)
gamma = np.identity(2)

mu_eta = 0
mu_e = 0
nu0 = np.zeros(3)

sys = sig.lti(A, B, np.identity(2), [[0.,0.],[0.,0.]])
