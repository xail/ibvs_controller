import numpy as np


def R(eta):
    return np.asarray([[np.cos(eta[2]), 0.0], [np.sin(eta[2]), 0.0], [0.0, 1.0]])


def T(L, H_e):
    right = np.dot(np.transpose(L), H_e)
    left_1 = -np.linalg.pinv(np.transpose(L).dot(L))
    left_2 = np.dot(left_1, np.transpose(L))
    return np.dot(left_2, H_e)


def F_1(nu):
    return -np.transpose(R(nu)).dot(K_1)


def F_2(L, K_e, H_e):
    return -K_e + A.dot(T(L, H_e)) + K_v.dot(T(L, H_e))


A = -np.identity(2)
B = np.identity(2)

K_1 = 10 * np.identity(3)
K_2 = 5 * np.identity(3)
K_v = np.asarray([[8, 0], [0, 20]])

#H_e = 0.5 * np.identity(8)
alpha = - np.identity(2)
gamma = np.identity(2)

mu_eta = 0
mu_e = 0
nu0 = np.zeros(3)

