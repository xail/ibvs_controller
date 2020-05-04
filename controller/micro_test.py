import numpy as np

A_def = np.asarray([[0.9853, -0.0028], [0.0380, 0.6594]])
B_def = np.asarray([[-1.5032, 0.8525], [6.3636, -13.7924]])
K_1_def = 10 * np.ones([3, 3])
K_2_def = 5 * np.ones([3, 3])
K_mu_def = np.asarray([[8, 0], [0, 20]])
H_def = 0.5 * np.ones([8, 8])
alpha_def = - np.ones([2, 2])
gamma_def = np.ones([2, 2])


def Reg(dt, nu, L, z_nu_prev, z_e_prev, z_mu_prev, K_1=K_1_def, K_2=K_2_def, K_mu=K_mu_def, A=None, B=None):
    if B is None:
        B = B_def
    if A is None:
        A = A_def
    if z_e_prev is None:
        z_e_prev = np.zeros([len(L), 2])
    K_e = np.transpose(L)
    z_mu_right = - np.dot(B, np.dot(K_e, z_e_prev)) * dt + np.dot(np.dot(np.transpose(R(nu)), K_1)*dt,
                                                           (nu - np.dot(np.linalg.pinv(K_2*dt + np.ones([3, 3])),
                                                                       np.dot(K_2, nu) * dt + z_nu_prev))) + z_mu_prev
    z_mu_left = np.linalg.pinv(np.ones([2, 2]) - A * dt +
                               np.dot(np.dot(np.dot(np.transpose(R(nu)), K_1*dt), np.linalg.pinv(K_2*dt + np.ones([3, 3]))), R(nu) * dt)
                               + np.dot(B*dt, np.dot(K_e, L*dt) + K_mu))
    z_mu = np.dot(z_mu_left, z_mu_right)
    z_e = np.dot(L, z_mu)*dt + z_e_prev
    z_nu = np.dot(np.linalg.pinv(K_2 * dt + np.ones([3, 3])),
                  np.dot(R(nu), z_mu) * dt + np.dot(K_2, nu) * dt + z_nu_prev)
    tau = -K_mu*z_mu - K_e*z_e
    return tau, z_nu, z_e, z_mu


def F_1_0(nu, K_1=K_1_def):
    return -np.transpose(R(nu))*K_1


def R(nu):
    return np.asarray([[np.cos(nu[2]), 0.0], [np.sin(nu[2]), 0.0], [0.0, 1.0]])


L = np.zeros([52, 2])
z_e_prev = None
z_mu_prev = np.zeros(2)
z_nu_prev = np.zeros(3)
dt = 1.0
nu = np.array([0, 0, 0])


m, z_nu, z_e, z_mu = Reg(dt, nu, L, z_nu_prev, z_e_prev, z_mu_prev)
print(m)