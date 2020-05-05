import numpy as np

A_def = np.asarray([[0.9853, -0.0028], [0.0380, 0.6594]])
B_def = np.asarray([[-1.5032, 0.8525], [6.3636, -13.7924]])
K_1_def = 10 * np.ones([3, 3])
K_2_def = 5 * np.ones([3, 3])
K_mu_def = np.asarray([[8, 0], [0, 20]]) /4
H_def = 0.5 * np.ones([8, 8])
alpha_def = - np.ones([2, 2])
gamma_def = np.ones([2, 2])


def Reg(dt, nu, L, z_nu_prev, z_e_prev, z_mu_prev, K_1=K_1_def, K_2=K_2_def, K_mu=K_mu_def, A=None, B=None):
    if B is None:
        B = B_def
    if A is None:
        A = A_def
    if z_e_prev is None:
        z_e_prev = np.zeros([len(L)])
    K_e = np.transpose(L)
    z_mu_right_1 = - np.dot(B, np.dot(K_e, z_e_prev)) * dt
    z_mu_right_2 = np.dot(np.transpose(R(nu)), K_1)*dt
    z_mu_right_3 = (nu - np.dot(np.linalg.pinv(K_2*dt + np.ones([3, 3])), np.dot(K_2, nu) * dt + z_nu_prev))
    z_mu_right = z_mu_right_1 + np.dot(z_mu_right_2, z_mu_right_3) + z_mu_prev
    z_mu_left_1 = np.ones([2, 2]) - A * dt
    z_mu_left_2 = np.dot(np.dot(np.transpose(R(nu)), K_1*dt), np.linalg.pinv(K_2*dt + np.ones([3, 3])))
    z_mu_left_3 = np.dot(B*dt, np.dot(K_e, L*dt) + K_mu)
    z_mu_left = np.linalg.pinv(z_mu_left_1 + np.dot(z_mu_left_2, R(nu) * dt) + z_mu_left_3)
    #print("nu =", nu)
    # print("dt = ", dt)
    # print("z_mu_right_1 =", z_mu_right_1)
    # print("z_mu_right_2 =", z_mu_right_2)
    # print("z_mu_right_3 =", z_mu_right_3)
    #print("z_mu_left =", z_mu_left)
    z_mu = np.dot(z_mu_left, z_mu_right)
    z_e = np.dot(L, z_mu)*dt + z_e_prev
    z_nu_1 = np.linalg.pinv(K_2 * dt + np.ones([3, 3]))
    z_nu_2 = np.dot(R(nu), z_mu) * dt + np.dot(K_2, nu) * dt + z_nu_prev
    #print("z_nu_1 =", z_nu_1)
    #print("z_nu_2 =", z_nu_2)
    print(dt)
    z_nu = np.dot(z_nu_1, z_nu_2)
    tau = -np.dot(K_mu, z_mu) - np.dot(K_e, z_e)
    return tau, z_nu, z_e, z_mu

def SimpleReg(v, e, L, K_mu=K_mu_def):
    K_e = np.transpose(L)
    tau = -np.dot(K_mu, v) - np.dot(K_e, e)
    print(np.dot(K_mu, v))
    print(np.dot(K_e, e))
    return tau

def F_1_0(nu, K_1=K_1_def):
    return -np.transpose(R(nu))*K_1


def R(nu):
    return np.asarray([[np.cos(nu[2]), 0.0], [np.sin(nu[2]), 0.0], [0.0, 1.0]])
