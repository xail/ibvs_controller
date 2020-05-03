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

    K_e = np.transpose(L)
    z_mu = (B * z_e_prev * dt + np.transpose(R(nu))*K_1*dt*nu + (K_2 * nu * dt + z_nu_prev) / (1 + K_2 * dt) +
            z_mu_prev) / (1 - B * (-K_mu - K_e*L*dt) * dt - A * dt + np.transpose(R(nu))*K_1*dt*(R(nu) * dt) /
                          (1 + K_2 * dt))
    z_e = L*z_mu*dt + z_e_prev
    z_nu = (R(nu) * z_mu * dt) / (1 + K_2 * dt) + (K_2 * nu * dt + z_nu_prev) / (1 + K_2 * dt)
    tau = -K_mu*z_mu - K_e*z_e
    return [tau, z_nu, z_e, z_mu]


def F_1_0(nu, K_1=K_1_def):
    return -np.transpose(R(nu))*K_1



def R(nu):
    return [[np.cos(nu[2]),0],[np.sin(nu[2])],[0,1]]