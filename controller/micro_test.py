import numpy as np
import eff_reg as er

A_def = np.asarray([[0.9853, -0.0028], [0.0380, 0.6594]])
B_def = np.asarray([[-1.5032, 0.8525], [6.3636, -13.7924]])
K_1_def = 10 * np.ones([3, 3])
K_2_def = 5 * np.ones([3, 3])
K_mu_def = np.asarray([[8, 0], [0, 20]])
H_def = 0.5 * np.ones([8, 8])
alpha_def = - np.ones([2, 2])
gamma_def = np.ones([2, 2])

L = np.zeros([52, 2])
z_e_prev = None
z_mu_prev = np.zeros(2)
z_nu_prev = np.zeros(3)
dt = 1.0
nu = np.array([0, 0, 0])


m, z_nu, z_e, z_mu = er.Reg(dt, nu, L, z_nu_prev, z_e_prev, z_mu_prev)
print(np.zeros([2]).shape)