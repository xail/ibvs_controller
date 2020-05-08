import numpy as np
import eff_reg as er

A_def = np.asarray([[0.9853, -0.0028], [0.0380, 0.6594]])
B_def = np.asarray([[-1.5032, 0.8525], [6.3636, -13.7924]])
K_1_def = 10 * np.ones([3, 3])
K_2_def = 5 * np.ones([3, 3])
K_mu_def = np.asarray([[1, 1], [1, 1]])/2000
#K_mu_def = np.asarray([[4, 0], [0, 8]])
H_def = 0.5 * np.ones([8, 8])
alpha_def = - np.ones([2, 2])
gamma_def = np.ones([2, 2])
K_mu_def_1 = np.asarray([[-9.8999, -0.5911],
                       [-4.5704, -1.0456]])
K_mu_def_2 = np.asarray([[-1.3404, -0.0620],
                        [-0.6212, -0.1127]])
K_mu_def_3 = np.asarray([[-0.7998, -0.0286],
                         [-0.3718, -0.0538]])
K_mu_def_4 = np.asarray([[-0.9980, -0.0955], [-0.4632, -0.1716]])
K_mu_def_5 = np.asarray([[-0.4741 ,  -0.0014],[   -0.0321,   -0.0058]])
K_mu_def_6 = np.asarray([[-0.4741, -0.0091],[-0.0321, -0.0194]])
L = np.zeros([52, 2])
z_e_prev = None
z_mu_prev = np.zeros(2)
z_nu_prev = np.zeros(3)
dt = 1.0
nu = np.array([0, 0, 0])

K = np.array([[804.5931813021568, 0.0, 640.5],
              [0.0, 804.5931813021568, 360.5],
              [0.0, 0.0, 1.0]])
P = np.array([[804.5931813021568, 0.0, 640.5, -56.32152269115098],
             [0.0, 804.5931813021568, 360.5, 0.0],
             [0.0, 0.0, 1.0, 0.0]])
R = np.array([[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]])

#K_inv = np.linalg.inv(K)
#R_inv = np.linalg.inv(R)
# m, z_nu, z_e, z_mu = er.Reg(dt, nu, L, z_nu_prev, z_e_prev, z_mu_prev)
#T = np.dot(K_inv, P)[:, 3].reshape([-1, 1])
#print(T)
#Buf = np.dot(K_inv, P) - np.array([np.ones, np.dot(K_inv, P)])
#P_new = np.append(R, np.zeros([3, 1]), axis=1)
#print(P_new)
print(np.linalg.eig(A_def-np.dot(B_def, K_mu_def_6)))
print(A_def-np.dot(B_def, K_mu_def_6))