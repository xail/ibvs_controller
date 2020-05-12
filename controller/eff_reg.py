import numpy as np
from scipy.integrate import odeint
import rclpy
from controller.model import *

#
# def Reg(dt, eta, L, z_eta_prev, z_e_prev, z_v_prev, e):
#     if z_e_prev is None:
#         z_e_prev = np.zeros([len(L)])
#     K_e = np.dot(np.linalg.inv(B), np.transpose(L))
#     H_e = 0.5 * np.identity(len(z_e_prev))
#     if len(e) > len(z_e_prev):
#         e = e[0:len(z_e_prev)]
#     z_mu_right_1 = - np.dot(B, np.dot(K_e, z_e_prev)) * dt
#     z_mu_right_2 = np.dot(np.transpose(R(eta)), K_1) * dt
#     z_mu_right_3 = (eta - np.dot(np.linalg.pinv(K_2 * dt + np.identity(3)), np.dot(K_2, eta) * dt + z_eta_prev))
#     z_mu_right = z_mu_right_1 + np.dot(z_mu_right_2, z_mu_right_3) + z_v_prev
#     z_mu_left_1 = np.identity(2) - A * dt
#     z_mu_left_2 = np.dot(np.dot(np.transpose(R(eta)), K_1 * dt), np.linalg.pinv(K_2 * dt + np.identity(3)))
#     z_mu_left_3 = np.dot(B*dt, np.dot(K_e, L*dt) + K_v)
#     z_mu_left = np.linalg.pinv(z_mu_left_1 + np.dot(z_mu_left_2, R(eta) * dt) + z_mu_left_3)
#     #print("nu =", eta)
#     # print("dt = ", dt)
#     # print("z_mu_right_1 =", z_mu_right_1)
#     # print("z_mu_right_2 =", z_mu_right_2)
#     # print("z_mu_right_3 =", z_mu_right_3)
#     #print("z_mu_left =", z_mu_left)
#     #print("z_mu_right =", z_mu_right)
#     z_v = np.dot(z_mu_left, z_mu_right)
#     z_e = np.dot(L, z_v)*dt + z_e_prev + np.dot(H_e, e - z_e_prev)*dt
#     z_eta_1 = np.linalg.pinv(K_2 * dt + np.identity(3))
#     z_eta_2 = np.dot(R(eta), z_v) * dt + np.dot(K_2, eta) * dt + z_eta_prev
#     #print("z_nu_1 =", z_nu_1)
#     #print("z_nu_2 =", z_nu_2)
#     #print(dt)
#     z_eta = np.dot(z_eta_1, z_eta_2)
#     print('z_mu=', z_v)
#     print('z_e=', z_e[0:10])
#     print('e=', - np.dot(K_e, z_e))
#     print('V=', -np.dot(K_v, z_v))
#     print('nu - z_nu = ', eta - z_eta)
#     print("nu =", eta)
#     print("z_nu =", z_eta)
#     print('dt =', dt)
#     tau = -np.dot(K_mu, z_mu) - np.dot(K_e, z_e)
#     print("tau1=", tau)
#     #tau[1] *= 0.7
#     print("tau2=", tau)
#     print(' ')
#     return tau, z_eta, z_e, z_mu


def SimpleReg(v, e, L):
    k = 2
    # K_e = k * np.dot(np.linalg.inv(B), np.linalg.pinv(L))
    #K_e = k * np.transpose(L)
    K_e = k * np.dot(np.linalg.pinv(B), np.linalg.pinv(L))
    #print('v = ', v)
    tau = -np.dot(K_v, v) - np.dot(K_e, e)
    #print('Kv*v=', -np.dot(K_v, v))
    #print('Ke*e=', - np.dot(K_e, e))
    #print('m =', tau)
    return tau


def Reg_dist(L, z_eta_prev, z_e_prev, z_v_prev, tau_prev, clock_sub, pos_sub, e, p_prev, L0):
    k = 0.5
    # K_e = np.dot(k, np.dot(np.linalg.pinv(B), np.linalg.pinv(L)))
    K_e = np.dot(k, np.linalg.pinv(L))
    # K_e = np.dot(np.linalg.pinv(B), np.linalg.pinv(L))
    # K_e = k * np.dot(np.linalg.pinv(B), np.transpose(Le))
    H_e = 0.5 * np.identity(len(e))
    rclpy.spin_once(clock_sub)
    while clock_sub.clock - clock_sub.prev_clock <= 0.0:
        rclpy.spin_once(clock_sub)
    rclpy.spin_once(pos_sub)
    t = np.linspace(0, clock_sub.clock - clock_sub.prev_clock, 10)
    #print('eta = ', pos_sub.eta)
    # z_e = e
    #z_e_prev = e
    z_v_buff = odeint(d_z_v, z_v_prev, t, args=(tau_prev, z_eta_prev, pos_sub.eta))
    z_v = z_v_buff[len(t)-1]
    #print('z_v = ', z_v)
    z_eta_buff = odeint(d_z_eta, z_eta_prev, t, args=(z_v, pos_sub.eta))
    z_eta = z_eta_buff[len(t) - 1]
    #print('z_eta = ', z_eta)
    z_e_buff = odeint(d_z_e, z_e_prev, t, args=(z_v, L, e, H_e))
    z_e = z_e_buff[len(t) - 1]
    p_buff = odeint(d_p, p_prev, t, args=(pos_sub.eta, z_eta, e, z_e, L0, K_e, H_e))
    p = p_buff[len(t) - 1]
    # p = np.zeros(2)
    # print('p = ', p)
    xi = gamma.dot(p) #+ mu_eta * (pos_sub.eta - z_eta) + mu_e * (e - z_e)
    #print( 'xi = ', xi)
    #print('np.dot(K_v, z_v) = ',np.dot(K_v, z_v))
    #print('np.dot(K_e, z_e) = ',np.dot(K_e, z_e))
    tau = -np.dot(K_v, z_v) - np.dot(K_e, z_e) + xi
    #print('m =', tau)
    if tau[0] > 1.5:
        tau[0] = 1.5
    if tau[1] > 1.5:
        tau[1] = 1.5
    # if abs(tau[0] - tau_prev[0]) > 0.1:
    #     tau[0] = 0.1 * np.sign(tau[0] - tau_prev[0])
    # if abs(tau[1] - tau_prev[1]) > 0.1:
    #     tau[1] = 0.1 * np.sign(tau[1] - tau_prev[1])
    return tau, z_eta, z_e, z_v, p, np.dot(K_e, z_e)


def d_z_v(z_v, t, tau_prev, z_eta_prev, eta):
    dz_v = A.dot(z_v) + B.dot(tau_prev) + np.dot(np.transpose(R(eta)).dot(K_1), eta - z_eta_prev)
    return dz_v


def d_z_eta(z_eta, t, z_v, eta):
    dz_eta = np.dot(R(eta), z_v) + K_2.dot(eta - z_eta)
    return dz_eta


def d_z_e(z_e, t, z_v, L, e, H_e):
    dz_e = np.dot(L, z_v) + H_e.dot(e - z_e)
    return dz_e


def d_p(p, t, eta, z_eta, e, z_e, L, K_e, H_e):
    beta_eta = F_1(nu0)
    beta_e = F_2(L, K_e, H_e)
    dp = alpha.dot(p) + beta_eta.dot(eta - z_eta) + beta_e.dot(e - z_e)
    return dp


