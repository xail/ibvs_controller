from scipy.integrate import odeint
import numpy as np
import rclpy
from controller.model import *


def motor(tau, clock_sub, vel_sub):
    rclpy.spin_once(clock_sub)
    # while clock_sub.clock - clock_sub.prev_clock <= 0.0:
    #     rclpy.spin_once(clock_sub)
    # rclpy.spin_once(vel_sub)
    t = np.linspace(clock_sub.prev_clock, clock_sub.clock, 10)
    v = odeint(motor_model, vel_sub.vel2, t, args=(tau[0], tau[1]))
    clock_sub.prev_clock = clock_sub.clock
    #print(v[9])
    return v[9]


def motor_model(v, t, tau_left, tau_right):
    tau = [tau_left, tau_right]
    dv_dt = np.dot(A, v) + np.dot(B, tau)
    return dv_dt

