import numpy as np
import ros2connect.gazebo_connect as gc
import rclpy
import matplotlib.pyplot as plt
import time
import controller.graph as gr
from os.path import expanduser

home = expanduser("~")

folder = home + '/resources/'


def test(args=None):
    robot_namespace = '/turtlebot'
    rclpy.init(args=args)
    sub_clock = gc.ClockSubscriber()
    sub_vel = gc.SpeedSubscriber(robot_namespace)
    sub_joint = gc.JointSubscriber(robot_namespace)
    eff_pub = gc.EffortPublisher(robot_namespace=robot_namespace)
    vel_clock_exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    vel_clock_exec.add_node(sub_clock)
    vel_clock_exec.add_node(sub_vel)
    v = []
    t = []
    v_stop = []
    t_stop = []
    left = 0.0016
    right = 0.0023
    step = 0.0001
    k = 0.001
    #time.sleep(10)
    vel_clock_exec.spin_once()
    v.append(sub_vel.vel.linear.x)
    t.append(sub_clock.clock)
    eff_pub.pub(left, right)
    #print(sub_vel.vel.angular)
    while sub_vel.vel.linear.x < 1 or abs(sub_vel.vel.angular.z) > 0.01:
        right -= k * sub_vel.vel.angular.z
        left += k * sub_vel.vel.angular.z
        eff_pub.pub(left, right)
        time.sleep(0.05)
        vel_clock_exec.spin_once()
        rclpy.spin_once(sub_joint)
        v.append(sub_vel.vel.linear.x)
        t.append(sub_clock.clock)
    eff_pub.pub(0.0, 0.0)
    while sub_vel.vel.linear.x > 0:
        vel_clock_exec.spin_once()
        v_stop.append(sub_vel.vel.linear.x)
        t_stop.append(sub_clock.clock)
        time.sleep(0.05)
    gr.plotVel(v, t, name='vel_start')
    gr.plotVel(v_stop, t_stop, name='vel_stop')
    sub_clock.destroy_node()
    sub_vel.destroy_node()
    vel_clock_exec.shutdown()
    eff_pub.destroy_node()
    rclpy.shutdown()


def test2(args=None):
    robot_namespace = '/turtlebot'
    rclpy.init(args=args)
    sub_clock = gc.ClockSubscriber()
    sub_vel = gc.SpeedSubscriber(robot_namespace)
    eff_pub = gc.EffortPublisher(robot_namespace=robot_namespace)
    vel_clock_exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    vel_clock_exec.add_node(sub_clock)
    vel_clock_exec.add_node(sub_vel)
    vel_clock_exec.spin_once()
    left = 0.002
    right = 0.003
    k = 1.28274
    v = []
    t = []
    v_stop = []
    t_stop = []
    vel_clock_exec.spin_once()
    v.append(sub_vel.vel.linear.x)
    t.append(sub_clock.clock)
    #step = 0.00001
    eff_pub.pub(0, 0)
    eff_pub.pub(right*30, right*30)
    time.sleep(0.1)
    eff_pub.pub(k*right*17, right*15)
    time.sleep(0.1)
    eff_pub.pub(right*k, right)
    # while abs(sub_vel.vel.angular.z) > 0.0005:
    #     if sub_vel.vel.angular.z > 0:
    #         k += 0.00001
    #     elif sub_vel.vel.angular.z < 0:
    #         k -= 0.00001
    #     eff_pub.pub(right*k, right)
    #     time.sleep(5)
    #     vel_clock_exec.spin_once()
    #     print(k)
    #     print(sub_vel.vel.angular.z)
    while sub_vel.vel.linear.x < 1:
        vel_clock_exec.spin_once()
        v.append(sub_vel.vel.linear.x)
        t.append(sub_clock.clock)
        time.sleep(0.05)
    eff_pub.pub(0.0, 0.0)
    while sub_vel.vel.linear.x > 0:
        vel_clock_exec.spin_once()
        v_stop.append(sub_vel.vel.linear.x)
        t_stop.append(sub_clock.clock)
        time.sleep(0.05)
    gr.plotVel(v, t, name='vel_start')
    gr.plotVel(v_stop, t_stop, name='vel_stop')
    sub_clock.destroy_node()
    sub_vel.destroy_node()
    vel_clock_exec.shutdown()
    eff_pub.destroy_node()
    rclpy.shutdown()


# def test3(args=None):
#     robot_namespace = '/turtlebot'
#     rclpy.init(args=args)
#     sub_clock = gc.ClockSubscriber()
#     sub_vel = gc.SpeedSubscriber(robot_namespace)
#     eff_pub = gc.EffortPublisher(robot_namespace=robot_namespace)
#     vel_clock_exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
#     vel_clock_exec.add_node(sub_clock)
#     vel_clock_exec.add_node(sub_vel)
#     v = []
#     t = []
#     v_stop = []
#     t_stop = []
#     left = 0.0016
#     right = 0.05
#     step = 0.0001
#     k = 0.001
#     #time.sleep(10)
#     eff_pub.pub(0, right)
#     time.sleep(0.5)
#     eff_pub.pub(0, right)
#     time.sleep(0.5)
#     eff_pub.pub(0, right)
#     time.sleep(0.5)
#     time.sleep(5)
#     vel_clock_exec.spin_once()
#     v.append(sub_vel.vel.angular.z)
#     t.append(sub_clock.clock)
#     #print(sub_vel.vel.angular)
#     while abs(sub_vel.vel.angular.z) > 0.01:
#         right -= k * sub_vel.vel.angular.z
#         eff_pub.pub(0, right)
#         time.sleep(0.05)
#         vel_clock_exec.spin_once()
#         v.append(sub_vel.vel.angular.z)
#         t.append(sub_clock.clock)
#     eff_pub.pub(0.0, 0.0)
#     gr.plotVel(v, t, name='omega_wtf', linear=False)
#     sub_clock.destroy_node()
#     sub_vel.destroy_node()
#     vel_clock_exec.shutdown()
#     eff_pub.destroy_node()
#     rclpy.shutdown()


def test3(args=None):
    robot_namespace = '/turtlebot'
    rclpy.init(args=args)
    sub_clock = gc.ClockSubscriber()
    sub_vel = gc.SpeedSubscriber(robot_namespace)
    sub_joint = gc.JointSubscriber(robot_namespace)
    eff_pub = gc.EffortPublisher(robot_namespace=robot_namespace)
    vel_clock_exec = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    vel_clock_exec.add_node(sub_clock)
    vel_clock_exec.add_node(sub_vel)
    v = []
    t = []
    v_stop = []
    t_stop = []
    left = 0.0016 *2
    right = 0.0023 *2
    step = 0.0001
    k = 0.001
    nu = 0.0001
    #time.sleep(10)
    vel_clock_exec.spin_once()
    v.append(sub_vel.vel.linear.x)
    t.append(sub_clock.clock)
    eff_pub.pub(left, right)
    #print(sub_vel.vel.angular)
    while sub_vel.vel.linear.x < 0.9546 or abs(sub_vel.vel.angular.z) > 0.01:
        right -= k * sub_vel.vel.angular.z
        left += k * sub_vel.vel.angular.z
        #трение
        rclpy.spin_once(sub_joint)
        left_frict = left - nu * sub_joint.joint_state.velocity[0]
        right_frict = right - nu * sub_joint.joint_state.velocity[1]
        eff_pub.pub(left_frict, right_frict)
        time.sleep(0.05)
        vel_clock_exec.spin_once()
        v.append(sub_vel.vel.linear.x)
        print(sub_vel.vel.linear.x)
        t.append(sub_clock.clock)
    eff_pub.pub(0.0, 0.0)
    while sub_vel.vel.linear.x > 0:
        left_frict = -nu * sub_joint.joint_state.velocity[0]
        right_frict = -nu * sub_joint.joint_state.velocity[1]
        eff_pub.pub(left_frict, right_frict)
        vel_clock_exec.spin_once()
        v_stop.append(sub_vel.vel.linear.x)
        t_stop.append(sub_clock.clock)
        time.sleep(0.05)
    eff_pub.pub(0.0, 0.0)
    gr.plotVel(v, t, name='vel_start')
    gr.plotVel(v_stop, t_stop, name='vel_stop')
    sub_clock.destroy_node()
    sub_vel.destroy_node()
    vel_clock_exec.shutdown()
    eff_pub.destroy_node()
    rclpy.shutdown()


def stop(args=None):
    robot_namespace = '/turtlebot'
    rclpy.init(args=args)
    eff_pub = gc.EffortPublisher(robot_namespace=robot_namespace)
    eff_pub.pub(0.0, 0.0)
    time.sleep(0.2)
    eff_pub.pub(0.0, 0.0)
    time.sleep(0.2)
    eff_pub.pub(0.0, 0.0)
    time.sleep(0.2)
    eff_pub.pub(0.0, 0.0)
    eff_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__test__':
    test()


if __name__ == '__test2__':
    test2()

if __name__ == '__test3__':
    test()


if __name__ == '__stop__':
    stop()